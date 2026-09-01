"""Tests for the obstacle system (docs/OBSTACLES.md, Part 1).

Covers: placement validation, lane-direction auto-alignment (two-way halves
and one-ways), move/delete, the stop-on-contact physics (full braking, no
interpenetration, no teleports), JSON save/load with per-map scoping and
off-road validation on load, and the REST endpoints (via Flask test client).

Run headless from the project root:
    python -m pytest tests/test_obstacles.py -v
"""

import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src import config
from src.car import Car
from src.obstacles import (ObstacleManager, PlacementError, boxes_intersect,
                           box_corners, lane_heading_at, obstacle_footprint,
                           player_body_corners, point_in_box)
from src.test_maps import build_test_map

PPPM = config.PIXELS_PER_METER
DT = 1.0 / 60.0


def make_network():
    return build_test_map("basic")


class TestPlacement(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.network = make_network()

    def setUp(self):
        # Fresh manager per test: ids start at 1 every time.
        self.mgr = ObstacleManager("basic", base_dir="/tmp/obst-test-unused")

    def test_place_on_road_gets_id_and_heading(self):
        # Straight tile: segment runs north (y=350 m) -> south (y=50 m), so
        # the direction of travel is SOUTH (heading 180). Centerline at
        # x = 100 m = 200 px; road width 7 m.
        ob = self.mgr.place(self.network, "car", "blue", 97 * PPPM, 100 * PPPM)
        self.assertEqual(ob.id, 1)
        self.assertEqual(ob.type, "car")
        self.assertEqual(ob.color, "blue")
        self.assertAlmostEqual(ob.heading, 180.0, places=5)

    def test_alignment_two_way_halves(self):
        # Right half of the lane (facing south, right = west side):
        # a car dropped there faces forward (south, heading 180).
        h_right = lane_heading_at(97 * PPPM, 100 * PPPM, self.network)
        # Left/oncoming half (east side of the centerline): faces north.
        h_left = lane_heading_at(103 * PPPM, 100 * PPPM, self.network)
        self.assertAlmostEqual(h_right, 180.0, places=5)
        self.assertAlmostEqual(h_left, 0.0, places=5)

    def test_alignment_one_way_always_legal_direction(self):
        # One-way tile: ow_w -> ow_center flows WEST -> EAST (heading 90).
        # Both sides of the centerline face the legal direction.
        h_north = lane_heading_at(1100 * PPPM, 719 * PPPM, self.network)
        h_south = lane_heading_at(1100 * PPPM, 721 * PPPM, self.network)
        self.assertAlmostEqual(h_north, 90.0, places=5)
        self.assertAlmostEqual(h_south, 90.0, places=5)

    def test_offroad_rejected(self):
        # 22 m west of the centerline - far beyond the 3.5 m half-width.
        with self.assertRaises(PlacementError):
            self.mgr.place(self.network, "car", "blue", 78 * PPPM, 100 * PPPM)

    def test_bad_type_and_color_rejected(self):
        with self.assertRaises(PlacementError):
            self.mgr.place(self.network, "truck", "blue", 97 * PPPM, 100 * PPPM)
        with self.assertRaises(PlacementError):
            self.mgr.place(self.network, "car", "red", 97 * PPPM, 100 * PPPM)

    def test_move_realigns_and_remove_works(self):
        ob = self.mgr.place(self.network, "car", "yellow", 97 * PPPM, 100 * PPPM)
        self.assertAlmostEqual(ob.heading, 180.0, places=5)
        # Move to the oncoming half of a different part of the road:
        # re-placed exactly where dropped and re-aligned (now north).
        moved = self.mgr.move(self.network, ob.id, 103 * PPPM, 120 * PPPM)
        self.assertAlmostEqual(moved.x, 103 * PPPM, places=9)
        self.assertAlmostEqual(moved.heading, 0.0, places=5)
        # The original instance is replaced (frozen dataclass swap).
        self.assertIsNot(self.mgr.get(ob.id), ob)
        self.assertIsNone(self.mgr.get(999))
        self.assertTrue(self.mgr.remove(ob.id))
        self.assertFalse(self.mgr.remove(ob.id))
        with self.assertRaises(KeyError):
            self.mgr.move(self.network, ob.id, 97 * PPPM, 100 * PPPM)

    def test_move_offroad_rejected_and_unchanged(self):
        ob = self.mgr.place(self.network, "car", "white", 97 * PPPM, 100 * PPPM)
        with self.assertRaises(PlacementError):
            self.mgr.move(self.network, ob.id, 78 * PPPM, 100 * PPPM)
        self.assertAlmostEqual(self.mgr.get(ob.id).x, 97 * PPPM, places=9)


class TestCurveAlignment(unittest.TestCase):
    """Placed cars must follow the SMOOTHED centerline on curves and
    rounded corners (the reported bug: they snapped to one of the two
    straight segment chords instead - e2e test 1's corner jumped
    180 -> 270, test 2's corner jumped 180 -> 0 -> 270 -> 90)."""

    @classmethod
    def setUpClass(cls):
        cls.network = make_network()
        from src.smooth_geometry import smoothed_network
        cls.sm = smoothed_network(cls.network)

    def _line_near_node(self, node_name, slack_px=15.0):
        n_c = self.network.nodes[node_name]
        return next(L for L in self.sm.lines if any(
            abs(c[0] - n_c[0]) < slack_px and abs(c[1] - n_c[1]) < slack_px
            for c in L["coords"]))

    def _corner_sweep(self, node_name):
        """(px, py, expected_heading) at 1.5 m right of the paved
        centerline, swept 8 m before -> 8 m after the corner."""
        line = self._line_near_node(node_name)
        curve = line["curve"]
        n_c = self.network.nodes[node_name]
        best_s, best_d = 0.0, float("inf")
        for i in range(len(curve._s)):
            x, y = curve.point_at(curve._s[i])
            d2 = (x - n_c[0]) ** 2 + (y - n_c[1]) ** 2
            if d2 < best_d:
                best_d, best_s = d2, curve._s[i]
        out = []
        for off in range(-8, 9, 2):
            s = max(0.0, min(curve.total, best_s + off))
            qx, qy = curve.point_at(s)
            h = math.radians(curve.heading_at(s))
            rx, ry = math.cos(h), -math.sin(h)      # right of travel
            out.append((qx + 1.5 * PPPM * rx, qy + 1.5 * PPPM * ry,
                        curve.heading_at(s)))
        return out

    def _assert_follows_road(self, points, tol_deg=4.0):
        for px, py, expected in points:
            self.assertTrue(self.network.is_on_road(px, py))
            h = lane_heading_at(px, py, self.network)
            diff = abs(((h - expected + 180.0) % 360.0) - 180.0)
            self.assertLessEqual(
                diff, tol_deg,
                f"heading {h:.1f} at ({px / PPPM:.1f} m, {py / PPPM:.1f} m) "
                f"should follow the road tangent {expected:.1f}")

    def test_alignment_follows_right_corner(self):
        # cornerR (e2e test 1's curve): south -> west 90 deg. The heading
        # must sweep continuously through ~225 instead of jumping 180->270.
        pts = self._corner_sweep("cornerR_c")
        self._assert_follows_road(pts)
        hs = [lane_heading_at(px, py, self.network) for px, py, _ in pts]
        self.assertGreater(max(hs) - min(hs), 60.0,
                           "heading should sweep around the corner")

    def test_alignment_follows_left_corner(self):
        # cornerL (e2e test 2's curve): south -> east. The old chord logic
        # jumped 180 -> 0 -> 270 -> 90 here; now it must sweep ~180..~90.
        pts = self._corner_sweep("cornerL_c")
        self._assert_follows_road(pts)
        hs = [lane_heading_at(px, py, self.network) for px, py, _ in pts]
        self.assertGreater(max(hs) - min(hs), 60.0,
                           "heading should sweep around the corner")

    def test_alignment_follows_roundabout_ring(self):
        # One-way ring: a car anywhere on it faces the local tangent in
        # the legal (counter-clockwise) flow direction.
        line = self._line_near_node("rb_r8", slack_px=50.0)
        curve = line["curve"]
        for frac in (0.2, 0.55, 0.8):
            s = curve.total * frac
            qx, qy = curve.point_at(s)
            h = math.radians(curve.heading_at(s))
            rx, ry = math.cos(h), -math.sin(h)
            px, py = qx + 1.5 * PPPM * rx, qy + 1.5 * PPPM * ry
            got = lane_heading_at(px, py, self.network)
            expected = curve.heading_at(s)
            diff = abs(((got - expected + 180.0) % 360.0) - 180.0)
            self.assertLessEqual(
                diff, 4.0,
                f"ring heading {got:.1f} at s={s:.0f} m should follow "
                f"the tangent {expected:.1f}")


class TestGeometry(unittest.TestCase):
    def test_boxes_overlap(self):
        a = box_corners(0, 0, 0, 4.5, 2.0)
        b = box_corners(1 * PPPM, 0, 0, 4.5, 2.0)
        self.assertTrue(boxes_intersect(a, b))

    def test_boxes_separated(self):
        a = box_corners(0, 0, 0, 4.5, 2.0)
        b = box_corners(10 * PPPM, 0, 0, 4.5, 2.0)
        self.assertFalse(boxes_intersect(a, b))

    def test_touching_edges_count_as_contact(self):
        # Both face north (heading 0): LENGTH runs along y, WIDTH (2 m) along
        # x. A's east edge is at x = +1 m; B centered at x = +2 m has its
        # west edge exactly on it.
        a = box_corners(0, 0, 0, 4.5, 2.0)
        b = box_corners(2.0 * PPPM, 0, 0, 4.5, 2.0)
        self.assertTrue(boxes_intersect(a, b))

    def test_rotated_overlap(self):
        a = box_corners(0, 0, 0, 4.5, 2.0)
        b = box_corners(1 * PPPM, 1 * PPPM, 90, 4.5, 2.0)   # T-shape overlap
        self.assertTrue(boxes_intersect(a, b))

    def test_point_in_box(self):
        a = box_corners(0, 0, 45, 4.5, 2.0)
        self.assertTrue(point_in_box(0, 0, a))
        self.assertFalse(point_in_box(10 * PPPM, 10 * PPPM, a))


class TestContactStop(unittest.TestCase):
    """The per-frame stop-on-contact response (all modes)."""

    @classmethod
    def setUpClass(cls):
        cls.network = make_network()

    def _fresh(self):
        mgr = ObstacleManager("basic", base_dir="/tmp/obst-test-unused")
        # Car on the straight tile heading south, 10 m/s.
        car = Car(100 * PPPM, 340 * PPPM, 180.0, seg_idx=0)
        car.speed = 10.0
        # Parked car directly ahead in the same lane.
        mgr.place(self.network, "car", "blue", 100 * PPPM, 250 * PPPM)
        return mgr, car

    def _overlap_area_px2(self, car, mgr):
        from shapely.geometry import Polygon
        a = Polygon(player_body_corners(car))
        worst = 0.0
        for ob in mgr.snapshot():
            worst = max(worst,
                        a.intersection(Polygon(obstacle_footprint(ob))).area)
        return worst

    def test_drives_into_obstacle_brakes_and_rests(self):
        mgr, car = self._fresh()
        for _ in range(60 * 25):            # 25 s at 60 Hz - plenty
            pre_x, pre_y = car.x, car.y
            v_pre = car.speed
            rad = math.radians(car.heading)
            car.x += math.sin(rad) * car.speed * DT * PPPM
            car.y += math.cos(rad) * car.speed * DT * PPPM
            mgr.apply_contact_stop(car, DT, pre_x, pre_y)
            # No interpenetration at the end of ANY step.
            self.assertLess(self._overlap_area_px2(car, mgr), 1e-9)
            # No teleport: motion never exceeds what the pre-step speed allows.
            moved = math.hypot(car.x - pre_x, car.y - pre_y) / PPPM
            self.assertLessEqual(moved, abs(v_pre) * DT + 1e-9)

        self.assertEqual(car.speed, 0.0)    # braked to a stop (A_BRAKE)
        # Resting against the obstacle: front bumper at its rear edge.
        rad = math.radians(car.heading)
        bx, by = car.body_center()
        front_y = by - (config.CAR_LENGTH / 2.0) * PPPM   # heading south
        ob = mgr.snapshot()[0]
        rear_edge_y = ob.y + (config.CAR_LENGTH / 2.0) * PPPM
        gap_m = (front_y - rear_edge_y) / PPPM
        self.assertGreaterEqual(gap_m, -1e-6)     # never inside it
        self.assertLess(gap_m, 0.5)               # ...and resting against it

    def test_high_speed_contact_with_heading_drift(self):
        # Regression (live pass-through bug): the car approaches at full
        # speed while its heading micro-drifts each substep, as BICYCLE nav
        # does (it always steers in tiny increments toward the reference
        # line). The first contact pins the car to the contact point; on
        # every following substep the pre-step box re-registers as touching
        # because the body rotated by ~REAR_AXLE_OFFSET*dh around the axle
        # offset. The response must HOLD the car at the pre-step position
        # (wall behavior) - not let it plough through while braking.
        mgr = ObstacleManager("basic", base_dir="/tmp/obst-test-unused")
        car = Car(100 * PPPM, 340 * PPPM, 180.0, seg_idx=0)
        car.speed = 15.0                      # ~54 km/h - the live approach speed
        mgr.place(self.network, "car", "blue", 100 * PPPM, 250 * PPPM)
        # Nav-style heading: oscillates +-0.2 deg around the line (live
        # BICYCLE nav always steers in ~0.01-0.02 deg increments but
        # self-corrects, so net rotation over the approach is ~0 - the car
        # still drives straight into the obstacle).
        for step in range(60 * 25):
            pre_x, pre_y, pre_h = car.x, car.y, car.heading
            v_pre = car.speed
            rad = math.radians(car.heading)
            car.x += math.sin(rad) * car.speed * DT * PPPM
            car.y += math.cos(rad) * car.speed * DT * PPPM
            car.heading = (180.0 + 0.2 * math.sin(step / 9.0)) % 360.0
            mgr.apply_contact_stop(car, DT, pre_x, pre_y, pre_h)
            # No interpenetration at the end of ANY step.
            self.assertLess(self._overlap_area_px2(car, mgr), 1e-9)
            if car.speed == 0.0:
                break
        self.assertEqual(car.speed, 0.0)      # braked to a stop (A_BRAKE)
        # Rested against the obstacle - NOT through it: the axle must still
        # be on the approach side of the obstacle's near edge.
        ob = mgr.snapshot()[0]
        near_edge_y = ob.y + (config.CAR_LENGTH / 2.0) * PPPM   # heading south
        self.assertGreater(car.y, near_edge_y)
        # And actually resting against it (front bumper within a hair of the
        # obstacle's rear edge), not stopped short by the braking alone.
        bx, by = car.body_center()
        front_y = by - (config.CAR_LENGTH / 2.0) * PPPM
        gap_m = (front_y - near_edge_y) / PPPM
        self.assertGreaterEqual(gap_m, -1e-6)
        self.assertLess(gap_m, 0.05)

    def test_no_contact_means_no_intervention(self):
        mgr, car = self._fresh()
        # Move the obstacle far away laterally (still on the road).
        ob = mgr.get(1)
        mgr.move(self.network, ob.id, 97 * PPPM, 250 * PPPM)
        pre_x, pre_y = car.x, car.y
        rad = math.radians(car.heading)
        car.x += math.sin(rad) * car.speed * DT * PPPM
        car.y += math.cos(rad) * car.speed * DT * PPPM
        self.assertFalse(mgr.apply_contact_stop(car, DT, pre_x, pre_y))
        self.assertEqual(car.speed, 10.0)         # untouched

    def test_contact_detected(self):
        mgr, car = self._fresh()
        # Drive the car into the obstacle (bodies clearly overlapping).
        ob = mgr.get(1)
        rad = math.radians(ob.heading)
        gap = (config.CAR_LENGTH + config.CAR_LENGTH) / 2.0 * PPPM + 0.5
        car.x = ob.x - math.sin(rad) * gap
        car.y = ob.y - math.cos(rad) * gap
        car.heading = ob.heading
        self.assertIsNotNone(mgr.contact_with_car(car))
        # And clear again once it is backed away.
        car.x -= math.sin(rad) * 3.0 * PPPM
        car.y -= math.cos(rad) * 3.0 * PPPM
        self.assertIsNone(mgr.contact_with_car(car))


class TestSaveLoad(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.network = make_network()

    def test_roundtrip(self):
        import tempfile
        tmp_dir = tempfile.mkdtemp(prefix="obst_layouts_")
        mgr = ObstacleManager("basic", base_dir=tmp_dir)
        p1 = mgr.place(self.network, "car", "blue", 97 * PPPM, 100 * PPPM)
        p2 = mgr.place(self.network, "car", "yellow", 103 * PPPM, 120 * PPPM)
        p3 = mgr.place(self.network, "car", "white", 97 * PPPM, 140 * PPPM)

        path = mgr.save("test layout")
        self.assertTrue(os.path.isfile(path))
        self.assertEqual(
            os.path.dirname(path),
            os.path.join(tmp_dir, "obstacles", "basic"))
        with open(path) as f:
            payload = json.load(f)
        self.assertEqual(payload["map"], "basic")
        self.assertEqual(len(payload["obstacles"]), 3)

        # A fresh manager loads the same set (ids preserved).
        mgr2 = ObstacleManager("basic", base_dir=tmp_dir)
        loaded, skipped = mgr2.load("test layout", self.network)
        self.assertEqual((loaded, skipped), (3, 0))
        got = {o.id: o for o in mgr2.snapshot()}
        for orig in (p1, p2, p3):
            self.assertIn(orig.id, got)
            self.assertEqual(got[orig.id].to_dict(), orig.to_dict())
        # New placements continue the id sequence.
        nxt = mgr2.place(self.network, "car", "blue", 97 * PPPM, 160 * PPPM)
        self.assertEqual(nxt.id, 4)

    def test_layouts_are_map_scoped(self):
        import tempfile
        tmp_dir = tempfile.mkdtemp(prefix="obst_layouts_")
        mgr = ObstacleManager("basic", base_dir=tmp_dir)
        mgr.place(self.network, "car", "blue", 97 * PPPM, 100 * PPPM)
        mgr.save("only_basic")
        other = ObstacleManager("othermap", base_dir=tmp_dir)
        self.assertEqual(other.list_layouts(), [])
        with self.assertRaises(FileNotFoundError):
            other.load("only_basic", self.network)

    def test_load_validates_against_paved_polygon(self):
        import tempfile
        tmp_dir = tempfile.mkdtemp(prefix="obst_layouts_")
        d = os.path.join(tmp_dir, "obstacles", "basic")
        os.makedirs(d)
        payload = {
            "map": "basic",
            "name": "mixed",
            "saved_at": "2026-01-01T00:00:00+00:00",
            "obstacles": [
                # on the road (right half of the straight tile)
                {"id": 1, "type": "car", "color": "blue",
                 "x": 97 * PPPM, "y": 100 * PPPM, "heading": 180.0},
                # no longer on the road (22 m off the centerline)
                {"id": 2, "type": "car", "color": "yellow",
                 "x": 78 * PPPM, "y": 100 * PPPM, "heading": 180.0},
                # malformed (no y)
                {"id": 3, "type": "car", "color": "white",
                 "x": 97 * PPPM, "heading": 180.0},
            ],
        }
        with open(os.path.join(d, "mixed.json"), "w") as f:
            json.dump(payload, f)
        mgr = ObstacleManager("basic", base_dir=tmp_dir)
        loaded, skipped = mgr.load("mixed", self.network)
        self.assertEqual((loaded, skipped), (1, 2))
        remaining = mgr.snapshot()
        self.assertEqual([o.id for o in remaining], [1])

    def test_save_overwrites_same_name(self):
        import tempfile
        tmp_dir = tempfile.mkdtemp(prefix="obst_layouts_")
        mgr = ObstacleManager("basic", base_dir=tmp_dir)
        mgr.place(self.network, "car", "blue", 97 * PPPM, 100 * PPPM)
        path = mgr.save("dup")
        mgr.remove(1)
        mgr.save("dup")
        with open(path) as f:
            self.assertEqual(len(json.load(f)["obstacles"]), 0)


class TestRestAPI(unittest.TestCase):
    """The /obstacles endpoints, via the Flask test client (no server)."""

    @classmethod
    def setUpClass(cls):
        cls.network = make_network()
        import tempfile
        cls.tmp_dir = tempfile.mkdtemp(prefix="obst_api_")
        from src.rest_api import GameAPI
        cls.mgr = ObstacleManager("basic", base_dir=cls.tmp_dir)
        api = GameAPI()
        api.set_obstacles(cls.mgr, cls.network)
        cls.client = api.app.test_client()

    def test_place_list_delete(self):
        r = self.client.get("/obstacles")
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.get_json(), [])

        # Place on the road: 201 + id + computed heading.
        r = self.client.post("/obstacles", json={
            "type": "car", "color": "blue",
            "x": 97 * PPPM, "y": 100 * PPPM})
        self.assertEqual(r.status_code, 201)
        ob = r.get_json()
        self.assertEqual(ob["id"], 1)
        self.assertAlmostEqual(ob["heading"], 180.0, places=5)

        # List reflects it.
        r = self.client.get("/obstacles")
        self.assertEqual(r.get_json(), [ob])

        # Off-road: 4xx.
        r = self.client.post("/obstacles", json={
            "type": "car", "color": "blue",
            "x": 78 * PPPM, "y": 100 * PPPM})
        self.assertEqual(r.status_code, 400)
        # Invalid color: 4xx.
        r = self.client.post("/obstacles", json={
            "type": "car", "color": "red",
            "x": 97 * PPPM, "y": 100 * PPPM})
        self.assertEqual(r.status_code, 400)
        # Malformed body: 4xx.
        r = self.client.post("/obstacles", json={"color": "blue"})
        self.assertEqual(r.status_code, 400)

        # Delete; unknown id -> 404.
        r = self.client.delete("/obstacles/1")
        self.assertEqual(r.status_code, 200)
        self.assertEqual(self.client.get("/obstacles").get_json(), [])
        r = self.client.delete("/obstacles/1")
        self.assertEqual(r.status_code, 404)


if __name__ == "__main__":
    unittest.main()
