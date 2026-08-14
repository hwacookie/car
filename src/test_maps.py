# Synthetic Test Maps
# Deterministic, hand-crafted road networks for reproducible testing.
# Selected via --map <name> command line flag.

from __future__ import annotations

import math
from . import config
from .road_network import RoadNetwork, RoadSegment


class MapBuilder:
    """Helper to construct a RoadNetwork from named nodes given in METERS.

    Coordinates given in meters are converted to world pixels via
    config.PIXELS_PER_METER. Y increases "south"/downward in this
    synthetic coordinate system, consistent with the OSM-derived maps
    (world Y grows away from the northern edge).
    """

    def __init__(self):
        self.pppm = config.PIXELS_PER_METER
        self._nodes_m: dict[str, tuple[float, float]] = {}
        self._segments: list[RoadSegment] = []
        self._next_seg_id = 1
        self._start_points: dict[str, str] = {}  # name -> node_id (must be degree-1)

    def node(self, node_id: str, x_m: float, y_m: float) -> None:
        """Define a node position in meters."""
        self._nodes_m[node_id] = (x_m, y_m)

    def road(
        self,
        n1: str,
        n2: str,
        highway: str = "residential",
        oneway: bool = False,
    ) -> None:
        """Add a road segment between two named nodes."""
        x1_m, y1_m = self._nodes_m[n1]
        x2_m, y2_m = self._nodes_m[n2]
        pppm = self.pppm

        x1, y1 = x1_m * pppm, y1_m * pppm
        x2, y2 = x2_m * pppm, y2_m * pppm

        road_cfg = config.ROAD_TYPES.get(highway)
        if road_cfg:
            width = road_cfg["width_1way"] if oneway else road_cfg["width_2way"]
        else:
            width = 3.5

        length_m = math.hypot(x2_m - x1_m, y2_m - y1_m)

        self._segments.append(RoadSegment(
            id=self._next_seg_id,
            x1=x1, y1=y1, x2=x2, y2=y2,
            highway=highway,
            oneway=oneway,
            width=width,
            start_node=n1,
            end_node=n2,
            length=length_m,
        ))
        self._next_seg_id += 1

    def start(self, name: str, node_id: str) -> None:
        """Register a named, deterministic start point at a node.

        The node must have exactly one connected segment (a scenario's
        entry point) so the car's initial heading and direction of travel
        are unambiguous. Use this to give tests a reproducible spawn
        location + heading approaching a specific junction.
        """
        self._start_points[name] = node_id

    def build(self, margin_m: float = 50.0) -> RoadNetwork:
        """Finalize and return the RoadNetwork."""
        pppm = self.pppm

        nodes = {nid: (x * pppm, y * pppm) for nid, (x, y) in self._nodes_m.items()}

        # node_connections + node_degree
        node_connections: dict[str, list[int]] = {}
        node_degree: dict[str, int] = {}
        for idx, seg in enumerate(self._segments):
            node_connections.setdefault(seg.start_node, []).append(idx)
            node_connections.setdefault(seg.end_node, []).append(idx)
            node_degree[seg.start_node] = node_degree.get(seg.start_node, 0) + 1
            node_degree[seg.end_node] = node_degree.get(seg.end_node, 0) + 1

        # node_max_width (skip degree-2 straight nodes, like from_osm_data does)
        node_info: dict[str, tuple[float, str]] = {}
        for seg in self._segments:
            half = (seg.width / 2) * pppm
            for nid in (seg.start_node, seg.end_node):
                cur = node_info.get(nid)
                if cur is None or half > cur[0]:
                    node_info[nid] = (half, seg.highway)
        node_info = {
            nid: info for nid, info in node_info.items()
            if node_degree.get(nid, 0) != 2
        }

        # Bounds: extent of all nodes + margin
        xs = [x for x, y in self._nodes_m.values()]
        ys = [y for x, y in self._nodes_m.values()]
        max_x = (max(xs) + margin_m) * pppm if xs else 1000
        max_y = (max(ys) + margin_m) * pppm if ys else 1000

        # Resolve named start points: (x, y, heading, seg_idx, forward)
        # The named node must have exactly one connected segment so the
        # direction of travel (heading, forward flag) is unambiguous.
        start_points: dict[str, tuple[float, float, float, int, bool]] = {}
        for name, node_id in self._start_points.items():
            connected = node_connections.get(node_id, [])
            if len(connected) != 1:
                raise ValueError(
                    f"Start point '{name}' -> node '{node_id}' must have exactly "
                    f"1 connected segment, found {len(connected)}"
                )
            seg_idx = connected[0]
            seg = self._segments[seg_idx]
            forward = (seg.start_node == node_id)
            x, y = (seg.x1, seg.y1) if forward else (seg.x2, seg.y2)
            dx = seg.x2 - seg.x1 if forward else seg.x1 - seg.x2
            dy = seg.y2 - seg.y1 if forward else seg.y1 - seg.y2
            heading = math.degrees(math.atan2(dx, dy))
            start_points[name] = (x, y, heading, seg_idx, forward)

        return RoadNetwork(
            nodes=nodes,
            segments=self._segments,
            origin_lat=0.0,
            origin_lon=0.0,
            world_width=max_x,
            world_height=max_y,
            node_connections=node_connections,
            node_degree=node_degree,
            node_max_width=node_info,
            start_points=start_points,
        )


def build_basic_test_map() -> RoadNetwork:
    """A comprehensive synthetic test track with known geometry.

    Layout: 4x2 grid of 400m x 400m tiles (500m pitch), each containing
    one specific test scenario:

        Tile (0,0): Straight road (baseline / acceleration test)
        Tile (1,0): 90 deg turn (corner A)
        Tile (2,0): 90 deg turn (corner B, mirrored)
        Tile (3,0): T-junction (3-way, perpendicular)
        Tile (0,1): Y-intersection (3-way, shallow diverging angles)
        Tile (1,1): 4-way intersection (crossroads)
        Tile (2,1): One-way street through a 4-way junction
        Tile (3,1): S-curve (gentle degree-2 bends)
        Tile (0,2): Dead-end
        Tile (1,2): Tight hairpin turn (narrow angle, low speed)
        Tile (2,2): Wide sweeping curve (gentle single bend)
    """
    b = MapBuilder()
    TILE = 500.0  # pitch between tiles, meters

    def origin(col: int, row: int) -> tuple[float, float]:
        return col * TILE, row * TILE

    # --- Tile (0,0): Straight road ---
    ox, oy = origin(0, 0)
    b.node("straight_n", ox + 100, oy + 50)
    b.node("straight_s", ox + 100, oy + 350)
    b.road("straight_n", "straight_s")
    b.start("straight", "straight_n")
    b.start("straight_reverse", "straight_s")

    # --- Tile (1,0): 90 deg turn (corner A) ---
    # Approach heading south then turn RIGHT (east)
    ox, oy = origin(1, 0)
    b.node("cornerA_n", ox + 100, oy + 50)
    b.node("cornerA_corner", ox + 100, oy + 250)
    b.node("cornerA_e", ox + 350, oy + 250)
    b.road("cornerA_n", "cornerA_corner")
    b.road("cornerA_corner", "cornerA_e")
    b.start("corner_right_entry", "cornerA_n")
    b.start("corner_right_exit", "cornerA_e")

    # --- Tile (2,0): 90 deg turn (corner B, mirrored) ---
    # Approach heading south then turn LEFT (west)
    ox, oy = origin(2, 0)
    b.node("cornerB_n", ox + 350, oy + 50)
    b.node("cornerB_corner", ox + 350, oy + 250)
    b.node("cornerB_w", ox + 100, oy + 250)
    b.road("cornerB_n", "cornerB_corner")
    b.road("cornerB_corner", "cornerB_w")
    b.start("corner_left_entry", "cornerB_n")
    b.start("corner_left_exit", "cornerB_w")

    # --- Tile (3,0): T-junction (3-way, perpendicular) ---
    ox, oy = origin(3, 0)
    b.node("tjunc_top", ox + 250, oy + 50)
    b.node("tjunc_center", ox + 250, oy + 250)
    b.node("tjunc_left", ox + 80, oy + 250)
    b.node("tjunc_right", ox + 420, oy + 250)
    b.road("tjunc_top", "tjunc_center")
    b.road("tjunc_center", "tjunc_left")
    b.road("tjunc_center", "tjunc_right")
    b.start("tjunction_from_top", "tjunc_top")
    b.start("tjunction_from_left", "tjunc_left")
    b.start("tjunction_from_right", "tjunc_right")

    # --- Tile (0,1): Y-intersection (shallow diverging angles) ---
    ox, oy = origin(0, 1)
    b.node("y_stem", ox + 250, oy + 50)
    b.node("y_center", ox + 250, oy + 220)
    b.node("y_left", ox + 100, oy + 400)
    b.node("y_right", ox + 400, oy + 400)
    b.road("y_stem", "y_center")
    b.road("y_center", "y_left")
    b.road("y_center", "y_right")
    b.start("y_from_stem", "y_stem")
    b.start("y_from_left", "y_left")
    b.start("y_from_right", "y_right")

    # --- Tile (1,1): 4-way intersection (crossroads) ---
    ox, oy = origin(1, 1)
    b.node("cross_center", ox + 250, oy + 220)
    b.node("cross_n", ox + 250, oy + 50)
    b.node("cross_s", ox + 250, oy + 400)
    b.node("cross_e", ox + 420, oy + 220)
    b.node("cross_w", ox + 80, oy + 220)
    b.road("cross_n", "cross_center")
    b.road("cross_center", "cross_s")
    b.road("cross_w", "cross_center")
    b.road("cross_center", "cross_e")
    b.start("crossroads_from_north", "cross_n")
    b.start("crossroads_from_south", "cross_s")
    b.start("crossroads_from_east", "cross_e")
    b.start("crossroads_from_west", "cross_w")

    # --- Tile (2,1): One-way street through a 4-way junction ---
    ox, oy = origin(2, 1)
    b.node("ow_center", ox + 250, oy + 220)
    b.node("ow_w", ox + 80, oy + 220)
    b.node("ow_e", ox + 420, oy + 220)
    b.node("ow_n", ox + 250, oy + 50)
    b.node("ow_s", ox + 250, oy + 400)
    # East-west road is one-way (west -> east only)
    b.road("ow_w", "ow_center", oneway=True)
    b.road("ow_center", "ow_e", oneway=True)
    # North-south road is normal two-way
    b.road("ow_n", "ow_center")
    b.road("ow_center", "ow_s")
    b.start("oneway_entry", "ow_w")            # legal: flows with the one-way
    b.start("oneway_wrong_way", "ow_e")        # illegal: would drive against the one-way
    b.start("oneway_cross_from_north", "ow_n")
    b.start("oneway_cross_from_south", "ow_s")

    # --- Tile (3,1): S-curve (gentle degree-2 bends) ---
    ox, oy = origin(3, 1)
    b.node("s_p0", ox + 100, oy + 50)
    b.node("s_p1", ox + 150, oy + 150)
    b.node("s_p2", ox + 280, oy + 220)
    b.node("s_p3", ox + 350, oy + 320)
    b.node("s_p4", ox + 400, oy + 420)
    b.road("s_p0", "s_p1")
    b.road("s_p1", "s_p2")
    b.road("s_p2", "s_p3")
    b.road("s_p3", "s_p4")
    b.start("s_curve", "s_p0")
    b.start("s_curve_reverse", "s_p4")

    # --- Tile (0,2): Dead-end ---
    ox, oy = origin(0, 2)
    b.node("dead_start", ox + 250, oy + 50)
    b.node("dead_end", ox + 250, oy + 300)
    b.road("dead_start", "dead_end")
    b.start("dead_end_approach", "dead_start")

    # --- Tile (1,2): Tight hairpin turn (~150 deg direction change) ---
    ox, oy = origin(1, 2)
    b.node("hair_a", ox + 100, oy + 50)
    b.node("hair_corner", ox + 100, oy + 250)
    b.node("hair_b", ox + 160, oy + 60)
    b.road("hair_a", "hair_corner")
    b.road("hair_corner", "hair_b")
    b.start("hairpin_entry", "hair_a")
    b.start("hairpin_exit", "hair_b")

    # --- Tile (2,2): Wide sweeping curve (gentle single bend, ~30 deg) ---
    ox, oy = origin(2, 2)
    b.node("sweep_a", ox + 100, oy + 50)
    b.node("sweep_mid", ox + 150, oy + 250)
    b.node("sweep_b", ox + 300, oy + 420)
    b.road("sweep_a", "sweep_mid")
    b.road("sweep_mid", "sweep_b")
    b.start("sweeping_curve", "sweep_a")
    b.start("sweeping_curve_reverse", "sweep_b")

    return b.build()


# Registry of all available test maps
TEST_MAPS = {
    "basic": build_basic_test_map,
}


def build_test_map(name: str) -> RoadNetwork:
    """Build a named synthetic test map."""
    if name not in TEST_MAPS:
        available = ", ".join(sorted(TEST_MAPS.keys()))
        raise ValueError(f"Unknown test map '{name}'. Available: {available}")
    return TEST_MAPS[name]()
