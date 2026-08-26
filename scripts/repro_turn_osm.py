"""Repro: does a signaled turn survive on the Kleinmachnow OSM map?

Searches for junctions where the STRAIGHT continuation ends at a dead end
within ~45 m (the driver's parking block would fire before the car reaches
the junction), then drives one such case headless exactly like the live
game (BICYCLE mode, signal via driver.signal_turn() as the REST API does)
and reports whether the car actually turns or ploughs straight through.

Usage: .venv/bin/python scripts/repro_turn_osm.py [candidate_index]
  without arg: just list candidate junctions
  with arg:    run the headless drive on that candidate and print outcome.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src import config
from src.osm_loader import fetch_osm_data
from src.road_network import RoadNetwork
from src.car import Car
from src.driver import BicycleDriver
from src.physics_validator import PhysicsValidator
from src.lane_guard import LaneGuard


class FakeKeys:
    """A live player holds W (throttle) while driving BICYCLE mode -
    the driver only accelerates when it is pressed. Everything else
    (turn signals) is set via signal_turn() like the REST API does."""
    def __getitem__(self, key):
        import pygame
        return key in (pygame.K_UP, pygame.K_w)


bb = config.BOUNDING_BOX
data = fetch_osm_data(bb["north"], bb["south"], bb["west"], bb["east"])
net = RoadNetwork.from_osm_data(
    data, bb["north"], bb["south"], bb["west"], bb["east"])


def straight_dist_to_dead_end(seg_idx: int, node: str):
    """Follow the straight continuation from `node` until a dead end;
    return total distance in m (None if it never ends within 8 hops)."""
    cur = seg_idx
    d = 0.0
    for _ in range(8):
        nxt = net.choose_next_segment(cur, node, "straight")
        if nxt is None or nxt == cur:
            return d
        nseg = net.segments[nxt]
        d += nseg.length
        node = nseg.end_node if nseg.start_node == node else nseg.start_node
        cur = nxt
    return None


def find_candidates():
    cands = []
    W, H = net.world_width, net.world_height
    for i, seg in enumerate(net.segments):
        j = seg.end_node  # forward end (car drives start->end)
        deg = len(net.get_connected_segments(j))
        if deg < 3:
            continue
        # Skip junctions at the edge of the OSM extract: roads are clipped
        # there and the paved polygon is unreliable.
        jx, jy = net.nodes[j]
        if not (250 < jx < W - 250 and 250 < jy < H - 250):
            continue
        sd = straight_dist_to_dead_end(i, j)
        if sd is None or sd > 45.0:
            continue
        branches = []
        for k in net.get_connected_segments(j):
            if k == i:
                continue
            a = abs(net.get_exit_angle(i, k))
            if a >= 30.0:
                branches.append((k, a))
        if not branches:
            continue
        cands.append((i, j, sd, branches))
    return cands


def run_drive(seg_idx: int, branch_seg: int, direction: str):
    """Drive from ~60 m before the junction with `direction` signaled."""
    seg = net.segments[seg_idx]
    j = seg.end_node
    L = seg.length
    s0 = max(5.0, L - 60.0)  # 60 m before junction (or near start)
    x0 = seg.x1 + (seg.x2 - seg.x1) * (s0 / L)
    y0 = seg.y1 + (seg.y2 - seg.y1) * (s0 / L)
    hdg = math.degrees(math.atan2(seg.x2 - seg.x1, seg.y2 - seg.y1)) % 360.0

    car = Car(x0, y0, hdg, seg_idx, BicycleDriver())
    car.progress = config.SPAWN_PROGRESS
    car.forward = True

    validator = PhysicsValidator(enabled=True)
    keys = FakeKeys()
    dt = 1 / 60.0
    t = 0.0
    crossed_branch = False
    stopped = False
    max_speed = 0.0
    signal_wiped_at = None
    signaled = False
    cross_t = None
    for step in range(60 * 30):
        # Signal at t=4 s (after the spawn pull-out) - like a live player.
        if not signaled and t >= 4.0:
            car.driver.signal_turn(direction)
            signaled = True
        control = car.driver.get_control(car, net, dt, keys)
        # watch the signal get wiped before the turn happened (clearing
        # AFTER crossing the branch is legitimate auto-off, not a wipe)
        if (signal_wiped_at is None and signaled and not crossed_branch
                and car.driver.pending_turn is None):
            signal_wiped_at = t
        car.update(dt, net, control)
        t += dt
        max_speed = max(max_speed, abs(car.speed))
        if car.seg_idx == branch_seg and cross_t is None:
            crossed_branch = True
            cross_t = t
        validator.check(car, dt, net)
        if crossed_branch and (t - cross_t) > 3.0:
            break   # turn done, let it settle briefly then stop simulating
        if abs(car.speed) < 0.05 and t > 8.0:
            stopped = True
            break

    print(f"\n== drive on seg {seg_idx} -> junction {j[:16]}..., signal "
          f"'{direction}', branch seg {branch_seg} ==")
    print(f"   t={t:.1f}s  end pos=({car.x:.0f},{car.y:.0f}) "
          f"seg={car.seg_idx} speed={car.speed*3.6:.1f} km/h "
          f"max_speed={max_speed*3.6:.0f} km/h")
    print(f"   TURN EXECUTED: {crossed_branch}   stopped: {stopped}")
    if signal_wiped_at is not None and not crossed_branch:
        print(f"   !!! signal wiped at t={signal_wiped_at:.1f}s "
              f"(parking block overrode the user's blinker)")
    print(f"   pending_turn now: {car.driver.pending_turn!r}, "
          f"blinker_right={car.driver.blinker_right}")
    print(f"   validator violations: {len(validator.violations)}")
    kinds = {}
    for v in validator.violations:
        k = v.get("type") or v.get("kind") or str(list(v.keys()))
        kinds[k] = kinds.get(k, 0) + 1
    print(f"   by kind: {kinds}")


if __name__ == "__main__":
    cands = find_candidates()
    print(f"{len(cands)} candidate junctions (straight-ahead dead end <=45m):")
    for n, (i, j, sd, branches) in enumerate(cands[:30]):
        bdesc = ", ".join(f"seg{k}@{a:.0f}deg" for k, a in branches)
        print(f"  [{n:2d}] seg {i:4d} -> node {j[:16]}... "
              f"straight_to_dead_end={sd:5.1f}m  branches: {bdesc}")
    if len(sys.argv) > 1 and cands:
        idx = int(sys.argv[1])
        i, j, sd, branches = cands[min(idx, len(cands) - 1)]
        k, a = branches[0]
        raw = net.get_exit_angle(i, k)   # positive = right, negative = left
        direction = "right" if raw > 10 else "left"
        run_drive(i, k, direction)
