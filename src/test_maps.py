# Synthetic Test Maps
# Deterministic, hand-crafted road networks for reproducible testing.
# Selected via --map <name> command line flag.
#
# COORDINATE SYSTEM (shared with the OSM maps - see src/road_network.py):
#   X grows EAST, Y grows NORTH (higher y = further north).
#   Heading is in degrees, 0 = north (+y), positive = right/east, and a
#   car's forward vector is (sin h, cos h). Every scenario below is laid
#   out in that frame, so the synthetic maps and the real OSM map use the
#   exact same convention and the same physics/geometry code works on both.

from __future__ import annotations

import math
from . import config
from .road_network import RoadNetwork, RoadSegment


class MapBuilder:
    """Helper to construct a RoadNetwork from named nodes given in METERS.

    Coordinates are given in the SHARED world frame (X east, Y NORTH),
    the same one the OSM projection produces. A node with a larger y is
    further north. Coordinates may be negative; build() shifts everything
    so the world starts at (0, 0).
    """

    def __init__(self):
        self.pppm = config.PIXELS_PER_METER
        self._nodes_m: dict[str, tuple[float, float]] = {}
        self._segments: list[RoadSegment] = []
        self._next_seg_id = 1
        self._start_points: dict[str, tuple[str, float, str | None]] = {}
        # name -> (node_id, lateral_offset_m, facing); degree-1 nodes or
        # loops with an explicit facing neighbour

    def node(self, node_id: str, x_m: float, y_m: float) -> None:
        """Define a node position in meters (X east, Y north)."""
        self._nodes_m[node_id] = (x_m, y_m)

    def road(
        self,
        n1: str,
        n2: str,
        highway: str = "residential",
        oneway: bool = False,
        width: float | None = None,
        lanes: int = 0,
        shoulder: float = 0.0,
        parking_lane_width: float = 0.0,
        level: int = 0,
    ) -> None:
        """Add a road segment between two named nodes.

        width (metres) overrides the ROAD_TYPES-derived width. lanes > 0
        marks the segment as a multi-lane carriageway with `lanes` DRIVING
        lanes per direction (one-way: total = lanes; two-way: 2 x lanes):
        the renderer draws a solid centreline on two-way roads, dashed
        dividers between the driving lanes of each direction, and - with
        `parking_lane_width` > 0 - a parking lane at each kerb (dashed
        boundary + painted P marks, ending >= PARK_LANE_END_GAP_M before
        any junction). `shoulder` metres of stop lane on the right apply
        to one-way layouts (right-hand traffic).

        `level` is the vertical level: 0 = ground, 1 = bridge over the
        ground, -1 = tunnel. Top-down view has no z axis; levels decide
        rendering order/style and keep cars on different levels from
        colliding (a figure-8 track crosses itself at one point where
        one ring is a bridge over the other).
        """
        x1_m, y1_m = self._nodes_m[n1]
        x2_m, y2_m = self._nodes_m[n2]
        pppm = self.pppm

        x1, y1 = x1_m * pppm, y1_m * pppm
        x2, y2 = x2_m * pppm, y2_m * pppm

        if width is None:
            road_cfg = config.ROAD_TYPES.get(highway)
            width = (
                road_cfg["width_1way"] if oneway else road_cfg["width_2way"]
            ) if road_cfg else 3.5

        length_m = math.hypot(x2_m - x1_m, y2_m - y1_m)

        self._segments.append(RoadSegment(
            id=self._next_seg_id,
            x1=x1, y1=y1, x2=x2, y2=y2,
            highway=highway,
            oneway=oneway,
            width=width,
            start_node=n1,
            end_node=n2,
            parking_lane_width=parking_lane_width,
            length=length_m,
            lanes=lanes,
            shoulder=shoulder,
            level=level,
        ))
        self._next_seg_id += 1

    def start(self, name: str, node_id: str, lateral_offset_m: float = 0.0,
              facing: str | None = None) -> None:
        """Register a named, deterministic start point at a node.

        The node must have exactly one connected segment (a scenario's
        entry point) so the car's initial heading and direction of travel
        are unambiguous. Use this to give tests a reproducible spawn
        location + heading approaching a specific junction.

        On a closed loop every node has TWO segments: pass `facing` = the
        id of the neighbour node the car faces (drives toward) to make
        the direction unambiguous (figure-8 lobbies).

        lateral_offset_m shifts the spawn laterally from the NORMAL
        DRIVING POSITION (right-lane centre): positive = toward the right
        kerb, negative = toward the left side of the road. Used by the
        parking-offset scenarios on the two-lane one-way tile.
        """
        self._start_points[name] = (node_id, lateral_offset_m, facing)

    def build(self, margin_m: float = 50.0) -> RoadNetwork:
        """Finalize and return the RoadNetwork."""
        pppm = self.pppm

        # ── Shift so world starts at (0, 0) — handle negative coords ──
        # Must happen FIRST, before any coordinate computation.
        xs_raw = [x for x, y in self._nodes_m.values()]
        ys_raw = [y for x, y in self._nodes_m.values()]
        min_x, min_y = min(xs_raw), min(ys_raw)
        if min_x < 0 or min_y < 0:
            # Shift all nodes
            self._nodes_m = {
                nid: (x - min_x, y - min_y)
                for nid, (x, y) in self._nodes_m.items()
            }
            # Shift segment endpoints (stored in px). Same direction as
            # the node shift above: new = old - min.
            sdx, sdy = min_x * pppm, min_y * pppm
            for seg in self._segments:
                seg.x1 -= sdx; seg.y1 -= sdy
                seg.x2 -= sdx; seg.y2 -= sdy

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
        start_points: dict[str, tuple[float, float, float, int, bool, float]] = {}
        for name, (node_id, lateral_offset_m, facing) in self._start_points.items():
            connected = node_connections.get(node_id, [])
            if len(connected) == 1:
                seg_idx = connected[0]
            elif len(connected) == 2 and facing is not None:
                # Closed loop: pick the segment toward `facing`.
                seg_idx = next(
                    (i for i in connected
                     if self._segments[i].start_node == facing
                     or self._segments[i].end_node == facing),
                    None)
                if seg_idx is None:
                    raise ValueError(
                        f"Start point '{name}' -> node '{node_id}': 'facing' "
                        f"node '{facing}' is not a neighbour")
            else:
                raise ValueError(
                    f"Start point '{name}' -> node '{node_id}' must have exactly "
                    f"1 connected segment (or 2 + facing on a loop), found "
                    f"{len(connected)}"
                )
            seg = self._segments[seg_idx]
            forward = (seg.start_node == node_id)
            x, y = (seg.x1, seg.y1) if forward else (seg.x2, seg.y2)
            dx = seg.x2 - seg.x1 if forward else seg.x1 - seg.x2
            dy = seg.y2 - seg.y1 if forward else seg.y1 - seg.y2
            heading = math.degrees(math.atan2(dx, dy))
            start_points[name] = (x, y, heading, seg_idx, forward,
                                  lateral_offset_m)

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

    A grid of ~500 m tiles, each holding one specific road situation.
    All coordinates are in the shared world frame (X east, Y NORTH).

        Tile (0,0): Straight road (baseline / acceleration test)
        Tile (1,0): 90 deg RIGHT turn (approach heading south)
        Tile (2,0): 90 deg LEFT turn (approach heading south)
        Tile (3,0): T-junction (3-way, perpendicular)
        Tile (0,1): Y-intersection (3-way, shallow diverging angles)
        Tile (1,1): 4-way intersection (crossroads)
        Tile (2,1): One-way street through a 4-way junction
        Tile (3,1): S-curve (gentle degree-2 bends)
        Tile (0,2): Dead-end
        Tile (1,2): Tight hairpin turn (~150 deg direction change)
        Tile (2,2): Wide sweeping curve (gentle single bend)
        Tile (3,2): Roundabout (one-way ring, 4 two-way spokes)
        Tile (0,3): Sliver junction - a very short approach into a 4-way
                    with a near-straight, a sharp-right and a sharp-left
                    exit (mirrors the real-world 815 -> 1008 layout).
        Tile (4,0): Widths road - straight dead-end street in four 50 m
                    sections of decreasing width: 13 m, 9 m, 7 m, 4 m.
                    Used for U-turn tests at different road widths
                    (single swing on 13 m, three-point on 9/7 m, ...).
        Tile (4,1): Two-lane one-way straight - 300 m, 7 m wide
                    (2 x 3.5 m lanes), single driving direction (south).
                    Target of the "reverse park from different lateral
                    start positions" scenarios (docs/DRIVING_MANEUVERS.md
                    §1 variant): five named starts at five lateral
                    positions on the same road, same destination flag.
        Tile (4,2): One-way INTO a 4-way junction - the W spoke is
                    one-way heading into the junction (travel west->east),
                    N/E/S spokes two-way. Entering from the west, the car
                    must swing slightly right across the junction so it
                    ends up in its own lane on the two-way east side.
        Tile (4,3): One-way OUT of a 4-way junction - the W spoke is
                    one-way heading away from the junction (travel
                    east->west), N/E/S spokes two-way. Entering from the
                    east, the car crosses the junction and eases onto the
                    narrow one-way exit, where there is no oncoming lane.
    """
    b = MapBuilder()
    TILE = 500.0  # pitch between tiles, meters

    def origin(col: int, row: int) -> tuple[float, float]:
        return col * TILE, row * TILE

    # --- Tile (0,0): Straight road (north-south) ---
    ox, oy = origin(0, 0)
    b.node("straight_n", ox + 100, oy + 350)   # north (large y)
    b.node("straight_s", ox + 100, oy + 50)    # south (small y)
    b.road("straight_n", "straight_s")
    b.start("straight", "straight_n")           # spawn at north, heading south
    b.start("straight_reverse", "straight_s")   # spawn at south, heading north

    # --- Tile (4,0): Widths road - four 50 m sections, one width each ---
    # Straight dead-end street, north to south: 13 m, 9 m, 7 m, 4 m.
    # U-turn scenarios run in the widest section; the narrow sections are
    # for future "too tight -> back up and retry" tests.
    ox, oy = origin(4, 0)
    b.node("widths_n",  ox + 100, oy + 300)    # north dead end (in 13 m section)
    b.node("widths_139", ox + 100, oy + 250)   # 13 -> 9
    b.node("widths_97",  ox + 100, oy + 200)   # 9 -> 7
    b.node("widths_74",  ox + 100, oy + 150)   # 7 -> 4
    b.node("widths_s",  ox + 100, oy + 100)    # south dead end (in 4 m section)
    b.road("widths_n", "widths_139", width=13.0)
    b.road("widths_139", "widths_97", width=9.0)
    b.road("widths_97", "widths_74", width=7.0)
    b.road("widths_74", "widths_s", width=4.0)
    b.start("widths", "widths_n")              # spawn at north, heading south

    # --- Tile (1,0): 90 deg RIGHT turn (approach heading south) ---
    # Come down from the north, turn right (WEST) at the corner.
    # (Facing south, west is on your right.)
    ox, oy = origin(1, 0)
    b.node("cornerR_n", ox + 350, oy + 350)
    b.node("cornerR_c", ox + 350, oy + 100)
    b.node("cornerR_w", ox + 100, oy + 100)
    b.road("cornerR_n", "cornerR_c")
    b.road("cornerR_c", "cornerR_w")
    b.start("corner_right_entry", "cornerR_n")
    b.start("corner_right_exit", "cornerR_w")

    # --- Tile (2,0): 90 deg LEFT turn (approach heading south) ---
    # Come down from the north, turn left (EAST) at the corner.
    # (Facing south, east is on your left.)
    ox, oy = origin(2, 0)
    b.node("cornerL_n", ox + 100, oy + 350)
    b.node("cornerL_c", ox + 100, oy + 100)
    b.node("cornerL_e", ox + 350, oy + 100)
    b.road("cornerL_n", "cornerL_c")
    b.road("cornerL_c", "cornerL_e")
    b.start("corner_left_entry", "cornerL_n")
    b.start("corner_left_exit", "cornerL_e")

    # --- Tile (3,0): T-junction (3-way, perpendicular) ---
    # Stem comes down from the north onto a west-east bar.
    ox, oy = origin(3, 0)
    b.node("tjunc_top", ox + 250, oy + 350)
    b.node("tjunc_center", ox + 250, oy + 100)
    b.node("tjunc_w", ox + 80, oy + 100)
    b.node("tjunc_e", ox + 420, oy + 100)
    b.road("tjunc_top", "tjunc_center")
    b.road("tjunc_center", "tjunc_w")
    b.road("tjunc_center", "tjunc_e")
    b.start("tjunction_from_top", "tjunc_top")
    b.start("tjunction_from_west", "tjunc_w")
    b.start("tjunction_from_east", "tjunc_e")

    # --- Tile (0,1): Y-intersection (shallow diverging angles) ---
    # Stem comes down from the north, forks to the south-west and
    # south-east (a shallow "Y").
    ox, oy = origin(0, 1)
    b.node("y_stem", ox + 250, oy + 400)
    b.node("y_center", ox + 250, oy + 220)
    b.node("y_sw", ox + 100, oy + 50)
    b.node("y_se", ox + 400, oy + 50)
    b.road("y_stem", "y_center")
    b.road("y_center", "y_sw")
    b.road("y_center", "y_se")
    b.start("y_from_stem", "y_stem")
    b.start("y_from_sw", "y_sw")
    b.start("y_from_se", "y_se")

    # --- Tile (1,1): 4-way intersection (crossroads) ---
    ox, oy = origin(1, 1)
    b.node("cross_center", ox + 250, oy + 220)
    b.node("cross_n", ox + 250, oy + 400)
    b.node("cross_s", ox + 250, oy + 50)
    b.node("cross_w", ox + 80, oy + 220)
    b.node("cross_e", ox + 420, oy + 220)
    b.road("cross_n", "cross_center")
    b.road("cross_center", "cross_s")
    b.road("cross_w", "cross_center")
    b.road("cross_center", "cross_e")
    b.start("crossroads_from_north", "cross_n")
    b.start("crossroads_from_south", "cross_s")
    b.start("crossroads_from_west", "cross_w")
    b.start("crossroads_from_east", "cross_e")

    # --- Tile (2,1): One-way street through a 4-way junction ---
    # East-west road is one-way (west -> east only); north-south is two-way.
    ox, oy = origin(2, 1)
    b.node("ow_center", ox + 250, oy + 220)
    b.node("ow_w", ox + 80, oy + 220)
    b.node("ow_e", ox + 420, oy + 220)
    b.node("ow_n", ox + 250, oy + 400)
    b.node("ow_s", ox + 250, oy + 50)
    b.road("ow_w", "ow_center", oneway=True)
    b.road("ow_center", "ow_e", oneway=True)
    b.road("ow_n", "ow_center")
    b.road("ow_center", "ow_s")
    b.start("oneway_entry", "ow_w")                # legal: flows with the one-way
    b.start("oneway_wrong_way", "ow_e")            # illegal: against the one-way
    b.start("oneway_cross_from_north", "ow_n")
    b.start("oneway_cross_from_south", "ow_s")

    # --- Tile (3,1): S-curve (gentle degree-2 bends) ---
    ox, oy = origin(3, 1)
    b.node("s_p0", ox + 100, oy + 400)
    b.node("s_p1", ox + 150, oy + 300)
    b.node("s_p2", ox + 280, oy + 220)
    b.node("s_p3", ox + 350, oy + 120)
    b.node("s_p4", ox + 400, oy + 50)
    b.road("s_p0", "s_p1")
    b.road("s_p1", "s_p2")
    b.road("s_p2", "s_p3")
    b.road("s_p3", "s_p4")
    b.start("s_curve", "s_p0")
    b.start("s_curve_reverse", "s_p4")

    # --- Tile (0,2): Dead-end ---
    ox, oy = origin(0, 2)
    b.node("dead_start", ox + 250, oy + 350)
    b.node("dead_end", ox + 250, oy + 100)
    b.road("dead_start", "dead_end")
    b.start("dead_end_approach", "dead_start")

    # --- Tile (1,2): Tight hairpin turn (~150 deg direction change) ---
    # Come down from the north, then fold back up to the east.
    ox, oy = origin(1, 2)
    b.node("hair_a", ox + 100, oy + 350)
    b.node("hair_corner", ox + 100, oy + 100)
    b.node("hair_b", ox + 160, oy + 340)
    b.road("hair_a", "hair_corner")
    b.road("hair_corner", "hair_b")
    b.start("hairpin_entry", "hair_a")
    b.start("hairpin_exit", "hair_b")

    # --- Tile (2,2): Wide sweeping curve (gentle single bend, ~30 deg) ---
    ox, oy = origin(2, 2)
    b.node("sweep_a", ox + 100, oy + 350)
    b.node("sweep_mid", ox + 150, oy + 150)
    b.node("sweep_b", ox + 300, oy + 50)
    b.road("sweep_a", "sweep_mid")
    b.road("sweep_mid", "sweep_b")
    b.start("sweeping_curve", "sweep_a")
    b.start("sweeping_curve_reverse", "sweep_b")

    # --- Tile (3,2): Roundabout (one-way ring, 4 two-way spokes) ---
    ox, oy = origin(3, 2)
    cx, cy = ox + 250, oy + 220
    R = 100.0
    D = R * 0.7071  # diagonal offset (NE/SE/SW/NW), R*cos(45deg)
    SPOKE = 150.0   # distance from ring out to each approach's far end

    # 32-node ring (every 11.25 deg) for a smooth curve. One-way,
    # COUNTER-CLOCKWISE in this north-up frame (N -> W -> S -> E), the
    # correct direction for right-hand traffic (Germany): the central
    # island stays on your LEFT as you go around. Many nodes = very short
    # straight chords = the curvature is detected on nearly every chord,
    # so the speed profile slows the car down for the ring (a coarse ring
    # has long straight chords where curvature reads 0, so the car blasts
    # through the ring and swings wide).
    import math as _m
    N_RING = 64
    ring_nodes = []
    for i in range(N_RING):
        # Start at north (90 deg) and go counter-clockwise (increasing angle
        # in standard math = counter-clockwise in north-up frame).
        ang = _m.radians(90 + i * (360.0 / N_RING))
        nx = cx + R * _m.cos(ang)
        ny = cy + R * _m.sin(ang)
        name = f"rb_r{i}"
        b.node(name, nx, ny)
        ring_nodes.append(name)
    for a, b_node in zip(ring_nodes, ring_nodes[1:] + ring_nodes[:1]):
        b.road(a, b_node, oneway=True)
    # Four two-way spokes, one per cardinal ring node. With 64 ring nodes
    # (every 5.625 deg), the cardinal nodes are rb_r0 (north, 90 deg),
    # rb_r16 (west, 180 deg), rb_r32 (south, 270 deg), rb_r48 (east, 0 deg).
    b.node("rb_north_far", cx, cy + R + SPOKE)
    b.node("rb_east_far", cx + R + SPOKE, cy)
    b.node("rb_south_far", cx, cy - R - SPOKE)
    b.node("rb_west_far", cx - R - SPOKE, cy)
    b.road("rb_north_far", "rb_r0")
    b.road("rb_east_far", "rb_r48")
    b.road("rb_south_far", "rb_r32")
    b.road("rb_west_far", "rb_r16")

    b.start("roundabout_from_north", "rb_north_far")
    b.start("roundabout_from_east", "rb_east_far")
    b.start("roundabout_from_south", "rb_south_far")
    b.start("roundabout_from_west", "rb_west_far")

    # --- Tile (0,3): Sliver junction (the real-world 815 -> 1008 layout) ---
    # A very SHORT approach (4.16 m) into a 4-way junction. The junction
    # has a near-straight continuation, a sharp-right exit and a sharp-left
    # exit - all 7 m wide. The approach is far too short to plan a turn in
    # advance, which is exactly what made the rail model crash here.
    #
    # Local geometry (junction at the tile center, in the shared frame):
    #   approach from the north (sliver, 4.16 m)
    #   straight continuation to the south (+1.4 deg)
    #   sharp-right exit to the west  (+91.5 deg)
    #   sharp-left  exit to the east  (-87.9 deg)
    ox, oy = origin(0, 3)
    cx, cy = ox + 250, oy + 250
    b.node("sliv_ap",   cx - 0.73, cy + 4.16)     # sliver approach (north)
    b.node("sliv_junc", cx,        cy)            # the 4-way junction
    b.node("sliv_str",  cx + 0.74, cy - 4.93)     # straight continuation (south)
    b.node("sliv_w",    cx - 36.2, cy - 5.4)      # sharp-right exit (west)
    b.node("sliv_e",    cx + 19.6, cy + 2.7)      # sharp-left exit (east)
    b.road("sliv_ap", "sliv_junc")
    b.road("sliv_junc", "sliv_str")
    b.road("sliv_w", "sliv_junc")
    b.road("sliv_junc", "sliv_e")
    b.start("sliver_approach", "sliv_ap")   # spawn on the sliver, heading for the junction
    b.start("sliver_from_west", "sliv_w")   # spawn on the sharp-right exit
    b.start("sliver_from_east", "sliv_e")   # spawn on the sharp-left exit

    # --- Tile (1,3): Figure-8 - ONE continuous loop crossing itself ---
    # A single closed circuit shaped like an 8: it goes around the left
    # lobe, crosses at P = (750, 1750), goes around the right lobe,
    # crosses again and closes. The crossing is modelled with TWO nodes
    # at the SAME point: p1 on the NE-SW branch, p2 on the NW-SE branch.
    # Every node has degree 2 (no junction logic needed - the driver
    # simply drives straight through), yet the whole figure-8 is ONE
    # connected cycle. The NE-SW branch (through p1) is a BRIDGE: the
    # two crossing segments plus one on each side are level 1, so a car
    # on that branch passes OVER the other branch.
    ox, oy = origin(1, 3)
    px, py = ox + 250, oy + 250            # the crossing point P
    # The loop is a sampled LEMNISCATE (Gerono curve): x = cos t,
    # y = sin t * cos t - smooth everywhere, crosses itself at exactly
    # one point with a 90-degree angle. 48 samples keep every chord
    # short enough that the spline stays smooth. The two crossing
    # passages are separate nodes n6 (t=90deg) and n18 (t=270deg) at
    # the SAME point P - both degree 2, so no junction logic is needed,
    # yet the whole figure-8 is one connected cycle.
    import math as _math
    N_FIG8 = 48
    for i in range(N_FIG8):
        t = 2 * _math.pi * i / N_FIG8
        b.node(f"fig8_n{i}",
               round(px + 245 * _math.cos(t), 1),
               round(py + 230 * _math.sin(t) * _math.cos(t), 1))
    # The full cycle: n_i -> n_{i+1} (and n47 -> n0). The crossing
    # point P is at n12 (t=90deg) and n36 (t=270deg). Segments 10..13
    # (n10->n11 ... n13->n14) form the NE-SW branch through n12 =
    # the BRIDGE: the two crossing pieces plus one segment each side.
    for i in range(N_FIG8):
        b.road(f"fig8_n{i}", f"fig8_n{(i + 1) % N_FIG8}",
               level=1 if i in (10, 11, 12, 13) else 0)
    b.start("fig8", "fig8_n24", facing="fig8_n25")   # leftmost point, heading north
    # ~19 m before the crossing on the GROUND branch (t=262.5deg): spawns
    # right before the car dives under the bridge deck.
    b.start("fig8_under", "fig8_n35", facing="fig8_n36")
    # Both at the crossing point P itself: n36 is the GROUND branch node
    # (car renders UNDER the deck - hidden), n12 the ELEVATED one (car on
    # top of the deck - visible). A/B pair for the occlusion check.
    b.start("fig8_deck", "fig8_n36", facing="fig8_n37")
    b.start("fig8_bridge", "fig8_n12", facing="fig8_n13")

    # --- Tile (4,0): WWW zig-zag (sharp corners → smoothed by Catmull-Rom)
    # A W-shaped zig-zag road: right-down, left-down, right-down.
    # With §10 smoothed geometry the sharp kinks at B/C/D become
    # slightly rounded curves instead of hard 90° corners.
    # NOTE: offsets must be POSITIVE and inside 0..TILE. Written as
    # `ox - 250` these landed at x=1750, which is exactly the T-junction
    # tile's stem - the zig-zag was drawn straight through tile (3,0),
    # silently wrecking every tjunction_* scenario.
    ox, oy = origin(4, 0)
    b.node("www_a", ox +  50, oy + 350)   # start, top-left
    b.node("www_b", ox + 130, oy +  60)   # V bottom
    b.node("www_c", ox + 210, oy + 350)   # peak
    b.node("www_d", ox + 290, oy +  60)   # V bottom
    b.node("www_e", ox + 370, oy + 350)   # peak
    b.node("www_f", ox + 450, oy +  60)   # end, bottom-right
    b.road("www_a", "www_b")
    b.road("www_b", "www_c")
    b.road("www_c", "www_d")
    b.road("www_d", "www_e")
    b.road("www_e", "www_f")
    b.start("www_entry", "www_a")
    b.start("www_exit", "www_f")

    # --- Tile (4,1): Parking avenue (2 driving + 1 parking lane per side) ---
    # 300 m straight two-way street, 19.4 m wide: per direction two 3.5 m
    # driving lanes plus a 2.7 m parking lane at the kerb (6 lanes total).
    # Solid centreline (crossing it = oncoming lane), painted P marks in
    # both parking lanes. Target of the "park from different lateral start
    # positions" scenarios (docs/DRIVING_MANEUVERS.md §1 variant): same
    # route and destination flag for all three, only the lateral spawn
    # position differs - always on our own side of the road. Offsets are
    # relative to the normal driving position (centre of the outermost
    # driving lane, 5.25 m right of the centreline), positive = toward
    # the right kerb:
    #   -3.50  -> middle of the left (overtaking) driving lane: a human
    #             merges right first, then parks (see §1 variant)
    #     0.0  -> middle of the right (normal) driving lane
    #   +3.10  -> middle of the parking lane (8.35 m from centreline)
    ox, oy = origin(4, 1)
    b.node("pkw_n", ox + 100, oy + 350)
    b.node("pkw_s", ox + 100, oy + 50)
    b.road("pkw_n", "pkw_s", oneway=False, lanes=2, width=19.4,
           parking_lane_width=2.7)
    b.start("park_6lane_left_lane",  "pkw_n", lateral_offset_m=-3.50)
    b.start("park_6lane_right_lane", "pkw_n")
    b.start("park_6lane_parking",    "pkw_n", lateral_offset_m=+3.10)

    # --- Tile (4,2): One-way INTO a 4-way junction ---
    # W spoke is one-way heading INTO the junction (travel west->east);
    # N/E/S spokes are two-way. The "keep the junction centre on our left"
    # corridor rule protects against oncoming traffic and therefore does
    # NOT apply on the one-way approach: entering from the west, the car
    # should swing slightly right across the junction so it ends up in its
    # own (right) lane on the two-way east side.
    ox, oy = origin(4, 2)
    b.node("mixin_center", ox + 250, oy + 220)
    b.node("mixin_w", ox + 80, oy + 220)
    b.node("mixin_e", ox + 420, oy + 220)
    b.node("mixin_n", ox + 250, oy + 400)
    b.node("mixin_s", ox + 250, oy + 50)
    b.road("mixin_w", "mixin_center", oneway=True)   # seg 110: into the junction
    b.road("mixin_center", "mixin_e")                # seg 111: two-way exit (test target)
    b.road("mixin_n", "mixin_center")
    b.road("mixin_center", "mixin_s")
    b.start("mixed_from_west", "mixin_w")

    # --- Tile (4,3): One-way OUT of a 4-way junction ---
    # W spoke is one-way heading AWAY from the junction (travel east->west);
    # N/E/S spokes are two-way. Entering from the east (two-way), the car
    # crosses the junction and eases onto the narrow one-way exit, where
    # there is no oncoming lane to keep clear of.
    ox, oy = origin(4, 3)
    b.node("mixout_center", ox + 250, oy + 220)
    b.node("mixout_w", ox + 80, oy + 220)
    b.node("mixout_e", ox + 420, oy + 220)
    b.node("mixout_n", ox + 250, oy + 400)
    b.node("mixout_s", ox + 250, oy + 50)
    b.road("mixout_center", "mixout_w", oneway=True) # seg 114: one-way exit (test target)
    b.road("mixout_e", "mixout_center")              # two-way approach from the east
    b.road("mixout_n", "mixout_center")
    b.road("mixout_center", "mixout_s")
    b.start("mixed_from_east", "mixout_e")

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
