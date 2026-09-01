# Road Network
# Stores the road graph with nodes, segments and connectivity,
# projects geographic coords to world pixels.

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import config


@dataclass
class RoadSegment:
    """A single road segment between two nodes."""
    id: int
    x1: float       # world pixel coords
    y1: float
    x2: float
    y2: float
    highway: str
    oneway: bool
    width: float    # metres
    start_node: str = ""   # node id at (x1, y1)
    end_node: str = ""     # node id at (x2, y2)
    length: float = 0.0    # metres
    lanes: int = 0         # DRIVING lanes per direction of travel; > 0
                           # marks a multi-lane carriageway that gets
                           # offset lane markings instead of a plain
                           # dashed centerline. One-way: total = lanes;
                           # two-way: total = 2 x lanes.
    shoulder: float = 0.0  # metres of stop lane (shoulder) on the right
    level: int = 0         # vertical LEVEL: 0 = ground, 1 = bridge over
                           # the ground, 2 = bridge over a bridge,
                           # -1 = tunnel. Top-down view has no z axis -
                           # levels only decide rendering order/style and
                           # that cars on different levels never collide.
    parking_lane_width: float = 0.0  # metres; > 0 = the outermost right
                                     # lane is a PARKING LANE (at each
                                     # kerb - i.e. both sides of a two-way
                                     # road). Drives the painted P marks
                                     # and the parking-lane boundary line,
                                     # which end >= config.PARK_LANE_END_GAP_M
                                     # before any junction.


@dataclass
class RoadNetwork:
    nodes: dict                # id -> (x, y) in world pixels
    segments: list[RoadSegment]
    origin_lat: float
    origin_lon: float
    world_width: float
    world_height: float
    node_connections: dict = field(default_factory=dict)  # node_id -> [segment_indices]
    node_degree: dict = field(default_factory=dict)       # node_id -> connection count
    node_max_width: dict = field(default_factory=dict)    # node_id -> (half_width_px, highway)
    start_points: dict = field(default_factory=dict)      # name -> (x, y, heading, seg_idx, forward, lateral_offset_m)

    def get_start_point(self, name: str) -> tuple[float, float, float, int, bool, float]:
        """Look up a named, deterministic start point (defined by synthetic
        test maps). Returns (x, y, heading, seg_idx, forward,
        lateral_offset_m) - the offset shifts the spawn laterally from the
        normal driving position (positive = toward the right kerb)."""
        if name not in self.start_points:
            available = ", ".join(sorted(self.start_points.keys())) or "(none defined)"
            raise KeyError(f"No start point named '{name}'. Available: {available}")
        return self.start_points[name]

    # --- Construction ---

    @classmethod
    def from_osm_data(cls, data: dict, north: float, south: float, west: float, east: float) -> "RoadNetwork":
        """Build a RoadNetwork from parsed OSM data."""
        pppm = config.PIXELS_PER_METER

        # Project all nodes
        nodes = {}
        for nid, n in data["nodes"].items():
            x, y = latlon_to_world(n["lat"], n["lon"], south, west, pppm)
            nodes[nid] = (x, y)

        # Build segments with node references
        segments = []
        seg_node_ids: list[tuple[str, str]] = []
        for way in data["ways"]:
            way_nodes = way["nodes"]
            highway = way["highway"]
            oneway = way["oneway"]

            for i in range(len(way_nodes) - 1):
                n1 = way_nodes[i]
                n2 = way_nodes[i + 1]
                if n1 not in nodes or n2 not in nodes:
                    continue
                x1, y1 = nodes[n1]
                x2, y2 = nodes[n2]

                road_cfg = config.ROAD_TYPES.get(highway)
                if road_cfg:
                    width = road_cfg["width_1way"] if oneway else road_cfg["width_2way"]
                else:
                    width = 3.5

                # Length in metres (world coords are in pixels, divide by pppm)
                seg_length = math.hypot(x2 - x1, y2 - y1) / pppm

                segments.append(RoadSegment(
                    id=way["id"],
                    x1=x1, y1=y1, x2=x2, y2=y2,
                    highway=highway,
                    oneway=oneway,
                    width=width,
                    start_node=n1,
                    end_node=n2,
                    length=seg_length,
                ))
                seg_node_ids.append((n1, n2))

        # Snap dangling endpoints onto nearby roads
        snapped = snap_endpoints(segments, pppm)
        if snapped:
            print(f"  Snapped {snapped} dangling endpoints")

        # Recompute node positions from snapped endpoints
        node_sum: dict[str, list[float]] = {}
        node_degree: dict[str, int] = {}
        for seg, (n1, n2) in zip(segments, seg_node_ids):
            for nid, x, y in ((n1, seg.x1, seg.y1), (n2, seg.x2, seg.y2)):
                s = node_sum.setdefault(nid, [0.0, 0.0, 0.0])
                s[0] += x
                s[1] += y
                s[2] += 1
                node_degree[nid] = node_degree.get(nid, 0) + 1
        for nid, (sx, sy, c) in node_sum.items():
            nodes[nid] = (sx / c, sy / c)

        # Build node_connections: which segment indices touch each node
        node_connections: dict[str, list[int]] = {}
        for idx, seg in enumerate(segments):
            node_connections.setdefault(seg.start_node, []).append(idx)
            node_connections.setdefault(seg.end_node, []).append(idx)

        # Junction info: widest road at each node (for rendering)
        node_info: dict[str, tuple[float, str]] = {}
        for seg in segments:
            half = (seg.width / 2) * pppm
            for nid in (seg.start_node, seg.end_node):
                cur = node_info.get(nid)
                if cur is None or half > cur[0]:
                    node_info[nid] = (half, seg.highway)

        # Skip degree-2 nodes (straight road continuation)
        node_info = {
            nid: info for nid, info in node_info.items()
            if node_degree.get(nid, 0) != 2
        }

        world_width = latlon_to_world(south, east, south, west, pppm)[0]
        world_height = latlon_to_world(north, west, south, west, pppm)[1]

        return cls(
            nodes=nodes,
            segments=segments,
            origin_lat=south,
            origin_lon=west,
            world_width=world_width,
            world_height=world_height,
            node_connections=node_connections,
            node_degree=node_degree,
            node_max_width=node_info,
        )

    # --- Graph queries ---

    def get_connected_segments(self, node_id: str) -> list[int]:
        """Return indices of all segments connected to a node."""
        return self.node_connections.get(node_id, [])

    def get_exit_angle(self, from_seg_idx: int, to_seg_idx: int) -> float:
        """Return the turning angle (degrees, negative=left, positive=right)
        when going from one segment to another at a shared node."""
        from_seg = self.segments[from_seg_idx]
        to_seg = self.segments[to_seg_idx]

        # Find shared node
        shared = None
        for n in (from_seg.start_node, from_seg.end_node):
            if n in (to_seg.start_node, to_seg.end_node):
                shared = n
                break
        if not shared:
            return 0.0

        # Entry direction: towards the shared node (along from_seg)
        fn = self.nodes[from_seg.start_node] if from_seg.start_node != shared else self.nodes[from_seg.end_node]
        fs = self.nodes[shared]
        # Vector pointing TOWARDS shared node
        from_vec = (fs[0] - fn[0], fs[1] - fn[1])

        # Exit direction: away from shared node (along to_seg)
        tn = self.nodes[to_seg.start_node] if to_seg.start_node != shared else self.nodes[to_seg.end_node]
        ts = self.nodes[shared]
        # Vector pointing AWAY from shared node
        to_vec = (tn[0] - ts[0], tn[1] - ts[1])

        # Angle between vectors (left = negative, right = positive)
        from_angle = math.degrees(math.atan2(from_vec[0], from_vec[1]))
        to_angle = math.degrees(math.atan2(to_vec[0], to_vec[1]))
        diff = to_angle - from_angle
        # Normalize to (-180, 180]
        while diff > 180:
            diff -= 360
        while diff <= -180:
            diff += 360
        return diff

    def has_right_of_way_conflict(self, from_seg_idx: int, node_id: str) -> bool:
        """Check if there's a road coming from the right at this junction.
        Returns True if we need to yield (rechts vor links)."""
        connected = self.get_connected_segments(node_id)
        if len(connected) <= 2:
            # Not a real junction
            return False
        
        for idx in connected:
            if idx == from_seg_idx:
                continue
            
            angle = self.get_exit_angle(from_seg_idx, idx)
            # "From the right" means angle between -45° and -135° (right side)
            if -135 <= angle <= -45:
                return True
        
        return False

    def choose_next_segment(self, from_seg_idx: int, node_id: str, turn_direction: str) -> int | None:
        """Choose the next segment when reaching a node.
        turn_direction: 'left', 'right', or 'straight'.
        Returns segment index or None if no suitable segment found."""
        connected = self.get_connected_segments(node_id)
        if len(connected) <= 1:
            # Dead end or continuation — just go back the way we came
            return from_seg_idx if len(connected) == 1 else None

        candidates = []
        for idx in connected:
            if idx == from_seg_idx:
                continue
            angle = self.get_exit_angle(from_seg_idx, idx)
            candidates.append((idx, angle))

        if not candidates:
            return None

        # For oneway roads, respect direction: a oneway segment may only
        # be ENTERED at its own start_node (that's the only way to then
        # travel start->end, the one legal direction) - entering it at
        # its end_node would mean driving it backward. This used to be a
        # no-op (every candidate was let through regardless), which was
        # invisible on simple one-way streets (the only other option at
        # that junction was usually also fine) but broke badly on a
        # roundabout's all-oneway ring: the car could get routed the
        # wrong way around a ring segment, which the turn-planning
        # geometry doesn't handle (it assumes normal forward traversal),
        # producing tangent-point mismatches and position jumps.
        filtered = []
        for idx, angle in candidates:
            seg = self.segments[idx]
            if seg.oneway and seg.start_node != node_id:
                continue
            filtered.append((idx, angle))

        if not filtered:
            return None

        if turn_direction == "left":
            # Most negative angle (sharp left preferred)
            best = min(filtered, key=lambda x: x[1])
            if best[1] < -10:
                return best[0]
        elif turn_direction == "right":
            # Most positive angle (sharp right preferred)
            best = max(filtered, key=lambda x: x[1])
            if best[1] > 10:
                return best[0]
        else:
            # Straight: smallest absolute angle
            best = min(filtered, key=lambda x: abs(x[1]))
            if abs(best[1]) < 30:
                return best[0]

        # Fallback: if preferred turn not available, just go straight
        best = min(filtered, key=lambda x: abs(x[1]))
        return best[0]

    # --- Spatial queries ---

    def is_on_road(self, wx: float, wy: float) -> bool:
        """Check if a world position is on any road.

        Uses the exact same paved-area polygon that gets rendered (see
        get_paved_polygon() / Renderer.draw_roads()) - rounded bends and
        junction fillets included - instead of a cruder, independent
        rectangle+circle approximation that used to let the visual road
        and the actual drivable area disagree (e.g. a car following the
        smooth rendered curve through a bend could get flagged as
        off-road because the old check only knew about sharp-cornered
        rectangles).
        """
        from shapely.geometry import Point
        tolerance_px = config.ROAD_EDGE_TOLERANCE_M * config.PIXELS_PER_METER
        return self.get_paved_polygon().distance(Point(wx, wy)) <= tolerance_px

    def is_car_on_road(self, wx: float, wy: float, heading_deg: float,
                       length_m: float = 4.5, width_m: float = 1.8) -> bool:
        """Check if a 4-wheel car (not just its center point) is on the road.

        A bicycle-style check (center point only) lets the car's body/wheels
        overhang the road edge without being detected. This checks all FOUR
        CORNERS of the car's bounding box against the same paved polygon, so
        any wheel leaving the road is flagged. The corners are computed from
        the car's center, heading, length and width (north-up frame: heading
        0 = north, forward = (sin h, cos h), right = (cos h, -sin h)).
        """
        from shapely.geometry import Point
        tolerance_px = config.ROAD_EDGE_TOLERANCE_M * config.PIXELS_PER_METER
        h = math.radians(heading_deg)
        fx, fy = math.sin(h), math.cos(h)   # forward
        rx, ry = math.cos(h), -math.sin(h)  # right
        pppm = config.PIXELS_PER_METER
        half_l = (length_m / 2.0) * pppm
        half_w = (width_m / 2.0) * pppm
        poly = self.get_paved_polygon()
        for sfx in (1.0, -1.0):
            for srx in (1.0, -1.0):
                cx = wx + (sfx * fx * half_l + srx * rx * half_w)
                cy = wy + (sfx * fy * half_l + srx * ry * half_w)
                if poly.distance(Point(cx, cy)) > tolerance_px:
                    return False
        return True

    def _old_is_on_road(self, wx: float, wy: float) -> bool:
        pppm = config.PIXELS_PER_METER
        for seg in self.segments:
            half_width = (seg.width / 2) * pppm
            dist = point_to_segment_distance(wx, wy, seg.x1, seg.y1, seg.x2, seg.y2)
            if dist <= half_width:
                return True
        
        for node_id, degree in self.node_degree.items():
            if degree < 2:
                continue
            node_xy = self.nodes.get(node_id)
            if node_xy is None:
                continue
            connected = self.node_connections.get(node_id, [])
            if not connected:
                continue
            widest_seg = max((self.segments[i] for i in connected), key=lambda s: s.width)
            radius_px = (widest_seg.width / 2 + config.JUNCTION_WIDENING_M) * pppm
            if math.hypot(wx - node_xy[0], wy - node_xy[1]) <= radius_px:
                return True
        
        return False

    # --- Paved-area geometry (shared by physics AND the renderer, so
    # the drivable area always exactly matches what's painted) ---

    def get_road_polygons_by_color(self):
        """Build (color, [(exterior_coords, [hole_coords, ...]), ...])
        groups for drawing. Holes matter for closed-loop roads (e.g. a
        roundabout's ring) - a plain buffered ring genuinely has a hole
        in the middle (the roundabout's island), and dropping that hole
        would render/treat it as a solid filled disk instead. Cached -
        the road network never changes at runtime."""
        if getattr(self, "_road_polygons_cache", None) is None:
            self._road_polygons_cache = _build_road_polygons(self)
        return self._road_polygons_cache

    def get_elevated_polygons(self):
        """Buffered road surface of every segment with level >= 1 (bridge).

        Returns [(exterior_coords, [hole_coords, ...]), ...] in world
        PIXELS. Uses the SAME smoothed spline + buffer parameters as the
        ground roads so the bridge deck matches the carriageway under it
        exactly (no sliver of ground road poking out from under the edge).
        Renderers draw these on top of the ground, with their own styling,
        and keep cars at lower levels below them in z-order. Cached -
        the network never changes at runtime."""
        if getattr(self, "_elevated_polygons_cache", None) is None:
            from shapely.geometry import LineString
            from shapely.ops import unary_union
            from .smooth_geometry import (SmoothCurve, SmoothedNetwork,
                                          smoothed_network)

            sm_net = smoothed_network(self)
            pppm = config.PIXELS_PER_METER

            # 1) Walk the elevated segments into ordered chains. Each
            #    chain: (node_coords_px in path order, max_width).
            by_node: dict[str, list[int]] = {}
            for idx, seg in enumerate(self.segments):
                if seg.level >= 1:
                    by_node.setdefault(seg.start_node, []).append(idx)
                    by_node.setdefault(seg.end_node, []).append(idx)
            seen_seg: set[int] = set()
            # chain: (node ids in path order, node coords px, max width)
            chains: list[tuple[list[str], list[tuple[float, float]], float]] = []
            for start_idx in range(len(self.segments)):
                if self.segments[start_idx].level < 1 or start_idx in seen_seg:
                    continue
                seg = self.segments[start_idx]
                ids = [seg.start_node, seg.end_node]
                widths = [seg.width]
                seen_seg.add(start_idx)
                # extend the chain in both directions while elevated
                for direction in (0, 1):  # 0 = toward start_node, 1 = end
                    cur = seg.start_node if direction == 0 else seg.end_node
                    while True:
                        i2 = next((i for i in by_node.get(cur, [])
                                   if i not in seen_seg), None)
                        if i2 is None:
                            break
                        s2 = self.segments[i2]
                        other = (s2.end_node if s2.start_node == cur
                                 else s2.start_node)
                        if direction == 0:
                            ids.insert(0, other)
                        else:
                            ids.append(other)
                        widths.append(s2.width)
                        seen_seg.add(i2)
                        cur = other
                chains.append((ids, [self.nodes[i] for i in ids],
                               max(widths)))

            # 2) For each chain build a centripetal Catmull-Rom curve
            #    through the chain PLUS one context node at each end
            #    (so the interior tangents match the network spline -
            #    CR at a point only needs its two neighbours), then slice
            #    between the chain's own end nodes' arc lengths.
            adj: dict[str, set[str]] = {}
            for seg in self.segments:
                adj.setdefault(seg.start_node, set()).add(seg.end_node)
                adj.setdefault(seg.end_node, set()).add(seg.start_node)

            parts = []
            centerlines: list[list[tuple[float, float]]] = []
            for ids, nodes, width in chains:
                ext = list(nodes)
                # one context neighbour at each end (the one NOT in the
                # chain) so the CR tangents at the slice ends match the
                # network spline
                for nid, end in ((ids[0], 0), (ids[-1], -1)):
                    for nbr in adj.get(nid, ()):
                        if nbr not in ids:
                            if end == 0:
                                ext.insert(0, self.nodes[nbr])
                            else:
                                ext.append(self.nodes[nbr])
                            break
                curve = SmoothCurve(ext, pppm=pppm)
                s0 = SmoothedNetwork._node_s(curve, nodes[0])
                s1 = SmoothedNetwork._node_s(curve, nodes[-1])
                if s0 > s1:
                    s0, s1 = s1, s0
                half_w_px = (width / 2.0) * pppm
                n = max(2, int((s1 - s0) / (0.5 * pppm)) + 1)
                pts = [curve.point_at(s0 + (s1 - s0) * i / (n - 1))
                       for i in range(n)]
                parts.append(LineString(pts).buffer(
                    half_w_px, cap_style="flat", join_style="round",
                    resolution=8))
                # the deck's own centreline (renderers draw it ABOVE the
                # deck - the ground-level one is covered by it)
                centerlines.append(pts)

            self._elevated_centerlines_cache: list[list[tuple[float, float]]] = centerlines
            if parts:
                unioned = unary_union(parts)
                polys = unioned.geoms if hasattr(unioned, "geoms") else [unioned]
                self._elevated_polygons_cache = [
                    (list(p.exterior.coords),
                     [list(r.coords) for r in p.interiors])
                    for p in polys if not p.is_empty]
            else:
                self._elevated_polygons_cache = []
        return self._elevated_polygons_cache

    def get_elevated_centerlines(self):
        """Centreline polylines (world PIXELS) of the elevated decks -
        the same sliced splines the deck surface is buffered from.
        Renderers draw them ABOVE the deck (the ground-level centreline
        underneath is covered). Computed alongside get_elevated_polygons."""
        self.get_elevated_polygons()
        return getattr(self, "_elevated_centerlines_cache", [])

    def get_paved_polygon(self):
        """A single unioned Shapely (Multi)Polygon covering the entire
        drivable paved area (every road, rounded bends and junction
        fillets included, holes like a roundabout's island properly
        excluded) - the authoritative geometry for on-road checks, built
        from the exact same per-color polygons used for rendering.
        Cached - the road network never changes at runtime."""
        if getattr(self, "_paved_polygon_cache", None) is None:
            from shapely.geometry import MultiPolygon, Polygon
            from shapely.ops import unary_union
            polys = [
                Polygon(ext, holes)
                for _color, exteriors in self.get_road_polygons_by_color()
                for ext, holes in exteriors
            ]
            unioned = unary_union(polys) if polys else Polygon()
            # unary_union leaves zero-area SLIVER rings at junctions
            # (degenerate rings whose vertices collapse onto a line, as
            # outer parts AND as holes). Invisible as fill, but the
            # paved-edge outline draws every ring - including these - as
            # floating line fragments in both frontends. Drop degenerate
            # outer parts and degenerate holes alike.
            def _clean(poly):
                holes = [h for h in poly.interiors if Polygon(h).area >= 1.0]
                return Polygon(poly.exterior, holes)

            geoms = (list(unioned.geoms) if unioned.geom_type == "MultiPolygon"
                     else [unioned])
            kept = [_clean(g) for g in geoms if g.area >= 1.0]
            self._paved_polygon_cache = (
                MultiPolygon(kept) if len(kept) > 1
                else (kept[0] if kept else Polygon()))
        return self._paved_polygon_cache

    def spawn_path_point(self, seg_idx: int, node_xy: tuple[float, float],
                         distance_m: float):
        """Point + heading at `distance_m` along the ACTUAL road path from
        a degree-1 node (a scenario's entry point).

        Spawn placement must follow the corner-rounded merged centreline
        the pavement is built from - not the segment's chord. At a sharp
        U-turn the two diverge by tens of metres: the fillet tangent
        distance is R/tan(gap/2), and for a 166-degree hairpin with the
        6 m curb radius that is ~49 m, so a chord-based spawn at 80% of
        the segment sits up to ~30 m off the real road (measured: off-
        pavement on hairpin_entry, wrong-side crash on hairpin_exit).

        A degree-1 node is an endpoint of exactly one merged line, so we
        simply walk that line from the node. Returns (x_px, y_px,
        heading_deg) or None if no merged line ends within 30 m of the
        node - callers fall back to the plain chord.
        """
        groups = _merge_and_round_lines(self)
        nx, ny = node_xy
        best_line: list | None = None
        best_d2 = float("inf")
        for lines in groups.values():
            for coords in lines:
                if len(coords) < 2:
                    continue
                for end in (coords[0], coords[-1]):
                    d2 = (end[0] - nx) ** 2 + (end[1] - ny) ** 2
                    if d2 < best_d2:
                        best_d2, best_line = d2, list(coords)
        if best_line is None or \
                best_d2 > (30.0 * config.PIXELS_PER_METER) ** 2:
            return None
        if best_line[0][0] != nx or best_line[0][1] != ny:
            best_line.reverse()   # walk from the end at the node INTO the road
        target = max(0.0, distance_m) * config.PIXELS_PER_METER
        x, y = best_line[0]
        heading = 0.0
        remaining = target
        for i in range(len(best_line) - 1):
            x0, y0 = best_line[i]
            x1, y1 = best_line[i + 1]
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len < 1e-9:
                continue
            if remaining <= seg_len or i == len(best_line) - 2:
                t = min(1.0, remaining / seg_len)
                x = x0 + (x1 - x0) * t
                y = y0 + (y1 - y0) * t
                heading = math.degrees(math.atan2(x1 - x0, y1 - y0))
                break
            remaining -= seg_len
        return x, y, heading

    def get_centerlines(self):
        """Merged, corner-rounded road CENTERLINES (no buffering), ALL
        two-way roads. Used by the LaneGuard (wrong-side check); the
        dashed-line rendering uses get_marking_centerlines() instead. Reuses the exact same merge-through-plain-bends +
        corner-rounding logic as the paved-area polygons (see
        _build_road_polygons), so the markings visually follow the same
        smooth curve as the road edges instead of cutting straight across
        a rounded bend. Returns a list of coordinate lists (one per
        merged line). Cached - the road network never changes at
        runtime."""
        if getattr(self, "_centerlines_cache", None) is None:
            self._centerlines_cache = _build_centerlines(self)
        return self._centerlines_cache

    def get_marking_centerlines(self):
        """Centerlines that actually GET a dashed white line drawn: only
        roads at least as wide as the test map's standard road (7 m).
        Narrow service lanes (3.5 m) would just be visual noise. Kept
        separate from get_centerlines() because the LaneGuard needs the
        unfiltered set."""
        if getattr(self, "_marking_centerlines_cache", None) is None:
            groups = _merge_and_round_lines(self, only_two_way=True,
                                            skip_multi_lane=True,
                                            stop_at_junctions=True)
            out = []
            for coords in (coords for (_hw, w), lines in groups.items()
                           if w >= config.CENTERLINE_MIN_WIDTH_M
                           for coords in lines):
                # No paint on the crossing itself: stop the dashes just
                # short of the fillet corner at 3+-way junctions.
                t0, t1 = _junction_marking_trim_px(self, coords)
                trimmed = _trim_ends(list(coords), t0, t1)
                if trimmed:
                    out.append(trimmed)
            self._marking_centerlines_cache = out
        return self._marking_centerlines_cache

    def get_lane_markings(self):
        """Lane markings for multi-lane one-way carriageways (segments
        with lanes > 0), per the German RQ 31 layout (right-hand
        traffic, from the median outward):

          'solid'     narrow (0.15 m)  edge of the overtaking lane
          'dashed'    (0.15 m, fine 2 m / 4 m dashes by the renderer)
                      between the two driving lanes
          'solid'     broad (0.30 m, Breitstrich) between the travel
                      lane and the stop lane (no line on the outer
                      edge of the stop lane)
          'guardrail' light-gray crash barrier at each edge of a
                      central median between two parallel carriageways

        Returns a list of (style, coords, width_m) tuples.
        Cached - the road network never changes at runtime."""
        if getattr(self, "_lane_markings_cache", None) is None:
            self._lane_markings_cache = _build_lane_markings(self)
        return self._lane_markings_cache

    def get_oneway_arrows(self):
        """Painted direction arrows for one-way roads: a small white arrow
        in EACH lane, every 100 m (starting 50 m in), pointing along the
        legal direction of travel. Makes the allowed direction visible at a
        glance while watching e2e runs.

        Returns a list of polygons [(x, y), ...] in world pixels.
        Cached - the road network never changes at runtime."""
        if getattr(self, "_oneway_arrows_cache", None) is None:
            self._oneway_arrows_cache = _build_oneway_arrows(self)
        return self._oneway_arrows_cache

    def get_parking_marks(self):
        """Painted 'P' letters in parking lanes (segments with
        parking_lane_width > 0): one per lane every 100 m (starting 50 m
        in), centred on the lane, readable by a driver travelling in that
        lane's direction. Two-way roads get P marks at BOTH kerbs (the
        left ones face the opposite way). Like the parking-lane boundary
        line, no mark lies closer than PARK_LANE_END_GAP_M to a junction
        - parking lanes end there (user decision).

        Returns a list of polygons [(x, y), ...] in world pixels.
        Cached - the road network never changes at runtime."""
        if getattr(self, "_parking_marks_cache", None) is None:
            self._parking_marks_cache = _build_parking_marks(self)
        return self._parking_marks_cache

    def random_road_point(self) -> tuple[float, float, float, int, str]:
        """Return a random (x, y, heading, seg_idx, node_id) on any road segment.
        Heading in degrees, 0=up. node_id is the start node."""
        import random
        seg_idx = random.randrange(len(self.segments))
        seg = self.segments[seg_idx]
        t = random.random()
        x = seg.x1 + t * (seg.x2 - seg.x1)
        y = seg.y1 + t * (seg.y2 - seg.y1)
        heading = math.degrees(math.atan2(seg.x2 - seg.x1, seg.y2 - seg.y1))
        return x, y, heading, seg_idx, seg.start_node

    # --- Bounds ---

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """(left, top, right, bottom) in world pixels."""
        return 0, 0, self.world_width, self.world_height


# --- Projection helpers ---

def latlon_to_world(lat: float, lon: float, ref_lat: float, ref_lon: float, pppm: float) -> tuple[float, float]:
    """Convert lat/lon to world pixel coordinates using equirectangular projection."""
    meters_per_lat_deg = 111132.9 - 566.0 * math.cos(2 * math.radians(lat)) + 1.2 * math.cos(4 * math.radians(lat))
    meters_per_lon_deg = 111320 * math.cos(math.radians(lat))

    dx_m = (lon - ref_lon) * meters_per_lon_deg
    dy_m = (lat - ref_lat) * meters_per_lat_deg

    return dx_m * pppm, dy_m * pppm


def point_to_segment(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> tuple[float, float, float]:
    """Return (distance, proj_x, proj_y) from point to line segment."""
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy

    if length_sq == 0:
        return math.hypot(px - x1, py - y1), x1, y1

    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return math.hypot(px - proj_x, py - proj_y), proj_x, proj_y


def point_to_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Shortest distance from point to line segment."""
    d, _, _ = point_to_segment(px, py, x1, y1, x2, y2)
    return d


def _chain_segments(segs: list, network: "RoadNetwork") -> list:
    """Chain segments into maximal polylines, continuing through a node
    ONLY if it is a plain degree-2 bend. Shapely's linemerge would also
    fuse arms that meet at a real junction (same group, shared endpoint),
    producing lines that run straight across the crossing - fine for the
    pavement union, wrong for painted markings, which must stop before
    the intersection (user decision)."""
    by_node: dict = {}
    for i, s in enumerate(segs):
        by_node.setdefault(s.start_node, []).append(i)
        by_node.setdefault(s.end_node, []).append(i)
    used = [False] * len(segs)
    chains: list[list] = []

    def extend(pts: list, from_node: str, prepend: bool):
        cur, last = from_node, pts[0] if prepend else pts[-1]
        new_pts = []
        while network.node_degree.get(cur, 0) == 2:
            cands = [j for j in by_node.get(cur, []) if not used[j]]
            if not cands:
                break
            t = segs[cands[0]]
            used[cands[0]] = True
            if (t.x1, t.y1) == last:
                nxt_pt, cur = (t.x2, t.y2), t.end_node
            else:
                nxt_pt, cur = (t.x1, t.y1), t.start_node
            new_pts.append(nxt_pt)
            last = nxt_pt
        if prepend:
            pts[:0] = list(reversed(new_pts))
        else:
            pts.extend(new_pts)

    for i0 in range(len(segs)):
        if used[i0]:
            continue
        s = segs[i0]
        used[i0] = True
        pts = [(s.x1, s.y1), (s.x2, s.y2)]
        extend(pts, s.end_node, prepend=False)
        extend(pts, s.start_node, prepend=True)
        chains.append(pts)
    return chains


def _merge_and_round_lines(network: "RoadNetwork", only_two_way: bool = False,
                           skip_multi_lane: bool = False,
                           stop_at_junctions: bool = False):
    """Group segments by (highway, width), merge contiguous ones through
    plain degree-2 nodes, and round each merged line's own corners (see
    _round_polyline_corners). Shared by both _build_road_polygons
    (buffers these into fillable polygons) and _build_centerlines (draws
    them directly as the dashed lane-marking line) so the two always
    agree exactly on where the road curves.

    only_two_way: skip oneway segments entirely - a single-lane one-way
    street has no opposing lane to divide, so it gets no center dashed
    line (used by _build_centerlines; _build_road_polygons still wants
    the pavement itself regardless of oneway).

    stop_at_junctions: chain manually instead of Shapely's linemerge and
    never continue through a degree-3+ node - for painted markings, which
    must end at the crossing instead of running across it.

    Returns {(highway, width): [smoothed_coords, ...]}.
    """
    from shapely.geometry import LineString
    from shapely.ops import linemerge

    pppm = config.PIXELS_PER_METER
    corner_radius_px = config.ROAD_CORNER_RADIUS_M * pppm

    groups: dict[tuple[str, float], list] = {}
    for seg in network.segments:
        if only_two_way and seg.oneway:
            continue
        # Multi-lane carriageways draw their OWN centerline (solid, via
        # _build_lane_markings) - the plain dashed one would double it.
        if skip_multi_lane and seg.lanes > 0:
            continue
        length = math.hypot(seg.x2 - seg.x1, seg.y2 - seg.y1)
        if length < 1e-6:
            continue
        groups.setdefault((seg.highway, seg.width), []).append(seg)

    result: dict[tuple[str, float], list] = {}
    for key, segs in groups.items():
        if stop_at_junctions:
            raw = _chain_segments(segs, network)
        else:
            lines = [LineString([(s.x1, s.y1), (s.x2, s.y2)]) for s in segs]
            merged = linemerge(lines) if len(lines) > 1 else lines[0]
            merged_lines = merged.geoms if hasattr(merged, "geoms") else [merged]
            raw = [list(line.coords) for line in merged_lines]
        result[key] = [
            _round_polyline_corners(coords, corner_radius_px) for coords in raw
        ]
    return result


def _build_centerlines(network: "RoadNetwork"):
    """Flatten _merge_and_round_lines()'s per-group coordinate lists into
    one plain list of polylines (color/width no longer matter for a
    lane-marking line). One-way segments are excluded - see
    _merge_and_round_lines(only_two_way=...).

    Returns ALL two-way centerlines: the LaneGuard needs every one of them
    (it falls back to a broken chord-distance heuristic when its list is
    empty). The dashed-line rendering filters by width separately - see
    RoadNetwork.get_marking_centerlines()."""
    groups = _merge_and_round_lines(network, only_two_way=True)
    return [coords for lines in groups.values() for coords in lines]


def _junction_trim_m(network: "RoadNetwork", coords: list) -> tuple[float, float]:
    """(start_trim, end_trim) in pixels: PARK_LANE_END_GAP_M where the
    line's end sits at a real junction (node degree >= 3), else 0.
    Parking-lane elements must end that far BEFORE a junction - visually,
    not just semantically (user decision)."""
    pppm = config.PIXELS_PER_METER
    gap = config.PARK_LANE_END_GAP_M * pppm

    def trim_at(px: float, py: float) -> float:
        for nid, nxy in network.nodes.items():
            if network.node_degree.get(nid, 0) < 3:
                continue
            if (nxy[0] - px) ** 2 + (nxy[1] - py) ** 2 <= (0.5 * pppm) ** 2:
                return gap
        return 0.0

    return trim_at(*coords[0]), trim_at(*coords[-1])


def _junction_marking_trim_px(network: "RoadNetwork", coords: list) -> tuple[float, float]:
    """(start_trim, end_trim) in pixels for PAINTED lane markings (dashed
    centerlines / lane dividers): where a line's end sits at a real
    junction (node degree >= 3), the paint stops half the WIDEST arm's
    width plus CENTERLINE_JUNCTION_GAP_M before the node centre - just
    short of the Eckausrundung corner point C on this arm's axis, so the
    dashes end where the rounded corner begins instead of cutting straight
    across the crossing (user decision). 0 at ends without a junction."""
    pppm = config.PIXELS_PER_METER

    def gap_at(px: float, py: float) -> float:
        for nid, nxy in network.nodes.items():
            if network.node_degree.get(nid, 0) < 3:
                continue
            if (nxy[0] - px) ** 2 + (nxy[1] - py) ** 2 <= (0.5 * pppm) ** 2:
                widest = max(network.segments[i].width
                             for i in network.node_connections.get(nid, []))
                return (widest / 2.0 + config.CENTERLINE_JUNCTION_GAP_M) * pppm
        return 0.0

    return gap_at(*coords[0]), gap_at(*coords[-1])


def _trim_ends(coords: list, start_m: float, end_m: float) -> list:
    """Drop `start_m` / `end_m` pixels of arc length from a polyline's
    ends, inserting exact cut points. Returns [] if nothing is left."""
    if len(coords) < 2 or start_m <= 0 and end_m <= 0:
        return list(coords)
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(coords, coords[1:]))
    if start_m + end_m >= total:
        return []

    def cut_at(pts: list, s: float) -> tuple[float, float]:
        acc = 0.0
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            seglen = math.hypot(b[0] - a[0], b[1] - a[1])
            if acc + seglen >= s and seglen > 0:
                t = (s - acc) / seglen
                return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            acc += seglen
        return pts[-1]

    head = cut_at(coords, start_m)
    tail = cut_at(list(reversed(coords)), end_m)
    out = [head]
    for p in coords[1:-1]:
        if (p[0] - head[0]) ** 2 + (p[1] - head[1]) ** 2 > 1e-9 and \
           (p[0] - tail[0]) ** 2 + (p[1] - tail[1]) ** 2 > 1e-9:
            out.append(p)
    out.append(tail)
    return out if len(out) >= 2 else []


def _build_lane_markings(network: "RoadNetwork"):
    """Offset lane-marking lines for multi-lane carriageways (lanes > 0).

    One-way (RQ 31 layout, right-hand traffic, from the median outward):
      solid   +W/2        left edge (edge of the overtaking lane)
      dashed  +S/2        between the driving lanes
      solid  -(W/2 - S)   between the travel lane and the stop lane

    Two-way (e.g. the parking avenue: two driving + one parking lane per
    side), measured from the centreline, symmetric about it:
      solid   0           SOLID centreline - crossing it means entering
                          the oncoming lane, so it must not be dashed
                          (user decision)
      dashed  +/-k*l      dividers between the two driving lanes of each
                          direction (l = driving-lane width) - fine 2 m /
                          4 m dashes (user decision)
      p_dash  +/-(W/2-P)  boundary of the parking lane at each kerb -
                          finer 1 m / 1 m dashes (user decision); like
                          the painted P marks it ends >=
                          PARK_LANE_END_GAP_M before any junction

    The lines are Shapely offset curves of the same corner-rounded
    centerlines the pavement is buffered from, so the markings follow the
    road's curves exactly.

    NOTE on signs: Shapely's offset_curve uses the standard math
    convention where a positive offset lands on the LEFT of the line's
    direction - but our world has Y pointing DOWN (south), which mirrors
    that convention, so a positive offset_curve distance actually lands
    on the RIGHT of travel here. The calls below therefore use negated
    distances.
    """
    from shapely.geometry import LineString

    pppm = config.PIXELS_PER_METER
    corner_radius_px = config.ROAD_CORNER_RADIUS_M * pppm

    groups: dict[tuple, list] = {}
    for seg in network.segments:
        if seg.lanes <= 0:
            continue
        if math.hypot(seg.x2 - seg.x1, seg.y2 - seg.y1) < 1e-6:
            continue
        groups.setdefault((seg.highway, seg.width, seg.lanes, seg.shoulder,
                           seg.oneway, seg.parking_lane_width), []).append(seg)

    markings: list[tuple[str, list, float]] = []
    for (_hw, width_m, lanes, shoulder_m, oneway, park_w), segs in groups.items():
        W = width_m * pppm
        S = shoulder_m * pppm
        P = park_w * pppm
        # Chain through plain bends only (never across a real junction -
        # the paint must stop before the crossing, see _chain_segments).
        raw = _chain_segments(segs, network)

        oriented: list[list] = []
        for coords0 in raw:
            coords = _round_polyline_corners(coords0, corner_radius_px)
            # linemerge does not preserve our direction of travel - re-
            # orient the line so its first point is a segment START
            # (start_node side), i.e. the line runs the way traffic flows.
            sx, sy = coords[0]
            if not any(abs(s.x1 - sx) < 1e-6 and abs(s.y1 - sy) < 1e-6 for s in segs):
                coords = list(reversed(coords))
            oriented.append(coords)

        def offset_of(coords: list, dist: float) -> list:
            try:
                oc = LineString(coords).offset_curve(dist, join_style="round")
            except Exception:
                return []
            if oc is None or oc.is_empty:
                return []
            geoms = oc.geoms if hasattr(oc, "geoms") else [oc]
            return [list(g.coords) for g in geoms if not g.is_empty]

        for coords in oriented:
            # Paint stops before the crossing (see
            # _junction_marking_trim_px); guardrails below are physical
            # objects and keep their full length.
            t0, t1 = _junction_marking_trim_px(network, coords)
            paint = (list(coords) if not (t0 or t1)
                     else _trim_ends(list(coords), t0, t1))
            if not paint:
                continue    # too short to fit paint away from both junctions
            if oneway:
                markings.extend(("solid", c, 0.15)
                                for c in offset_of(paint, -W / 2))
                markings.extend(("dashed", c, 0.15)
                                for c in offset_of(paint, -S / 2))
                markings.extend(("solid", c, 0.30)
                                for c in offset_of(paint, W / 2 - S))
            else:
                # Solid centreline: crossing it = oncoming lane.
                markings.append(("solid", list(paint), 0.15))
                D = W / 2 - P                     # driving strip per side
                l = D / lanes                     # driving-lane width
                for k in range(1, lanes):
                    d = k * l
                    markings.extend(("dashed", c, 0.15)
                                    for c in offset_of(paint, d))
                    markings.extend(("dashed", c, 0.15)
                                    for c in offset_of(paint, -d))
                if P > 0:
                    t0, t1 = _junction_trim_m(network, coords)
                    trimmed = _trim_ends(list(coords), t0, t1)
                    if trimmed:
                        markings.extend(("p_dash", c, 0.15)
                                        for c in offset_of(trimmed, D))
                        markings.extend(("p_dash", c, 0.15)
                                        for c in offset_of(trimmed, -D))

        # Central median: two carriageways of this group that run
        # roughly parallel within a couple of metres of each other form
        # a median strip; put a light-gray crash barrier (Schutzplanke)
        # on each carriageway's edge of that median.
        for i in range(len(oriented)):
            for j in range(i + 1, len(oriented)):
                if _line_distance(oriented[i], oriented[j]) >= 15.0 * pppm:
                    continue
                for a, b in ((oriented[i], oriented[j]), (oriented[j], oriented[i])):
                    s = _toward_sign(a, _midpoint(b))
                    for c in offset_of(a, s * (W / 2)):
                        markings.append(("guardrail", c, 0.15))
    return markings


def _build_oneway_arrows(network: "RoadNetwork"):
    """Painted direction arrows for one-way roads (see get_oneway_arrows).

    One arrow per lane every 100 m (50 + k*100, kept >= 20 m short of the
    far end), centred on the lane: lane i (0 = rightmost) sits
    W/2 - lane_w*(i+0.5) to the RIGHT of the centreline. Segments shorter
    than ~70 m get no arrow (there is no room for one away from both ends).
    """
    pppm = config.PIXELS_PER_METER
    arrows = []
    for seg in network.segments:
        if not seg.oneway:
            continue
        dx, dy = seg.x2 - seg.x1, seg.y2 - seg.y1
        length_px = math.hypot(dx, dy)
        if length_px < 1e-6:
            continue
        fx, fy = dx / length_px, dy / length_px   # unit direction of travel
        rx, ry = fy, -fx                          # right-hand side of it
        n_lanes = max(1, seg.lanes or 1)
        lane_w = seg.width / n_lanes              # metres
        for i in range(n_lanes):
            off_m = seg.width / 2.0 - lane_w * (i + 0.5)   # right of centreline
            s = 50.0
            while s <= seg.length - 20.0:
                cx = seg.x1 + fx * s * pppm + rx * off_m * pppm
                cy = seg.y1 + fy * s * pppm + ry * off_m * pppm
                arrows.append(_arrow_polygon(cx, cy, fx, fy, rx, ry, pppm))
                s += 100.0
    return arrows


def _arrow_polygon(cx: float, cy: float, fx: float, fy: float,
                   rx: float, ry: float, pppm: float) -> list:
    """A small painted direction arrow (3 m long, pointing along f): a 1.8 m
    shaft (0.6 m wide) with a 1.2 m head (1.4 m wide), centred on (cx, cy).
    (s_m, off_m) offsets are metres along / right of the arrow axis."""
    def pt(s_m: float, off_m: float) -> tuple[float, float]:
        return (cx + fx * s_m * pppm + rx * off_m * pppm,
                cy + fy * s_m * pppm + ry * off_m * pppm)
    return [
        pt(-1.5, -0.3),   # tail, left
        pt(0.3, -0.3),    # shaft -> head, left
        pt(0.3, -0.7),    # head wing, left
        pt(1.5, 0.0),     # tip
        pt(0.3, 0.7),     # head wing, right
        pt(0.3, 0.3),     # shaft -> head, right
        pt(-1.5, 0.3),    # tail, right
    ]


def _p_polygons(cx: float, cy: float, fx: float, fy: float,
                rx: float, ry: float, pppm: float) -> list:
    """A blocky painted 'P' (2.0 m tall x 1.35 m wide, centred on (cx, cy)),
    readable by traffic travelling along f: stem on the driver's left, bowl
    to the right. Built from four overlapping rectangles so no polygon holes
    are needed. (s_m, off_m) offsets are metres along / right of f."""
    def rect(s0, s1, o0, o1) -> list:
        return [
            (cx + fx * s0 * pppm + rx * o0 * pppm,
             cy + fy * s0 * pppm + ry * o0 * pppm),
            (cx + fx * s1 * pppm + rx * o0 * pppm,
             cy + fy * s1 * pppm + ry * o0 * pppm),
            (cx + fx * s1 * pppm + rx * o1 * pppm,
             cy + fy * s1 * pppm + ry * o1 * pppm),
            (cx + fx * s0 * pppm + rx * o1 * pppm,
             cy + fy * s0 * pppm + ry * o1 * pppm),
        ]
    # 5 x 7 pixel grid scaled to 1.35 m x 2.0 m (col w 0.27, row h ~0.286)
    return [
        rect(-1.0, 1.0, -0.675, -0.405),    # stem (full height, left)
        rect(0.714, 1.0, -0.675, 0.405),    # top bar
        rect(0.143, 1.0, 0.405, 0.675),     # right side of the bowl
        rect(-0.143, 0.143, -0.675, 0.405), # middle bar (closes the bowl)
    ]


def _build_parking_marks(network: "RoadNetwork"):
    """Painted P marks for parking lanes (see get_parking_marks)."""
    pppm = config.PIXELS_PER_METER
    gap = config.PARK_LANE_END_GAP_M * pppm
    marks: list[list] = []
    for seg in network.segments:
        if seg.parking_lane_width <= 0 or seg.lanes <= 0:
            continue
        dx, dy = seg.x2 - seg.x1, seg.y2 - seg.y1
        length_px = math.hypot(dx, dy)
        if length_px < 1e-6:
            continue
        fx, fy = dx / length_px, dy / length_px   # unit direction of travel
        rx, ry = fy, -fx                          # right-hand side of it
        # Parking lanes end >= PARK_LANE_END_GAP_M before a junction.
        s_min = gap if network.node_degree.get(seg.start_node, 0) >= 3 else 0.0
        s_max = (seg.length * pppm - gap
                 if network.node_degree.get(seg.end_node, 0) >= 3
                 else seg.length * pppm)
        # Centre of the parking lane: driving strip + half the parking
        # lane, right of the centreline. Two-way roads have one at each
        # kerb; the left one faces the opposite direction of travel.
        off_m = seg.width / 2.0 - seg.parking_lane_width / 2.0
        sides = [(1.0, fx, fy, rx, ry)]
        if not seg.oneway:
            sides.append((-1.0, -fx, -fy, -rx, -ry))
        for side_sign, sfx, sfy, srx, sry in sides:
            s = 50.0 * pppm
            while s <= s_max - 20.0 * pppm:
                if s >= s_min + 1e-6:
                    cx = seg.x1 + fx * s + rx * off_m * side_sign * pppm
                    cy = seg.y1 + fy * s + ry * off_m * side_sign * pppm
                    marks.extend(_p_polygons(cx, cy, sfx, sfy, srx, sry, pppm))
                s += 100.0 * pppm
    return marks


def _midpoint(coords: list) -> tuple[float, float]:
    return coords[len(coords) // 2]


def _toward_sign(coords: list, target: tuple[float, float]) -> int:
    """+1 or -1: which offset_curve() side of `coords` points toward
    `target`. offset_curve() follows the standard (y-up) math
    convention, so its positive-offset normal in coordinate space is
    (-dy, dx) of the line direction."""
    i = max(1, len(coords) // 2 - 1)
    dx = coords[i + 1][0] - coords[i][0]
    dy = coords[i + 1][1] - coords[i][1]
    L = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / L, dx / L
    mx, my = _midpoint(coords)
    return 1 if nx * (target[0] - mx) + ny * (target[1] - my) > 0 else -1


def _line_distance(c1: list, c2: list) -> float:
    """Mean distance from sample points of polyline c1 to polyline c2
    (small iff the two polylines run parallel close together)."""
    n = max(4, len(c1) // 5)
    total = 0.0
    for k in range(n):
        px, py = c1[int(k * (len(c1) - 1) / (n - 1))]
        total += min(point_to_segment(px, py, *c2[a], *c2[a + 1])[0]
                     for a in range(len(c2) - 1))
    return total / n


def _build_road_polygons(network: "RoadNetwork"):
    """Build road polygons from the §10 smoothed geometry (Catmull-Rom
    splines through the merged lines) instead of direct Shapely buffer
    on rounded polylines. The spline passes through every original node
    including degree-2 bends, so sharp zig-zag corners become slightly
    rounded curves.

    Grouped by (highway type, width) for coloring. Returns a list of
    (color, [exterior_ring_coords, ...]) - polygon holes are included
    (e.g. roundabout islands). Junction fillets at degree-3+ nodes are
    added separately via SmoothedNetwork.junction_fillets."""
    from shapely.geometry import LineString
    from shapely.ops import unary_union

    pppm = config.PIXELS_PER_METER
    from .smooth_geometry import smoothed_network
    sm_net = smoothed_network(network)

    # Group lines by (highway, width) — same grouping the old code used.
    groups: dict[tuple[str, float], list] = {}
    for line in sm_net.lines:
        key = (line["highway"], line["width"])
        groups.setdefault(key, []).append(line["resampled"])

    result = []
    for (highway, width), all_coords in groups.items():
        half_w_px = (width / 2) * pppm

        # Buffer each line's spline and union them together.
        buffered_parts = []
        for coords in all_coords:
            smooth_line = LineString(coords)
            buffered_parts.append(
                # EXPERIMENT: square ends (cap_style was "round")
                smooth_line.buffer(half_w_px, cap_style="flat", join_style="round", resolution=8)
            )
        buffered = unary_union(buffered_parts)

        color = config.ROAD_TYPES.get(highway, {}).get("color", (150, 150, 150))
        polys = buffered.geoms if hasattr(buffered, "geoms") else [buffered]
        exteriors = [(list(p.exterior.coords), [list(r.coords) for r in p.interiors]) for p in polys if not p.is_empty]
        result.append((color, exteriors))

    # Junction corner roundings (Eckausrundung patches).
    extras = _build_smoothed_junction_fillets(network, sm_net)
    result.extend(extras)
    return result


def _build_multiway_junction_fillets(network: "RoadNetwork", pppm: float):
    """Round the corners at 3-/4-way junctions (T, Y, crossroads) too.
    linemerge() only merges through degree-2 nodes, so at a real
    junction each connected road stays a separate segment/line with
    nothing rounding the gap between adjacent ones. For every pair of
    roads meeting at a shallow-enough angle (a real 'corner', not a
    near-straight pass-through or a near-duplicate), build a short
    virtual 3-point polyline through the junction node, run it through
    the exact same corner-rounding + buffer logic used for plain bends,
    and add the resulting little capsule patch - it blends in seamlessly
    since it's the same color and geometry construction as everything
    else."""
    from shapely.geometry import LineString

    MIN_GAP_DEG = 15.0
    MAX_GAP_DEG = 155.0
    corner_radius_px = config.ROAD_CORNER_RADIUS_M * pppm

    extras = []
    for node_id, connected in network.node_connections.items():
        if len(connected) < 3:
            continue  # degree-2 bends are already handled via linemerge
        node_xy = network.nodes.get(node_id)
        if node_xy is None:
            continue
        node_x, node_y = node_xy

        spokes = []
        for seg_idx in connected:
            seg = network.segments[seg_idx]
            if seg.start_node == node_id:
                away_dx, away_dy = seg.x2 - seg.x1, seg.y2 - seg.y1
            else:
                away_dx, away_dy = seg.x1 - seg.x2, seg.y1 - seg.y2
            length = math.hypot(away_dx, away_dy)
            if length < 1e-6:
                continue
            angle = math.atan2(away_dy, away_dx)
            spokes.append((angle, away_dx / length, away_dy / length, seg))
        if len(spokes) < 2:
            continue
        spokes.sort(key=lambda s: s[0])

        n = len(spokes)
        for i in range(n):
            _, ax, ay, seg_a = spokes[i]
            _, bx, by, seg_b = spokes[(i + 1) % n]
            gap = math.acos(max(-1.0, min(1.0, ax * bx + ay * by)))
            gap_deg = math.degrees(gap)
            if gap_deg < MIN_GAP_DEG or gap_deg > MAX_GAP_DEG:
                continue

            half_w_px = (min(seg_a.width, seg_b.width) / 2) * pppm
            reach = corner_radius_px * 3 + half_w_px
            virtual_line = [
                (node_x + reach * ax, node_y + reach * ay),
                (node_x, node_y),
                (node_x + reach * bx, node_y + reach * by),
            ]
            smooth = _round_polyline_corners(virtual_line, corner_radius_px)
            # EXPERIMENT: square ends (cap_style was "round")
            buffered = LineString(smooth).buffer(
                half_w_px, cap_style="flat", join_style="round", resolution=8
            )
            color = config.ROAD_TYPES.get(seg_a.highway, {}).get("color", (150, 150, 150))
            polys = buffered.geoms if hasattr(buffered, "geoms") else [buffered]
            exteriors = [(list(p.exterior.coords), [list(r.coords) for r in p.interiors]) for p in polys if not p.is_empty]
            extras.append((color, exteriors))
    return extras


def _build_smoothed_junction_fillets(network: "RoadNetwork",
                                      sm_net: "SmoothedNetwork") -> list:
    """Build the Eckausrundung patches (de.wikipedia.org/wiki/Eckausrundung):
    at every junction corner, the curvilinear triangle between the corner
    point and the rounding arc - the paved fill that lets a car swing from
    one road into the other without leaving the pavement. The arcs come
    precomputed from SmoothedNetwork.junction_fillets.

    (Historical note: this used to buffer node-centred fillet arcs by a
    road half-width, which bulged past the kerbs when road widths differed;
    see git history for the old implementation.)"""
    extras = []
    for fillet in sm_net.junction_fillets:
        ring = [fillet["corner"]] + list(fillet["arc"])
        color = config.ROAD_TYPES.get(network.segments[fillet["seg_a"]].highway,
                                      {}).get("color", (150, 150, 150))
        extras.append((color, [(ring, [])]))
    return extras


def _corner_tangent_budget(coords, radius):
    """Per-vertex tangent-distance allowance, sharing each edge between the
    two corners that use it in proportion to what they actually need.

    The simple rule - give every corner half of each adjoining edge - is
    safe but badly over-conservative when a corner's neighbour needs little
    or nothing. On the sliver junction the 4.22 m approach ends at a dead
    end, so the corner at the junction is the ONLY claimant, yet the half
    rule still handed it 2.11 m: a 2.06 m fillet radius, well inside the
    car's 3.46 m minimum turning radius, i.e. a reference line no car could
    follow. Sharing proportionally gives it the whole edge and a 4.11 m
    radius, which is drivable.
    """
    n = len(coords)
    want = [0.0] * n
    for i in range(1, n - 1):
        ax, ay = coords[i - 1][0] - coords[i][0], coords[i - 1][1] - coords[i][1]
        bx, by = coords[i + 1][0] - coords[i][0], coords[i + 1][1] - coords[i][1]
        la, lb = math.hypot(ax, ay), math.hypot(bx, by)
        if la < 1e-9 or lb < 1e-9:
            continue
        dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
        gap = math.acos(dot)
        if gap < 1e-6 or gap > math.pi - 1e-6:
            continue
        want[i] = radius / math.tan(gap / 2)

    budget = [float("inf")] * n
    for i in range(n - 1):
        Le = math.hypot(coords[i + 1][0] - coords[i][0],
                        coords[i + 1][1] - coords[i][1])
        d1, d2 = want[i], want[i + 1]
        if d1 + d2 > Le and (d1 + d2) > 1e-9:
            scale = Le / (d1 + d2)
            d1, d2 = d1 * scale, d2 * scale
        budget[i] = min(budget[i], d1 if d1 > 0 else float("inf"))
        budget[i + 1] = min(budget[i + 1], d2 if d2 > 0 else float("inf"))
    return budget


def _round_polyline_corners(coords, radius, arc_steps=10, fit_edges=False):
    """Replace every interior vertex of a polyline with a circular arc
    of the given radius, tangent to both adjoining edges - i.e. actually
    round the line's own corners, not just its eventual stroke outline.
    Endpoints are left untouched.

    fit_edges: share each edge between its two corners in proportion to
    demand (see _corner_tangent_budget) instead of giving each half. Used
    for the driving line, where an over-tight fillet is not merely ugly but
    unfollowable. Rendering keeps the half rule so road shapes are
    unchanged.
    """
    if len(coords) < 3 or radius <= 0:
        return coords
    budget = _corner_tangent_budget(coords, radius) if fit_edges else None

    result = [coords[0]]
    for i in range(1, len(coords) - 1):
        px, py = coords[i - 1]
        vx, vy = coords[i]
        nx, ny = coords[i + 1]

        ax, ay = px - vx, py - vy
        bx, by = nx - vx, ny - vy
        a_len = math.hypot(ax, ay)
        b_len = math.hypot(bx, by)
        if a_len < 1e-9 or b_len < 1e-9:
            result.append((vx, vy))
            continue
        ax, ay = ax / a_len, ay / a_len
        bx, by = bx / b_len, by / b_len

        # Angle between the two edges (both pointing away from the vertex).
        dot = max(-1.0, min(1.0, ax * bx + ay * by))
        gap = math.acos(dot)
        if gap < 1e-6 or gap > math.pi - 1e-6:
            # Straight-through (or fully doubled-back) - no real corner.
            result.append((vx, vy))
            continue

        half_gap = gap / 2
        tangent_dist = radius / math.tan(half_gap)
        # Cap the tangent distance so it never eats more than half of
        # either adjoining edge (avoids self-overlap on short segments).
        if budget is not None:
            tangent_dist = min(tangent_dist, budget[i])
        else:
            tangent_dist = min(tangent_dist, a_len / 2, b_len / 2)
        actual_radius = tangent_dist * math.tan(half_gap)

        center_dist = actual_radius / math.sin(half_gap)
        bis_x, bis_y = ax + bx, ay + by
        bis_len = math.hypot(bis_x, bis_y)
        if bis_len < 1e-9:
            result.append((vx, vy))
            continue
        bis_x, bis_y = bis_x / bis_len, bis_y / bis_len

        center_x = vx + center_dist * bis_x
        center_y = vy + center_dist * bis_y
        t1x, t1y = vx + tangent_dist * ax, vy + tangent_dist * ay
        t2x, t2y = vx + tangent_dist * bx, vy + tangent_dist * by

        angle_t1 = math.atan2(t1y - center_y, t1x - center_x)
        angle_t2 = math.atan2(t2y - center_y, t2x - center_x)

        # The correct arc sweep is always the EXTERIOR turn angle
        # (pi - gap) - a small, sharp corner (gap near 0) needs a wide
        # near-180 deg arc, a nearly-straight corner (gap near pi) needs
        # a tiny arc. Of the two possible *rotation directions* around
        # the circle from t1 to t2, pick whichever one's size actually
        # matches that value. Importantly, t1 must ALWAYS be emitted
        # before t2 - t1 lies on the incoming edge, t2 on the outgoing
        # edge, so swapping their order self-intersects the line.
        fwd_sweep = (angle_t2 - angle_t1) % (2 * math.pi)
        expected_sweep = math.pi - gap
        if abs(fwd_sweep - expected_sweep) < abs((2 * math.pi - fwd_sweep) - expected_sweep):
            direction, sweep = 1, fwd_sweep
        else:
            direction, sweep = -1, (2 * math.pi) - fwd_sweep

        result.append((t1x, t1y))
        for s in range(1, arc_steps):
            a = angle_t1 + direction * sweep * (s / arc_steps)
            result.append((center_x + actual_radius * math.cos(a),
                           center_y + actual_radius * math.sin(a)))
        result.append((t2x, t2y))

    result.append(coords[-1])
    return result


def snap_endpoints(segments: list[RoadSegment], pppm: float, snap_m: float = 8.0) -> int:
    """Snap dangling segment endpoints onto nearby roads."""
    snap_px = snap_m * pppm
    cell = max(snap_px, 1.0)
    grid: dict[tuple[int, int], list[int]] = {}

    for i, seg in enumerate(segments):
        minx, maxx = sorted((seg.x1, seg.x2))
        miny, maxy = sorted((seg.y1, seg.y2))
        for cx in range(int(minx // cell), int(maxx // cell) + 1):
            for cy in range(int(miny // cell), int(maxy // cell) + 1):
                grid.setdefault((cx, cy), []).append(i)

    snapped = 0
    for i, seg in enumerate(segments):
        for xa, ya in (("x1", "y1"), ("x2", "y2")):
            px, py = getattr(seg, xa), getattr(seg, ya)
            cx, cy = int(px // cell), int(py // cell)
            best_d = snap_px
            best_pt = None
            for gx in (cx - 1, cx, cx + 1):
                for gy in (cy - 1, cy, cy + 1):
                    for j in grid.get((gx, gy), ()):
                        if j == i:
                            continue
                        o = segments[j]
                        d, qx, qy = point_to_segment(px, py, o.x1, o.y1, o.x2, o.y2)
                        if d < best_d:
                            best_d = d
                            best_pt = (qx, qy)
            if best_pt is not None and best_d > 0.5:
                setattr(seg, xa, best_pt[0])
                setattr(seg, ya, best_pt[1])
                snapped += 1

    return snapped