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
    start_points: dict = field(default_factory=dict)      # name -> (x, y, heading, seg_idx, forward)

    def get_start_point(self, name: str) -> tuple[float, float, float, int, bool]:
        """Look up a named, deterministic start point (defined by synthetic
        test maps). Returns (x, y, heading, seg_idx, forward)."""
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

    def get_paved_polygon(self):
        """A single unioned Shapely (Multi)Polygon covering the entire
        drivable paved area (every road, rounded bends and junction
        fillets included, holes like a roundabout's island properly
        excluded) - the authoritative geometry for on-road checks, built
        from the exact same per-color polygons used for rendering.
        Cached - the road network never changes at runtime."""
        if getattr(self, "_paved_polygon_cache", None) is None:
            from shapely.geometry import Polygon
            from shapely.ops import unary_union
            polys = [
                Polygon(ext, holes)
                for _color, exteriors in self.get_road_polygons_by_color()
                for ext, holes in exteriors
            ]
            self._paved_polygon_cache = unary_union(polys) if polys else Polygon()
        return self._paved_polygon_cache

    def get_centerlines(self):
        """Merged, corner-rounded road CENTERLINES (no buffering) - used
        for drawing the dashed lane-marking line down the middle of each
        road. Reuses the exact same merge-through-plain-bends +
        corner-rounding logic as the paved-area polygons (see
        _build_road_polygons), so the markings visually follow the same
        smooth curve as the road edges instead of cutting straight across
        a rounded bend. Returns a list of coordinate lists (one per
        merged line). Cached - the road network never changes at
        runtime."""
        if getattr(self, "_centerlines_cache", None) is None:
            self._centerlines_cache = _build_centerlines(self)
        return self._centerlines_cache

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


def _merge_and_round_lines(network: "RoadNetwork", only_two_way: bool = False):
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
        length = math.hypot(seg.x2 - seg.x1, seg.y2 - seg.y1)
        if length < 1e-6:
            continue
        groups.setdefault((seg.highway, seg.width), []).append(seg)

    result: dict[tuple[str, float], list] = {}
    for key, segs in groups.items():
        lines = [LineString([(s.x1, s.y1), (s.x2, s.y2)]) for s in segs]
        merged = linemerge(lines) if len(lines) > 1 else lines[0]
        merged_lines = merged.geoms if hasattr(merged, "geoms") else [merged]
        result[key] = [
            _round_polyline_corners(list(line.coords), corner_radius_px)
            for line in merged_lines
        ]
    return result


def _build_centerlines(network: "RoadNetwork"):
    """Flatten _merge_and_round_lines()'s per-group coordinate lists into
    one plain list of polylines (color/width no longer matter for a
    lane-marking line). One-way segments are excluded - see
    _merge_and_round_lines(only_two_way=...)."""
    groups = _merge_and_round_lines(network, only_two_way=True)
    return [coords for lines in groups.values() for coords in lines]


def _build_road_polygons(network: "RoadNetwork"):
    """Merge connected same-width road segments into continuous lines,
    round off the CENTERLINE's own corners with a real arc of visible
    radius at every bend (a plain buffer's 'round join' only curves the
    outer/convex edge of a bend and leaves the inner edge a perfectly
    sharp mitre - correct for a generic stroke, but not what an actual
    paved road corner looks like), and only then buffer the
    already-smooth line into a fillable polygon. Grouped by (highway
    type, width) for coloring. Returns a list of
    (color, [exterior_ring_coords, ...]) - polygon holes are ignored
    (roads in practice don't produce meaningful ones)."""
    from shapely.geometry import LineString
    from shapely.ops import unary_union

    pppm = config.PIXELS_PER_METER
    groups = _merge_and_round_lines(network)

    result = []
    for (highway, width), smooth_lines in groups.items():
        half_w_px = (width / 2) * pppm

        buffered_parts = []
        for smooth_coords in smooth_lines:
            smooth_line = LineString(smooth_coords)
            buffered_parts.append(
                smooth_line.buffer(half_w_px, cap_style="round", join_style="round", resolution=8)
            )
        buffered = unary_union(buffered_parts)

        color = config.ROAD_TYPES.get(highway, {}).get("color", (150, 150, 150))
        polys = buffered.geoms if hasattr(buffered, "geoms") else [buffered]
        exteriors = [(list(p.exterior.coords), [list(r.coords) for r in p.interiors]) for p in polys if not p.is_empty]
        result.append((color, exteriors))

    result.extend(_build_multiway_junction_fillets(network, pppm))
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
            buffered = LineString(smooth).buffer(
                half_w_px, cap_style="round", join_style="round", resolution=8
            )
            color = config.ROAD_TYPES.get(seg_a.highway, {}).get("color", (150, 150, 150))
            polys = buffered.geoms if hasattr(buffered, "geoms") else [buffered]
            exteriors = [(list(p.exterior.coords), [list(r.coords) for r in p.interiors]) for p in polys if not p.is_empty]
            extras.append((color, exteriors))
    return extras


def _round_polyline_corners(coords, radius, arc_steps=10):
    """Replace every interior vertex of a polyline with a circular arc
    of the given radius, tangent to both adjoining edges - i.e. actually
    round the line's own corners, not just its eventual stroke outline.
    Endpoints are left untouched."""
    if len(coords) < 3 or radius <= 0:
        return coords

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