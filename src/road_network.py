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

        # For oneway roads, respect direction
        filtered = []
        for idx, angle in candidates:
            seg = self.segments[idx]
            # Simple check: if oneway, only allow if we're going with the flow
            # (this is approximate; proper check would need edge direction)
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
        """Check if a world position is on any road."""
        pppm = config.PIXELS_PER_METER
        for seg in self.segments:
            half_width = (seg.width / 2) * pppm
            dist = point_to_segment_distance(wx, wy, seg.x1, seg.y1, seg.x2, seg.y2)
            if dist <= half_width:
                return True
        return False

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