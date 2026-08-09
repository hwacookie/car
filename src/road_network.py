# Road Network
# Stores the road graph, projects geographic coords to world pixels,
# and provides spatial queries (e.g. "what road is at this position?").

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from . import config


@dataclass
class RoadSegment:
    """A single road segment between two points."""
    id: int
    x1: float       # world pixel coords
    y1: float
    x2: float
    y2: float
    highway: str
    oneway: bool
    width: float    # metres


@dataclass
class RoadNetwork:
    nodes: dict                # id -> (x, y) in world pixels
    segments: list[RoadSegment]
    origin_lat: float          # south-west corner lat
    origin_lon: float          # south-west corner lon
    world_width: float         # total world pixels (lon span * scale)
    world_height: float        # total world pixels (lat span * scale)
    node_max_width: dict = field(default_factory=dict)  # node id -> (half_width_px, highway)

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

        # Build segments
        segments = []
        # Track widest road at each node (for junction circles): node -> (half_width_px, highway)
        node_info: dict[str, tuple[float, str]] = {}
        # Count how many segments touch each node (to detect real intersections)
        node_degree: dict[str, int] = {}
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

                # Determine width from highway type
                road_cfg = config.ROAD_TYPES.get(highway)
                if road_cfg:
                    width = road_cfg["width_1way"] if oneway else road_cfg["width_2way"]
                else:
                    width = 3.5

                segments.append(RoadSegment(
                    id=way["id"],
                    x1=x1, y1=y1, x2=x2, y2=y2,
                    highway=highway,
                    oneway=oneway,
                    width=width,
                ))

                # Record widest road touching each endpoint node
                half = (width / 2) * pppm
                for nid in (n1, n2):
                    if nid in nodes:
                        node_degree[nid] = node_degree.get(nid, 0) + 1
                        cur = node_info.get(nid)
                        if cur is None or half > cur[0]:
                            node_info[nid] = (half, highway)

        # Only keep junction info for real intersections (degree >= 3) and
        # dead ends (degree == 1, for rounded caps). Intermediate nodes along
        # a single road (degree == 2) would create ugly bulges.
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
            node_max_width=node_info,
        )

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

    def random_road_point(self) -> tuple[float, float, float]:
        """Return a random (x, y, heading) on any road segment. Heading in degrees, 0=up."""
        import random
        seg = random.choice(self.segments)
        t = random.random()
        x = seg.x1 + t * (seg.x2 - seg.x1)
        y = seg.y1 + t * (seg.y2 - seg.y1)
        heading = math.degrees(math.atan2(seg.x2 - seg.x1, seg.y2 - seg.y1))
        return x, y, heading

    # --- Bounds ---

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """(left, top, right, bottom) in world pixels."""
        return 0, 0, self.world_width, self.world_height


# --- Projection helpers ---

def latlon_to_world(lat: float, lon: float, ref_lat: float, ref_lon: float, pppm: float) -> tuple[float, float]:
    """Convert lat/lon to world pixel coordinates using equirectangular projection.

    origin is bottom-left (ref_lat, ref_lon).
    x increases east, y increases north.
    """
    # Approximate metres-per-degree, adjusted for latitude
    meters_per_lat_deg = 111132.9 - 566.0 * math.cos(2 * math.radians(lat)) + 1.2 * math.cos(4 * math.radians(lat))
    meters_per_lon_deg = 111320 * math.cos(math.radians(lat))

    dx_m = (lon - ref_lon) * meters_per_lon_deg
    dy_m = (lat - ref_lat) * meters_per_lat_deg

    return dx_m * pppm, dy_m * pppm


def point_to_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Shortest distance from point (px, py) to line segment (x1,y1)-(x2,y2)."""
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy

    if length_sq == 0:
        # Degenerate segment
        return math.hypot(px - x1, py - y1)

    # Project point onto line, clamp to segment
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return math.hypot(px - proj_x, py - proj_y)
