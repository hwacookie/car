"""Tests for road_network — projection and spatial queries."""

import math
import unittest
from src.road_network import (
    RoadNetwork,
    RoadSegment,
    latlon_to_world,
    point_to_segment_distance,
)
from src import config


class TestProjection(unittest.TestCase):
    def test_origin_is_zero(self):
        """Origin should project to (0, 0)."""
        x, y = latlon_to_world(52.4, 13.2, 52.4, 13.2, config.PIXELS_PER_METER)
        self.assertAlmostEqual(x, 0, places=1)
        self.assertAlmostEqual(y, 0, places=1)

    def test_north_is_positive_y(self):
        """Moving north (higher lat) should increase y."""
        _, y = latlon_to_world(52.41, 13.2, 52.4, 13.2, config.PIXELS_PER_METER)
        self.assertGreater(y, 0)

    def test_east_is_positive_x(self):
        """Moving east (higher lon) should increase x."""
        x, _ = latlon_to_world(52.4, 13.21, 52.4, 13.2, config.PIXELS_PER_METER)
        self.assertGreater(x, 0)

    def test_distance_approximately_linear(self):
        """100m north ≈ 100 * PIXELS_PER_METER pixels."""
        _, y = latlon_to_world(52.4 + 0.0009, 13.2, 52.4, 13.2, config.PIXELS_PER_METER)
        # 0.0009 degrees ≈ 100m
        self.assertAlmostEqual(y / 100, config.PIXELS_PER_METER, delta=1.0)


class TestPointToSegment(unittest.TestCase):
    def test_on_segment(self):
        """Point on segment has zero distance."""
        d = point_to_segment_distance(5, 5, 0, 0, 10, 10)
        self.assertAlmostEqual(d, 0, places=1)

    def test_perpendicular_distance(self):
        """Point 3 units from segment has distance ~3."""
        d = point_to_segment_distance(5, 3, 0, 0, 10, 0)
        self.assertAlmostEqual(d, 3, places=1)

    def test_beyond_end(self):
        """Point beyond segment end → distance to nearest endpoint."""
        d = point_to_segment_distance(15, 5, 0, 0, 10, 0)
        self.assertAlmostEqual(d, math.hypot(5, 5), places=1)


class TestRoadNetwork(unittest.TestCase):
    def setUp(self):
        """Create a simple T-intersection network."""
        # Straight road east-west, one branch north
        self.network = RoadNetwork(
            nodes={
                "A": (0, 100),
                "B": (200, 100),
                "C": (200, 0),
            },
            segments=[
                RoadSegment(id=1, x1=0, y1=100, x2=200, y2=100,
                           highway="residential", oneway=False, width=7),
                RoadSegment(id=2, x1=200, y1=100, x2=200, y2=0,
                           highway="residential", oneway=False, width=7),
            ],
            origin_lat=52.40714,
            origin_lon=13.21831,
            world_width=300,
            world_height=200,
        )

    def test_is_on_road_midpoint(self):
        """Midpoint of a segment should be on road."""
        self.assertTrue(self.network.is_on_road(100, 100))

    def test_is_on_road_off_segment(self):
        """Point far from segments should be off road."""
        self.assertFalse(self.network.is_on_road(100, 0))

    def test_random_road_point_returns_valid(self):
        """Random point should always be on road."""
        x, y, _heading, _seg_idx, _node_id = self.network.random_road_point()
        self.assertTrue(self.network.is_on_road(x, y))


if __name__ == "__main__":
    import math
    unittest.main()
