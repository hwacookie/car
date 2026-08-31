# Configuration

import math
import pygame

# --- Window ---
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# --- Target area: Kleinmachnow (south of Berlin) ---
# Bounding box (lat, lon)
BOUNDING_BOX = {
    "north": 52.42382,
    "west": 13.21831,
    "south": 52.40714,
    "east": 13.25033,
}

# --- Projection ---
# Pixels per meter at zoom level 1.0
PIXELS_PER_METER = 2

# Real intersections have a much wider paved "corner-cutting" area than
# the connecting roads' width alone suggests (curb radii flare the
# pavement out at junctions - confirmed via satellite imagery). This is
# the extra radius (meters) added on top of the widest connected road's
# half-width when rendering junction nodes and when validating whether a
# turning arc stays on paved surface.
JUNCTION_WIDENING_M = 4.0
ROAD_CORNER_RADIUS_M = 6.0  # visible curb-style rounding radius at road bends
# Junction corner rounding (Eckausrundung, de.wikipedia.org/wiki/Eckausrundung):
# where two road EDGES meet at a degree>=3 node, the grass corner is rounded
# with a circular arc of this radius tangent to both edges. Fixed for all
# junctions (user decision 2026-08-31: 4 m).
# TODO: the radius should depend on road class / design vehicle ("nach Art
# und Lage der Straße", RASt/RAL Pauschalwerte) - bigger where buses or
# trucks turn, smaller on quiet residential corners.
JUNCTION_CORNER_RADIUS_M = 4.0
ROAD_EDGE_TOLERANCE_M = 0.35  # shared slack between planned-arc validation and live off-road checks

# Zoom limits: viewport width = WINDOW_WIDTH / (zoom * PIXELS_PER_METER)
# MAX_ZOOM  -> ~40 m viewport (zoomed in)
# MIN_ZOOM  -> ~3000 m viewport (zoomed out)
MAX_ZOOM = 16.0
MIN_ZOOM = 0.12
ZOOM_STEP = 1.15

# --- Car ---
CAR_SPEED = 55.6        # meters/second = 200 km/h (top speed of a normal car)
CAR_ACCELERATION = 2.8  # m/s²  → 0-100 km/h in ~10s (normales Auto)
CAR_BRAKING = 10.0      # m/s²  → starke Bremsung mit ABS (~1g)
PARK_BRAKING = 3.5      # m/s²  → komfortables Bremsen beim Parken (~0,35 g)
                        #          kein Volllastbremsen) - Spec §1 (A_PARK)
PARK_CREEP_SPEED_M = 2.0  # m/s (7 km/h) → Creep-Tempo in der Verschwenkzone
                          #                (Spec §1 Band 5–10 km/h)
CAR_TURN_SPEED = 180    # degrees/second (FREE-mode arcade feel, capped below)
# Mechanical minimum turning radius: wheelbase / tan(max steer angle) - the
# same limit the BICYCLE model uses (bicycle_nav.WHEELBASE / MAX_STEER).
# No car can turn tighter than this at ANY speed, so FREE mode clamps its
# yaw rate to v / MIN_TURN_RADIUS_M (see Car._update_free_mode).
MIN_TURN_RADIUS_M = 2.7 / math.tan(math.radians(38.0))   # ~3.46 m
# FREE mode: time for the virtual steering wheel to travel from center to
# full lock (and back). Instant full lock on key-down feels twitchy and
# "too direct"; a real wheel takes a moment to sweep over.
STEER_LOCK_TIME_S = 0.35
# Reverse top speed: 30 km/h (user spec - a fast parking-lot crawl).
REVERSE_MAX_SPEED_M = 30.0 / 3.6   # ~8.33 m/s
CAR_LENGTH = 4.4        # meters
CAR_WIDTH = 1.8         # meters

# --- Axle geometry ---
# The kinematic bicycle model integrates the REAR AXLE: the rear wheels
# roll without slipping, the front wheels steer, so the rear axle is the
# pivot. Car.x / Car.y therefore ARE the rear-axle midpoint, NOT the body
# centre.
#
# Everything visual must be placed relative to that point. Drawing the
# rear wheels symmetrically about (x, y) puts them ~1.7 m BEHIND the real
# pivot, and a point behind the pivot always swings OUT of a turn - the
# rear tyre tracks then curve right while the front wheels steer left,
# which no real car does (its rear wheels sit exactly on the pivot).
#
# Proportions from assets/car_sprite.svg: front axle 62/200 of the body
# length ahead of the body centre, rear axle 58/200 behind it, tyre
# centres 36/100 of the body width outboard.
FRONT_AXLE_OFFSET_M = CAR_LENGTH * 62 / 200   # 1.364 m ahead of body centre
REAR_AXLE_OFFSET_M = CAR_LENGTH * 58 / 200    # 1.276 m behind body centre
SPRITE_WHEELBASE_M = FRONT_AXLE_OFFSET_M + REAR_AXLE_OFFSET_M   # 2.64 m
TIRE_OUTBOARD_M = CAR_WIDTH * 36 / 100        # 0.648 m from the centreline

# --- Spawning / kerb ---
# Lateral gap left between the car's flank and the pavement edge when it
# sits at the kerb (spawned, or pulled over). Small on purpose: the car
# should be AT the kerb. This used to carry an extra 0.3 m of slack to
# stop the on-road check tripping, which was only needed because the
# four-corner box was mis-placed on the rear axle instead of the body.
# Must match the inset the raceline corridor uses (CAR_WIDTH/2 +
# ROAD_EDGE_TOLERANCE_M), otherwise the car spawns nearer the kerb than
# its own driving line is ever allowed to go, i.e. outside the corridor
# it is then asked to follow.
KERB_CLEARANCE_M = ROAD_EDGE_TOLERANCE_M
# ...but a PARKED car sits closer than a driving one: it is not tracking a
# line any more, it is standing still against the kerb (spec §1 "möglichst
# nah am rechten Rand"). The pull-over target uses this smaller clearance;
# the driving-line corridor keeps KERB_CLEARANCE_M.
PARK_KERB_CLEARANCE_M = 0.16   # empirically closest clean park (scripts/sweep_park_clearance.py): at <=0.14 the reverse-in tuck grows past what _reverse_park_ok allows and the style falls back to a forward park ~0.6 m out
# Parking lanes END before junctions - you cannot park right in front of
# one. The painted P marks and the parking-lane boundary line stop this
# far short of any junction (user decision: at least 5 m, visually).
PARK_LANE_END_GAP_M = 5.0
# Painted lane markings (dashed centerlines / lane dividers) stop BEFORE
# the crossing itself: at a junction, the paint ends half the WIDEST arm's
# width plus this margin before the node centre - just short of where the
# Eckausrundung corner point C lies along the arm's axis. Without this the
# dashes cut straight across the fillet corners (user decision: no paint
# on the crossing itself).
CENTERLINE_JUNCTION_GAP_M = 0.5

# How far into a segment a car spawns, as a FRACTION of the segment
# length. Never 0: a node sits in the middle of the junction rounding,
# where the lane geometry is ambiguous and the reference line is still
# curving. Fractional rather than a fixed distance so it also works on
# very short segments - a fixed 5 m advance overshot the whole 4.16 m
# sliver approach and dumped the car past its junction.
SPAWN_PROGRESS = 0.10


# Half-width (m) of the central-difference window used to measure a
# reference line's curvature. MUST be a fixed physical length, not a
# fraction of the route: at the old `max(1.0, total * 0.01)` a 494 m
# route measured curvature over a 4.94 m window, which is wider than half
# a 9.4 m corner fillet. That smeared a 4.25 m lane radius into a
# reported 6.30 m, so the speed profile handed the car a corner speed
# needing 2.96 m/s^2 when A_LAT_MAX is 2.0 - the car then could not hold
# its own reference line and understeered wide out of every bend.
CURVATURE_WINDOW_M = 1.0

# Clearance (m) kept between the car's left flank and the road centreline
# on a two-way road. This is the hard "never enter the oncoming lane"
# bound - the lower edge of the corridor the racing line is optimised in
# (src/raceline.py).
# It also has to absorb the controller's tracking error: the corridor
# constrains the reference LINE, but pure pursuit lags it (most in tight
# bends), and the hard rule applies to the CAR. Without headroom the car
# overshoots a line that is itself exactly legal. 0.30 m keeps the body
# clear of the centreline through the test map's tightest bends.
LANE_CENTRE_MARGIN_M = 0.50

# Nominal lane position, used only where a corridor cannot be built (very
# short routes). The normal driving line comes from the raceline solver.
LANE_OFFSET_DEFAULT_M = 1.75


def kerb_offset_m(road_width_m: float) -> float:
    """Lateral offset (m) from a road's centreline to the position where
    the car sits flush against the right kerb, with KERB_CLEARANCE_M to
    spare. Shared by the spawner and by BicycleNav's pull-over/pull-out
    target so the two cannot drift apart."""
    return max(0.0, road_width_m / 2.0 - CAR_WIDTH / 2.0 - KERB_CLEARANCE_M)


def park_offset_m(road_width_m: float) -> float:
    """Lateral offset (m) of a PARKED car's centre from the centreline:
    flush against the kerb with only PARK_KERB_CLEARANCE_M to spare."""
    return max(0.0, road_width_m / 2.0 - CAR_WIDTH / 2.0
               - PARK_KERB_CLEARANCE_M)


def lane_base_offset_m(width: float, lanes: int = 0,
                       parking_lane_width: float = 0.0,
                       oneway: bool = False) -> float:
    """Nominal driving position (m right of the centreline) for a road.

    Multi-lane carriageways (lanes > 0) pin to the CENTRE OF THE OUTERMOST
    DRIVING LANE - a human keeps right, so normal traffic drives next to
    the parking lane, not in it. One-way: all lanes run one way, so the
    driving strip is the full width. Two-way: each side has its own
    driving strip (half the road minus the parking lane). Plain roads keep
    the fixed nominal offset."""
    if lanes > 0 and not oneway:
        d = max(0.0, width / 2.0 - parking_lane_width)   # per side
        return max(0.0, d - (d / lanes) / 2.0)
    if lanes > 0:                                        # one-way
        l = width / lanes
        return max(0.0, width / 2.0 - l / 2.0)
    return min(LANE_OFFSET_DEFAULT_M, kerb_offset_m(width))

# --- Road rendering ---
# Real-world widths in meters (2-way / 1-way)
# Levels 1-7: motorway, trunk, primary, secondary, tertiary, residential, unclassified
ROAD_TYPES = {
    "motorway":      {"color": (68, 68, 68),     "width_2way": 14, "width_1way": 7},
    "motorway_link": {"color": (68, 68, 68),     "width_2way": 7,  "width_1way": 3.5},
    "trunk":         {"color": (85, 85, 85),     "width_2way": 10, "width_1way": 7},
    "trunk_link":    {"color": (85, 85, 85),     "width_2way": 7,  "width_1way": 3.5},
    "primary":       {"color": (102, 102, 102),  "width_2way": 10, "width_1way": 7},
    "primary_link":  {"color": (102, 102, 102),  "width_2way": 7,  "width_1way": 3.5},
    "secondary":     {"color": (136, 136, 136),  "width_2way": 7,  "width_1way": 3.5},
    "secondary_link":{"color": (136, 136, 136),  "width_2way": 7,  "width_1way": 3.5},
    "tertiary":      {"color": (153, 153, 153),  "width_2way": 7,  "width_1way": 3.5},
    "tertiary_link": {"color": (153, 153, 153),  "width_2way": 7,  "width_1way": 3.5},
    "residential":   {"color": (170, 170, 170),  "width_2way": 7,  "width_1way": 3.5},
    "unclassified":  {"color": (187, 187, 187),  "width_2way": 7,  "width_1way": 3.5},
    "service":       {"color": (204, 204, 204),  "width_2way": 3.5, "width_1way": 3.5},
}

# Drivable highway tags (subset) - levels 1-7
DRIVABLE_ROADS = set(ROAD_TYPES.keys())

# --- Colors ---
BG_COLOR = (34, 120, 34)       # grass green
ROAD_EDGE_COLOR = (50, 50, 50) # road markings
# All road types are drawn in this ONE uniform asphalt color (the per-type
# grays of ROAD_TYPES are only used for widths now).
ROAD_COLOR = (120, 120, 124)
# Centerline rule: every two-way road at least as wide as the test map's
# standard road (7 m) gets a dashed white centerline. Narrow service lanes
# (3.5 m) and one-ways don't. (Class-based filtering was tried first - but
# Kleinmachnow has no motorway/trunk/primary at all, its normal streets are
# 'secondary'/'residential', all 7 m wide like the test map.)
CENTERLINE_MIN_WIDTH_M = 7.0
# Junction dot (white circle at 3+-way nodes): physical size 30 cm in
# diameter, rendered in world space so it scales with zoom (1 px floor so
# it stays visible at low zoom - user decision).
JUNCTION_DOT_RADIUS_M = 0.15
MINIMAP_BG = (20, 60, 20)
MINIMAP_CAR_COLOR = (255, 0, 0)
MINIMAP_BORDER = (80, 80, 80)

# --- Minimap ---
MINIMAP_SIZE = 180
MINIMAP_MARGIN = 15
MINIMAP_XRANGE = 0.032  # lon
MINIMAP_YRANGE = 0.0167 # lat