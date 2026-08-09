# Configuration

import pygame

# --- Window ---
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# --- Target area: Bremen city center (~3 x 3 km around Marktplatz) ---
# Bounding box (lat, lon)
BOUNDING_BOX = {
    "north": 53.0893,
    "south": 53.0623,
    "west": 8.7848,
    "east": 8.8296,
}

# --- Projection ---
# Pixels per meter at zoom level 1.0
PIXELS_PER_METER = 2

# Zoom limits: viewport width = WINDOW_WIDTH / (zoom * PIXELS_PER_METER)
# MAX_ZOOM  -> ~40 m viewport (zoomed in)
# MIN_ZOOM  -> ~1000 m viewport (zoomed out)
MAX_ZOOM = 16.0
MIN_ZOOM = 0.64
ZOOM_STEP = 1.15

# --- Car ---
CAR_SPEED = 10          # meters/second
CAR_ACCELERATION = 15   # meters/second^2
CAR_BRAKING = 25        # meters/second^2
CAR_TURN_SPEED = 180    # degrees/second
CAR_LENGTH = 4.4        # meters
CAR_WIDTH = 1.8         # meters

# --- Road rendering ---
# Real-world widths in meters (2-way / 1-way)
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
    "living_street": {"color": (187, 187, 187),  "width_2way": 7,  "width_1way": 3.5},
    "road":          {"color": (187, 187, 187),  "width_2way": 7,  "width_1way": 3.5},
    "service":       {"color": (204, 204, 204),  "width_2way": 7,  "width_1way": 3.5},
}

# Drivable highway tags (subset)
DRIVABLE_ROADS = set(ROAD_TYPES.keys())

# --- Colors ---
BG_COLOR = (34, 120, 34)       # grass green
ROAD_EDGE_COLOR = (50, 50, 50) # road markings
MINIMAP_BG = (20, 60, 20)
MINIMAP_CAR_COLOR = (255, 0, 0)
MINIMAP_BORDER = (80, 80, 80)

# --- Minimap ---
MINIMAP_SIZE = 180
MINIMAP_MARGIN = 15
MINIMAP_XRANGE = 0.032  # lon
MINIMAP_YRANGE = 0.0167 # lat
