# OSM Data Loader
# Fetches road network from the OSM-Wars PostgreSQL database.

from __future__ import annotations

import psycopg
from psycopg.rows import dict_row

# PostgreSQL connection defaults (matches OSM-Wars .env)
DB_CONFIG = {
    "host": "localhost",
    "port": 5432,
    "dbname": "osm_wars",
    "user": "osm_wars",
    "password": "osm_wars",
}

# highway_type_id -> highway tag name
# NOTE: must stay in sync with OSM-Wars src/importer/common_data.py HIGHWAY_IDS
HIGHWAY_ID_TO_NAME = {
    1: "motorway",
    2: "motorway_link",
    3: "trunk",
    4: "trunk_link",
    5: "primary",
    6: "primary_link",
    7: "secondary",
    8: "secondary_link",
    9: "tertiary",
    10: "tertiary_link",
    11: "unclassified",
    12: "residential",
    13: "living_street",
    14: "service",
    15: "track",
    16: "road",
}

# highway tags treated as drivable roads (others, e.g. track, excluded)
DRIVABLE_HIGHWAY_IDS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16)


def _connect():
    return psycopg.connect(**DB_CONFIG, row_factory=dict_row)


def _get_map_schema(conn) -> str:
    """Detect which schema holds the imported road_geometry table."""
    rows = conn.execute(
        "SELECT table_schema FROM information_schema.tables "
        "WHERE table_name = 'road_geometry' "
        "AND table_schema NOT IN ('public', 'pg_catalog', 'information_schema')"
    ).fetchall()
    if not rows:
        raise RuntimeError("No imported map schema found. Import a map into the OSM-Wars DB first.")
    for row in rows:
        schema = row["table_schema"]
        if schema != "osm_wars":
            return schema
    return rows[0]["table_schema"]


def fetch_osm_data(
    north: float,
    south: float,
    west: float,
    east: float,
) -> dict:
    """Query the OSM-Wars DB for drivable roads in the bounding box (lat/lon).

    Returns a dict with keys:
        nodes  : {node_id: {"lat": float, "lon": float}}
        ways   : [ {
                    "id": int,
                    "nodes": [node_id, node_id],
                    "highway": str,
                    "oneway": bool,
                }, ... ]
    """
    conn = _connect()
    try:
        map_schema = _get_map_schema(conn)
        print(f"  Using map schema: {map_schema}")

        ids = ",".join(str(i) for i in DRIVABLE_HIGHWAY_IDS)
        # road_geometry.geom is EPSG:3857; filter with a transformed envelope
        rows = conn.execute(f"""
            SELECT
                id,
                oneway,
                highway_type_id,
                ST_X(ST_Transform(ST_StartPoint(geom), 4326)) AS lon1,
                ST_Y(ST_Transform(ST_StartPoint(geom), 4326)) AS lat1,
                ST_X(ST_Transform(ST_EndPoint(geom), 4326)) AS lon2,
                ST_Y(ST_Transform(ST_EndPoint(geom), 4326)) AS lat2
            FROM "{map_schema}".road_geometry
            WHERE highway_type_id IN ({ids})
              AND geom && ST_Transform(
                  ST_MakeEnvelope(%s, %s, %s, %s, 4326), 3857)
        """, (west, south, east, north)).fetchall()

        print(f"  Fetched {len(rows)} road segments")

        nodes: dict[str, dict] = {}
        ways: list[dict] = []

        for row in rows:
            n1 = f"{row['lat1']:.7f}_{row['lon1']:.7f}"
            n2 = f"{row['lat2']:.7f}_{row['lon2']:.7f}"
            nodes[n1] = {"lat": row["lat1"], "lon": row["lon1"]}
            nodes[n2] = {"lat": row["lat2"], "lon": row["lon2"]}

            highway = HIGHWAY_ID_TO_NAME.get(row["highway_type_id"], "residential")
            oneway = row["oneway"] in ("yes", "1", "true")

            ways.append({
                "id": row["id"],
                "nodes": [n1, n2],
                "highway": highway,
                "oneway": oneway,
            })

        return {"nodes": nodes, "ways": ways}
    finally:
        conn.close()
