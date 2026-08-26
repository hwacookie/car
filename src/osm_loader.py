# OSM Data Loader
# Fetches road network from the OSM-Wars PostgreSQL database, with a
# file-based cache so the game runs without any database once a bounding
# box has been fetched once.

from __future__ import annotations

import hashlib
import json
import os

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
# Levels 1-7 only: motorway, trunk, primary, secondary, tertiary, residential, service
# (including _link variants)
DRIVABLE_HIGHWAY_IDS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14)


def _connect():
    return psycopg.connect(**DB_CONFIG, row_factory=dict_row)


# --- File cache -----------------------------------------------------------
# After the first successful DB fetch, the result is stored as JSON under
# data/osm_cache/ keyed by bounding box. Subsequent starts load from that
# file and never touch the database - so a stopped Postgres (or Docker)
# does not block the game.

def _cache_path(north: float, south: float, west: float, east: float) -> str:
    key = f"{north:.7f}_{south:.7f}_{west:.7f}_{east:.7f}"
    digest = hashlib.sha1(key.encode()).hexdigest()[:16]
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..",
        "data", "osm_cache", f"osm_{digest}.json",
    )


def _load_cache(north: float, south: float, west: float, east: float):
    """Return cached OSM data for the bbox, or None if absent/stale."""
    if os.environ.get("OSM_FORCE_REFRESH") == "1":
        return None
    path = _cache_path(north, south, west, east)
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        if not data.get("ways"):
            return None
        print(f"  Using cached OSM data: {path} "
              f"({len(data['ways'])} segments, no database needed)")
        return data
    except (OSError, json.JSONDecodeError) as e:
        print(f"  (cache unreadable, ignoring: {e})")
        return None


def _write_cache(north: float, south: float, west: float, east: float, data: dict):
    path = _cache_path(north, south, west, east)
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f)
        print(f"  Cached OSM data to {path}")
    except OSError as e:
        print(f"  (could not write cache file: {e})")


def _get_map_schema(conn, west: float, south: float, east: float, north: float) -> str:
    """Detect which schema has road data in the given bounding box."""
    rows = conn.execute(
        "SELECT table_schema FROM information_schema.tables "
        "WHERE table_name = 'road_geometry' "
        "AND table_schema NOT IN ('public', 'pg_catalog', 'information_schema', 'osm_wars')"
    ).fetchall()
    if not rows:
        raise RuntimeError("No imported map schema found.")

    # Test each schema for data in the bounding box
    for row in rows:
        schema = row["table_schema"]
        result = conn.execute(f"""
            SELECT COUNT(*) FROM "{schema}".road_geometry
            WHERE geom && ST_Transform(
                ST_MakeEnvelope(%s, %s, %s, %s, 4326), 3857)
        """, (west, south, east, north)).fetchone()
        cnt = list(result.values())[0] if result else 0
        if cnt > 0:
            return schema

    # Fallback: first schema with any data
    for row in rows:
        schema = row["table_schema"]
        result = conn.execute(f'SELECT COUNT(*) FROM "{schema}".road_geometry').fetchone()
        cnt = list(result.values())[0] if result else 0
        if cnt > 0:
            return schema

    raise RuntimeError("No schema has data in the requested bounding box.")


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

    The result is cached to data/osm_cache/ (see _cache_path); if a cache
    file exists for this bounding box it is returned without touching the
    database. Set OSM_FORCE_REFRESH=1 to bypass the cache.
    """
    cached = _load_cache(north, south, west, east)
    if cached is not None:
        return cached

    try:
        conn = _connect()
    except psycopg.OperationalError as e:
        raise RuntimeError(
            "Cannot reach the OSM-Wars PostgreSQL database "
            f"({DB_CONFIG['host']}:{DB_CONFIG['port']}, db '{DB_CONFIG['dbname']}').\n"
            "No local cache exists for this bounding box yet, so the map\n"
            "cannot be loaded. To fix:\n"
            "  1. Start a Postgres that has the osm_wars database with an\n"
            "     imported road_geometry schema, e.g. the OSM-Wars Docker setup:\n"
            "         cd ~/prj/OSM-Wars/docker && docker compose up -d\n"
            "     or a local Homebrew cluster (if it holds the data):\n"
            "         brew services start postgresql@14\n"
            "  2. Check connectivity:  pg_isready\n"
            "  3. Restart the game - the first successful fetch is cached to\n"
            f"     {_cache_path(north, south, west, east)}\n"
            "     and afterwards the game runs WITHOUT any database.\n"
            "(Delete that cache file or set OSM_FORCE_REFRESH=1 to re-fetch.)"
        ) from e
    try:
        map_schema = _get_map_schema(conn, west, south, east, north)
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

        data = {"nodes": nodes, "ways": ways}
        _write_cache(north, south, west, east, data)
        return data
    finally:
        conn.close()
