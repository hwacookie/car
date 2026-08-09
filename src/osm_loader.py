# OSM Data Loader
# Fetches road network from Overpass API for a given bounding box.

import json
import os
import time
import requests

from . import config

# Cache file for downloaded OSM data
_CACHE_DIR = os.path.expanduser("~/.cache/cargame")
_CACHE_FILE = os.path.join(_CACHE_DIR, "osm_data.json")


def _cache_key(north, south, west, east) -> str:
    return f"_{north}_{south}_{west}_{east}"


def fetch_osm_data(
    north: float,
    south: float,
    west: float,
    east: float,
    use_cache: bool = True,
) -> dict:
    """Query Overpass API for drivable roads in the bounding box.

    Returns a dict with keys:
        nodes  : {node_id: {"lat": float, "lon": float}}
        ways   : [ {
                    "id": int,
                    "nodes": [node_id, ...],
                    "highway": str,
                    "oneway": bool,
                }, ... ]
    """
    # Try cache first
    if use_cache and os.path.exists(_CACHE_FILE):
        print("  Loading cached OSM data…")
        with open(_CACHE_FILE) as f:
            return json.load(f)

    print("  Fetching from Overpass API (this may take a moment)…")
    all_nodes = {}
    all_ways = []

    for i, highway in enumerate(config.DRIVABLE_ROADS):
        print(f"  Fetching {highway} ({i+1}/{len(config.DRIVABLE_ROADS)})…")

        query = (
            f'[out:json][timeout:25];'
            f'way["highway"="{highway}"]["area"!="yes"]'
            f'({south},{west},{north},{east});'
            f'out body;'
            f'>'
            f';out skel qt;'
        )

        url = "https://lz4.overpass-api.de/api/interpreter"
        while True:
            resp = requests.post(
                url,
                data={"data": query},
                headers={"User-Agent": "CarGame/1.0"},
                timeout=30,
            )
            if resp.status_code == 429:
                wait = int(resp.headers.get("Retry-After", 15))
                print(f"    Rate limited, waiting {wait}s…")
                time.sleep(wait)
                continue
            resp.raise_for_status()
            break

        data = resp.json()
        parsed = parse_osm_response(data, highway)
        all_nodes.update(parsed["nodes"])
        all_ways.extend(parsed["ways"])

        # Be polite — wait between requests
        time.sleep(1.0)

    result = {"nodes": all_nodes, "ways": all_ways}

    # Save to cache
    os.makedirs(_CACHE_DIR, exist_ok=True)
    with open(_CACHE_FILE, "w") as f:
        json.dump(result, f)

    return result


def parse_osm_response(data: dict, highway_filter: str) -> dict:
    """Parse raw Overpass JSON into nodes and ways dicts."""

    # Index nodes
    nodes = {}
    for elem in data.get("elements", []):
        if elem["type"] == "node":
            nodes[elem["id"]] = {
                "lat": elem["lat"],
                "lon": elem["lon"],
            }

    # Index ways
    ways = []
    for elem in data.get("elements", []):
        if elem["type"] == "way":
            tags = elem.get("tags", {})
            highway = tags.get("highway")
            if highway != highway_filter:
                continue

            oneway = tags.get("oneway") in ("yes", "1", "true")

            ways.append({
                "id": elem["id"],
                "nodes": elem["nodes"],
                "highway": highway,
                "oneway": oneway,
            })

    return {"nodes": nodes, "ways": ways}
