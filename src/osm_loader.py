# OSM Data Loader
# Fetches road network from Overpass API for a given bounding box.

import json
import requests

from . import config


def fetch_osm_data(
    north: float,
    south: float,
    west: float,
    east: float,
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
    # Build Overpass QL query
    drivable = " or ".join(
        f'highway="{tag}"' for tag in config.DRIVABLE_ROADS
    )

    query = f"""
    [out:json][timeout:30];
    (
      way["highway"]["area"!="yes"]
        ({south},{west},{north},{east})
        [{drivable}];
    );
    out body;
    >;
    out skel qt;
    """

    url = "https://overpass-api.de/api/interpreter"
    resp = requests.post(url, data={"data": query}, timeout=30)
    resp.raise_for_status()
    data = resp.json()

    return parse_osm_response(data)


def parse_osm_response(data: dict) -> dict:
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
            if highway not in config.DRIVABLE_ROADS:
                continue

            oneway = tags.get("oneway") in ("yes", "1", "true")

            ways.append({
                "id": elem["id"],
                "nodes": elem["nodes"],
                "highway": highway,
                "oneway": oneway,
            })

    return {"nodes": nodes, "ways": ways}
