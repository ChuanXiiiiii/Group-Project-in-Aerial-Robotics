"""
kml_parser.py — Extract polygon coordinates from KML files.
=============================================================
Parses AENGM0074.kml to extract named polygon boundaries:
    - Flight Area         (inclusion fence)
    - SSSI                (exclusion fence)
    - Search Area         (pattern generation — not a fence)
    - Take-Off Location   (single point)

Uses only the Python standard library (xml.etree) — no external
KML dependencies needed on the Pi.
"""

import xml.etree.ElementTree as ET


# KML uses this namespace for all elements
KML_NS = "{http://www.opengis.net/kml/2.2}"


def parse_kml(filepath):
    """
    Parse a KML file and extract all named Placemarks with polygon
    or point geometry.

    Returns a dict keyed by Placemark name:
        {
            "Flight Area": [(lat, lon), (lat, lon), ...],
            "SSSI":        [(lat, lon), (lat, lon), ...],
            "Search Area": [(lat, lon), (lat, lon), ...],
            "Take-Off Location": [(lat, lon)],
        }

    Coordinates are returned as (lat, lon) float tuples.
    KML stores coordinates as lon,lat,alt — this function swaps
    to lat,lon for consistency with ArduPilot and pymavlink.
    """
    tree = ET.parse(filepath)
    root = tree.getroot()

    placemarks = {}

    for pm in root.iter(f"{KML_NS}Placemark"):
        # Get name
        name_el = pm.find(f"{KML_NS}name")
        if name_el is None or name_el.text is None:
            continue
        name = name_el.text.strip()

        # Try Polygon first
        polygon = pm.find(f".//{KML_NS}Polygon")
        if polygon is not None:
            coords_el = polygon.find(
                f".//{KML_NS}outerBoundaryIs/{KML_NS}LinearRing/{KML_NS}coordinates"
            )
            if coords_el is not None and coords_el.text:
                coords = _parse_coordinate_string(coords_el.text)
                # KML polygons repeat the first point at the end — remove it
                if len(coords) > 1 and coords[0] == coords[-1]:
                    coords = coords[:-1]
                placemarks[name] = coords
                continue

        # Try Point (for TOL)
        point = pm.find(f".//{KML_NS}Point")
        if point is not None:
            coords_el = point.find(f"{KML_NS}coordinates")
            if coords_el is not None and coords_el.text:
                coords = _parse_coordinate_string(coords_el.text)
                placemarks[name] = coords
                continue

    return placemarks


def _parse_coordinate_string(text):
    """
    Parse a KML coordinate string into a list of (lat, lon) tuples.

    KML format: "lon,lat,alt lon,lat,alt ..."
    We swap to (lat, lon) and discard altitude.
    """
    coords = []
    for token in text.strip().split():
        parts = token.split(",")
        if len(parts) >= 2:
            lon = float(parts[0])
            lat = float(parts[1])
            coords.append((lat, lon))
    return coords


def get_fence_polygons(kml_path):
    """
    Convenience function: parse the KML and return the two fence
    polygons needed for the SAR mission.

    Returns:
        (flight_area, sssi) — each a list of (lat, lon) tuples.

    Raises KeyError if the expected Placemarks are not found.
    The function tries common name variations to be robust against
    minor KML naming differences.
    """
    data = parse_kml(kml_path)

    # Flight Area — try common names
    flight_area = None
    for name in ["Flight Area", "FlightArea", "flight area", "Flight area"]:
        if name in data:
            flight_area = data[name]
            break
    if flight_area is None:
        raise KeyError(
            f"'Flight Area' not found in KML. "
            f"Available placemarks: {list(data.keys())}"
        )

    # SSSI — try common names
    sssi = None
    for name in ["SSSI", "sssi", "Sssi"]:
        if name in data:
            sssi = data[name]
            break
    if sssi is None:
        raise KeyError(
            f"'SSSI' not found in KML. "
            f"Available placemarks: {list(data.keys())}"
        )

    return flight_area, sssi


def get_search_area(kml_path):
    """
    Extract the Search Area polygon from the KML.
    Returns a list of (lat, lon) tuples.
    """
    data = parse_kml(kml_path)

    for name in ["Search Area", "SearchArea", "search area", "Search area"]:
        if name in data:
            return data[name]

    raise KeyError(
        f"'Search Area' not found in KML. "
        f"Available placemarks: {list(data.keys())}"
    )


def get_tol(kml_path):
    """
    Extract the Take-Off Location from the KML.
    Returns a single (lat, lon) tuple.
    """
    data = parse_kml(kml_path)

    for name in ["Take-Off Location", "TOL", "Take Off Location",
                  "TakeOffLocation", "Takeoff Location"]:
        if name in data:
            coords = data[name]
            if coords:
                return coords[0]

    raise KeyError(
        f"'Take-Off Location' not found in KML. "
        f"Available placemarks: {list(data.keys())}"
    )
