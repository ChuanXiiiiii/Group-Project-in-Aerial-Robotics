"""
waypoint_generator.py — Teammate's focus area path planner.
=============================================================
Greedy lawnmower pattern over an arbitrary polygon using Shapely
strip decomposition.  Used by REPLAN for the PLB Focus Area search.

Public API
----------
    generate_waypoints(polygon_coords, width, angle, altitude)

    polygon_coords : list of (lon, lat) tuples  <- NOTE: lon first
    width          : strip width in metres
    angle          : scan line rotation in degrees
    altitude       : AGL altitude for every waypoint

    Returns: list of {"lat": float, "lon": float, "alt": float}

IMPORTANT — coordinate order:
    This module expects (lon, lat) input.  The KML parser returns
    (lat, lon).  Callers must swap before passing coordinates in.
"""

import numpy as np
import shapely
import shapely.ops
import shapely.affinity
from pyproj import Transformer
from shapely.geometry import Polygon

INPUT_EPSG = "EPSG:4326"
OUTPUT_EPSG = "EPSG:32630"


def gps_to_polygon(gps_coords):
    """Convert a list of (lon, lat) GPS tuples into a Shapely polygon in metres."""
    to_meters = Transformer.from_crs(INPUT_EPSG, OUTPUT_EPSG, always_xy=True)
    meter_coords = [to_meters.transform(lon, lat) for lon, lat in gps_coords]
    return Polygon(meter_coords)


def split_poly(poly, width):
    """Split the polygon into horizontal slices of equal width."""
    min_x, min_y, max_x, max_y = poly.bounds
    y_splits = np.arange(min_y + width, max_y, width)

    res = poly
    for y in y_splits:
        split_line = shapely.LineString([(min_x, y), (max_x, y)])
        res = shapely.MultiPolygon(shapely.ops.split(res, split_line))

    return res


def make_strips(splits):
    """Convert clipped polygon slices into rectangular strip envelopes."""
    return shapely.MultiPolygon([g.envelope for g in splits.geoms])


def build_path_greedy(strips):
    """
    Build a lawnmower path using a greedy nearest-strip heuristic.
    Each strip contributes two endpoints: left midpoint and right midpoint.
    """
    pairs = []

    for s in strips.geoms:
        min_x, min_y, max_x, max_y = s.bounds
        pt_left = shapely.Point(min_x, 0.5 * (min_y + max_y))
        pt_right = shapely.Point(max_x, 0.5 * (min_y + max_y))
        pairs.append((pt_left, pt_right))

    path = []
    current_pt = shapely.Point(0, 0)

    while pairs:
        dists = [
            (current_pt.distance(pt_left), current_pt.distance(pt_right))
            for pt_left, pt_right in pairs
        ]

        next_strip = int(np.argmin([min(ds) for ds in dists]))
        next_end = int(np.argmin(dists[next_strip]))

        path.append(pairs[next_strip][next_end])
        path.append(pairs[next_strip][1 - next_end])

        current_pt = pairs[next_strip][1 - next_end]
        pairs.pop(next_strip)

    return shapely.LineString(path)


def lawnmower(poly, width, angle=0, use_radians=False):
    """
    Generate a rotated lawnmower coverage path.

    1. Rotate polygon into the planner frame
    2. Split into equal-width slices
    3. Convert slices into rectangular strips
    4. Build greedy traversal path
    5. Rotate path and strips back to the original frame
    """
    poly_r = shapely.affinity.rotate(poly, -angle, origin=(0, 0), use_radians=use_radians)

    splits_r = split_poly(poly_r, width)
    strips_r = make_strips(splits_r)
    path_r = build_path_greedy(strips_r)

    strips = shapely.affinity.rotate(strips_r, angle, origin=(0, 0), use_radians=use_radians)
    path = shapely.affinity.rotate(path_r, angle, origin=(0, 0), use_radians=use_radians)

    return path, strips


def path_to_waypoints(path, altitude=10.0):
    """Convert a planner path into a list of GPS waypoint dicts."""
    to_gps = Transformer.from_crs(OUTPUT_EPSG, INPUT_EPSG, always_xy=True)

    waypoints = []
    for x, y in path.coords:
        lon, lat = to_gps.transform(x, y)
        waypoints.append({"lat": lat, "lon": lon, "alt": altitude})

    return waypoints


def generate_waypoints(polygon_coords, width=1.2, angle=-45, altitude=10.0):
    """
    Public entry point for teammates.

    Parameters
    ----------
    polygon_coords : list of (lon, lat) tuples
    width          : strip width in metres
    angle          : scan rotation in degrees
    altitude       : waypoint altitude AGL in metres

    Returns
    -------
    list of {"lat": float, "lon": float, "alt": float}
    """
    poly = gps_to_polygon(polygon_coords)
    path, strips = lawnmower(poly, width, angle)
    waypoints = path_to_waypoints(path, altitude)
    return waypoints
