"""
pattern_generator.py — Search pattern generators.
===================================================
Pure geometry.  No MAVLink, no drone, no network, no KML reading.
KML parsing is handled upstream by kml_parser.py.

Public API
----------
All three generators share the same signature and return format:

    generate_lawnmower(polygon, spacing_m, alt_m, heading_deg=0)
    generate_lawnmower_greedy(polygon, spacing_m, alt_m, heading_deg=0)
    generate_square_box(centre_lat, centre_lon, side_m, alt_m)

    polygon     : list of (lat, lon) tuples  — ≥ 3 points, no closing repeat
    spacing_m   : strip width in metres
    alt_m       : altitude AGL for every waypoint
    heading_deg : scan line orientation (0 = E–W, 90 = N–S)

    Returns: list of {"lat": float, "lon": float, "alt": float}

generate_lawnmower        — Carter: scanline-clip boustrophedon
generate_lawnmower_greedy — Teammate: shapely strip + greedy nearest-end ordering
generate_square_box       — Flight day 1 box pattern (does not use polygon arg)
"""

import math

import numpy as np
import shapely
import shapely.affinity
import shapely.ops
from pyproj import Transformer
from shapely.geometry import LineString, Point, Polygon


# ── Projection constants ─────────────────────────────────────

INPUT_EPSG  = "EPSG:4326"   # GPS lat/lon
OUTPUT_EPSG = "EPSG:32630"  # Bristol / UK — UTM Zone 30N

_EARTH_R      = 6_371_000.0
_DEG_LAT_M    = _EARTH_R * math.pi / 180.0


def _deg_lon_m(lat_deg):
    """Metres per degree longitude at a given latitude."""
    return _DEG_LAT_M * math.cos(math.radians(lat_deg))


# ══════════════════════════════════════════════════════════════
# Algorithm 1 — Scanline boustrophedon  (Carter)
# ══════════════════════════════════════════════════════════════

def _segment_intersect_y(p1, p2, y):
    (x1, y1), (x2, y2) = p1, p2
    if (y1 - y) * (y2 - y) > 0:
        return None
    if abs(y2 - y1) < 1e-14:
        return None
    t = (y - y1) / (y2 - y1)
    if t < 0.0 or t > 1.0:
        return None
    return x1 + t * (x2 - x1)


def _clip_scanline(polygon_xy, y):
    xs = []
    n = len(polygon_xy)
    for i in range(n):
        j = (i + 1) % n
        x = _segment_intersect_y(polygon_xy[i], polygon_xy[j], y)
        if x is not None:
            xs.append(x)
    xs.sort()
    return xs


def _rotate(points, angle_deg, cx, cy):
    a = math.radians(angle_deg)
    cos_a, sin_a = math.cos(a), math.sin(a)
    out = []
    for x, y in points:
        dx, dy = x - cx, y - cy
        out.append((cx + dx * cos_a - dy * sin_a,
                     cy + dx * sin_a + dy * cos_a))
    return out


def _unrotate(x, y, angle_deg, cx, cy):
    return _rotate([(x, y)], -angle_deg, cx, cy)[0]


def generate_lawnmower(polygon, spacing_m, alt_m, heading_deg=0):
    """
    Scanline-clip boustrophedon pattern (Carter).

    Converts polygon to local metre coordinates, rotates to align
    scan lines, clips horizontal scanlines against the polygon edges,
    then reconnects in serpentine order.
    """
    if len(polygon) < 3:
        raise ValueError(f"Polygon needs ≥ 3 vertices, got {len(polygon)}")

    c_lat = sum(p[0] for p in polygon) / len(polygon)
    c_lon = sum(p[1] for p in polygon) / len(polygon)
    m_per_lon = _deg_lon_m(c_lat)

    poly_m = [
        ((lon - c_lon) * m_per_lon, (lat - c_lat) * _DEG_LAT_M)
        for lat, lon in polygon
    ]

    poly_rot = _rotate(poly_m, -heading_deg, 0, 0)

    ys = [p[1] for p in poly_rot]
    y_min, y_max = min(ys), max(ys)

    y = y_min + spacing_m / 2.0
    scan_segments = []
    while y < y_max:
        xs = _clip_scanline(poly_rot, y)
        for k in range(0, len(xs) - 1, 2):
            scan_segments.append((xs[k], y, xs[k + 1], y))
        y += spacing_m

    if not scan_segments:
        raise ValueError(
            "No scan lines generated — polygon may be too small for the given spacing"
        )

    waypoints_rot = []
    for i, (x1, y1, x2, y2) in enumerate(scan_segments):
        if i % 2 == 0:
            waypoints_rot += [(x1, y1), (x2, y2)]
        else:
            waypoints_rot += [(x2, y2), (x1, y1)]

    waypoints = []
    for xr, yr in waypoints_rot:
        xm, ym = _unrotate(xr, yr, -heading_deg, 0, 0)
        lat = c_lat + ym / _DEG_LAT_M
        lon = c_lon + xm / m_per_lon
        waypoints.append({"lat": lat, "lon": lon, "alt": float(alt_m)})

    return waypoints


# ══════════════════════════════════════════════════════════════
# Algorithm 2 — Shapely strip + greedy ordering  (teammate)
# ══════════════════════════════════════════════════════════════

def _to_meters(polygon):
    """Project (lat, lon) polygon to UTM Zone 30N metre coordinates."""
    transformer = Transformer.from_crs(INPUT_EPSG, OUTPUT_EPSG, always_xy=True)
    return [transformer.transform(lon, lat) for lat, lon in polygon]


def _to_gps_transformer():
    return Transformer.from_crs(OUTPUT_EPSG, INPUT_EPSG, always_xy=True)


def _split_poly(poly, width):
    x_min, y_min, x_max, y_max = poly.bounds
    y_splits = np.arange(y_min + width, y_max, width)
    res = poly
    for y in y_splits:
        split_line = shapely.LineString([(x_min, y), (x_max, y)])
        res = shapely.MultiPolygon(shapely.ops.split(res, split_line))
    return res


def _make_strips(splits):
    return shapely.MultiPolygon([g.envelope for g in splits.geoms])


def _build_path_greedy(strips):
    """
    Build a path through strips using nearest-end greedy ordering.
    At each step, pick the unvisited strip whose nearest endpoint is
    closest to the current position, then traverse to its far end.
    """
    pairs = []
    for s in strips.geoms:
        x_min, y_min, x_max, y_max = s.bounds
        mid_y = 0.5 * (y_min + y_max)
        pairs.append((Point(x_min, mid_y), Point(x_max, mid_y)))

    path_points = []
    current = Point(0, 0)

    while pairs:
        dists = [
            (current.distance(left), current.distance(right))
            for left, right in pairs
        ]
        next_strip = int(np.argmin([min(d) for d in dists]))
        near_end   = int(np.argmin(dists[next_strip]))

        path_points.append(pairs[next_strip][near_end])
        path_points.append(pairs[next_strip][1 - near_end])
        current = pairs[next_strip][1 - near_end]
        pairs.pop(next_strip)

    return LineString(path_points)


def generate_lawnmower_greedy(polygon, spacing_m, alt_m, heading_deg=0):
    """
    Shapely strip decomposition with greedy nearest-end ordering (teammate).

    Projects the polygon to UTM, splits it into horizontal strips of
    width spacing_m, then builds a path through strip midlines using a
    greedy nearest-endpoint heuristic.

    heading_deg is accepted for API compatibility but ignored — strip
    orientation is always axis-aligned in UTM space.
    """
    if len(polygon) < 3:
        raise ValueError(f"Polygon needs ≥ 3 vertices, got {len(polygon)}")

    meter_coords = _to_meters(polygon)
    poly_m       = Polygon(meter_coords)

    if not poly_m.is_valid:
        poly_m = poly_m.buffer(0)   # attempt repair

    splits  = _split_poly(poly_m, spacing_m)
    strips  = _make_strips(splits)
    path    = _build_path_greedy(strips)
    to_gps  = _to_gps_transformer()

    waypoints = []
    for x, y in path.coords:
        lon, lat = to_gps.transform(x, y)
        waypoints.append({"lat": lat, "lon": lon, "alt": float(alt_m)})

    return waypoints


# ══════════════════════════════════════════════════════════════
# Flight day 1 — Square box pattern
# ══════════════════════════════════════════════════════════════

def generate_square_box(centre_lat, centre_lon, side_m, alt_m):
    """
    Generate a square box pattern centred on (centre_lat, centre_lon).

    Produces 5 waypoints: NW → NE → SE → SW → NW (closing repeat)
    so the drone completes a full square before the mission ends.

    Returns list of {"lat", "lon", "alt"} dicts.
    """
    half      = side_m / 2.0
    m_per_lon = _deg_lon_m(centre_lat)
    d_lat     = half / _DEG_LAT_M
    d_lon     = half / m_per_lon

    corners = [
        (centre_lat + d_lat, centre_lon - d_lon),   # NW
        (centre_lat + d_lat, centre_lon + d_lon),   # NE
        (centre_lat - d_lat, centre_lon + d_lon),   # SE
        (centre_lat - d_lat, centre_lon - d_lon),   # SW
        (centre_lat + d_lat, centre_lon - d_lon),   # NW close
    ]

    return [{"lat": lat, "lon": lon, "alt": float(alt_m)} for lat, lon in corners]
