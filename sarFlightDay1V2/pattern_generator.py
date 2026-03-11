"""
pattern_generator.py — Lawnmower search pattern from a polygon.
================================================================
Pure geometry.  No MAVLink, no drone, no network.

Public API
----------
    generate_lawnmower(polygon, spacing_m, alt_m, heading_deg=0)
        → list of {"lat": float, "lon": float, "alt": float}

Your CV teammates can replace the internals (swap spacing for
camera-derived swath width, add overlap, etc.) as long as the
return signature stays the same.
"""

import math

# ── Earth constants ──────────────────────────────────────────

_EARTH_R = 6_371_000.0                      # mean radius in metres
_DEG_LAT_M = _EARTH_R * math.pi / 180.0     # metres per degree latitude


def _deg_lon_m(lat_deg):
    """Metres per degree longitude at a given latitude."""
    return _DEG_LAT_M * math.cos(math.radians(lat_deg))


# ── Polygon / line helpers ───────────────────────────────────

def _segment_intersect_y(p1, p2, y):
    """
    Intersection x-coordinate of the segment p1→p2 with horizontal
    line y.  Returns None if no intersection within the segment.
    p1, p2 are (x, y) tuples.
    """
    (x1, y1), (x2, y2) = p1, p2
    if (y1 - y) * (y2 - y) > 0:          # both on same side
        return None
    if abs(y2 - y1) < 1e-14:             # horizontal segment
        return None
    t = (y - y1) / (y2 - y1)
    if t < 0.0 or t > 1.0:
        return None
    return x1 + t * (x2 - x1)


def _clip_scanline(polygon_xy, y):
    """
    Find all x-intersections of horizontal line y with the polygon
    edges.  Returns sorted list of x values (should be even count
    for a simple polygon).
    """
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
    """Rotate a list of (x, y) points around (cx, cy)."""
    a = math.radians(angle_deg)
    cos_a, sin_a = math.cos(a), math.sin(a)
    out = []
    for x, y in points:
        dx, dy = x - cx, y - cy
        out.append((cx + dx * cos_a - dy * sin_a,
                     cy + dx * sin_a + dy * cos_a))
    return out


def _unrotate(x, y, angle_deg, cx, cy):
    """Inverse of _rotate for a single point."""
    return _rotate([(x, y)], -angle_deg, cx, cy)[0]


# ── Public API ───────────────────────────────────────────────

def generate_square_box(centre_lat, centre_lon, side_m, alt_m):
    """
    Generate a simple 30×30 m square box pattern for first flight day.

    The drone flies the four corners of the square in order (NW → NE →
    SE → SW), returning to the start corner to close the loop.  This
    gives five waypoints total (corner 1 repeated at the end so ArduPilot
    cleanly terminates the mission).

    Parameters
    ----------
    centre_lat : float   Latitude of the centre of the box
    centre_lon : float   Longitude of the centre of the box
    side_m     : float   Side length in metres (use 30 for first flight day)
    alt_m      : float   Altitude AGL for all waypoints

    Returns
    -------
    list of dict  [{"lat": float, "lon": float, "alt": float}, ...]
    """
    half = side_m / 2.0
    m_per_lon = _deg_lon_m(centre_lat)

    dlat = half / _DEG_LAT_M
    dlon = half / m_per_lon

    # NW → NE → SE → SW → NW (close loop)
    corners = [
        (centre_lat + dlat, centre_lon - dlon),   # NW
        (centre_lat + dlat, centre_lon + dlon),   # NE
        (centre_lat - dlat, centre_lon + dlon),   # SE
        (centre_lat - dlat, centre_lon - dlon),   # SW
        (centre_lat + dlat, centre_lon - dlon),   # NW again — closes box
    ]

    return [{"lat": lat, "lon": lon, "alt": alt_m} for lat, lon in corners]


def generate_lawnmower(polygon, spacing_m, alt_m, heading_deg=0):
    """
    Generate a lawnmower (boustrophedon) pattern inside a polygon.

    Parameters
    ----------
    polygon : list of (lat, lon)
        Vertices of the search area.  Must have ≥ 3 points.
        Do NOT include a duplicate closing vertex.
    spacing_m : float
        Distance between parallel scan lines in metres.
    alt_m : float
        Altitude for every waypoint (metres AGL).
    heading_deg : float
        Orientation of scan lines.  0 = lines run East–West,
        90 = lines run North–South.

    Returns
    -------
    list of dict
        [{"lat": float, "lon": float, "alt": float}, ...]
        Ordered for serpentine (alternating-direction) flight.
    """
    if len(polygon) < 3:
        raise ValueError(f"Polygon needs ≥ 3 vertices, got {len(polygon)}")

    # ── Convert polygon to local metres (origin at centroid) ──
    c_lat = sum(p[0] for p in polygon) / len(polygon)
    c_lon = sum(p[1] for p in polygon) / len(polygon)
    m_per_lon = _deg_lon_m(c_lat)

    poly_m = []
    for lat, lon in polygon:
        x = (lon - c_lon) * m_per_lon
        y = (lat - c_lat) * _DEG_LAT_M
        poly_m.append((x, y))

    # ── Rotate polygon so scan lines are horizontal ──
    poly_rot = _rotate(poly_m, -heading_deg,  0, 0)

    # ── Bounding box of rotated polygon ──
    ys = [p[1] for p in poly_rot]
    y_min, y_max = min(ys), max(ys)

    # ── Generate scan lines ──
    # Offset first line by half-spacing so pattern is centred
    y = y_min + spacing_m / 2.0
    scan_segments = []
    while y < y_max:
        xs = _clip_scanline(poly_rot, y)
        # Take pairs of intersections as entry/exit points
        for k in range(0, len(xs) - 1, 2):
            scan_segments.append((xs[k], y, xs[k + 1], y))
        y += spacing_m

    if not scan_segments:
        raise ValueError("No scan lines generated — polygon may be too "
                         "small for the given spacing")

    # ── Connect in serpentine order ──
    waypoints_m = []
    for i, (x1, y1, x2, y2) in enumerate(scan_segments):
        if i % 2 == 0:
            waypoints_m.append((x1, y1))
            waypoints_m.append((x2, y2))
        else:
            waypoints_m.append((x2, y2))
            waypoints_m.append((x1, y1))

    # ── Rotate back and convert to lat/lon ──
    waypoints = []
    for xr, yr in waypoints_m:
        xm, ym = _unrotate(xr, yr, -heading_deg, 0, 0)
        lat = c_lat + ym / _DEG_LAT_M
        lon = c_lon + xm / m_per_lon
        waypoints.append({"lat": lat, "lon": lon, "alt": alt_m})

    return waypoints
