"""
pattern_generator.py — Multiple search patterns from a polygon.
================================================================
Pure geometry.  No MAVLink, no drone, no network.

Public API
----------
    generate_pattern(polygon, spacing_m, alt_m, heading_deg=0, mode=0)
        → list of {"lat": float, "lon": float, "alt": float}

    Mode codes
    ----------
    0 : Lawnmower      — classic boustrophedon parallel scan lines
    1 : Inward Spiral  — Archimedean spiral shrinking toward centroid
    2 : Contracting Polygon Orbit — polygon-shaped rings shrinking inward ring by ring

    generate_lawnmower(polygon, spacing_m, alt_m, heading_deg=0)
        → kept for backward compatibility, identical to mode=0

    PATTERN_NAMES : dict
        Human-readable name for each mode code (useful for logging/GUI).
"""

import math

# ── Earth constants ──────────────────────────────────────────

_EARTH_R = 6_371_000.0                      # mean radius in metres
_DEG_LAT_M = _EARTH_R * math.pi / 180.0     # metres per degree latitude


def _deg_lon_m(lat_deg):
    """Metres per degree longitude at a given latitude."""
    return _DEG_LAT_M * math.cos(math.radians(lat_deg))


# ── Coordinate helpers ───────────────────────────────────────

def _poly_to_metres(polygon):
    """
    Convert a list of (lat, lon) to local metres (x, y).
    Origin is set at the polygon centroid.
    Returns (poly_m, c_lat, c_lon, m_per_lon).
    """
    c_lat = sum(p[0] for p in polygon) / len(polygon)
    c_lon = sum(p[1] for p in polygon) / len(polygon)
    m_per_lon = _deg_lon_m(c_lat)

    poly_m = []
    for lat, lon in polygon:
        x = (lon - c_lon) * m_per_lon
        y = (lat - c_lat) * _DEG_LAT_M
        poly_m.append((x, y))

    return poly_m, c_lat, c_lon, m_per_lon


def _metres_to_latlon(xm, ym, c_lat, c_lon, m_per_lon, alt_m):
    """Convert local metres back to a lat/lon waypoint dict."""
    lat = c_lat + ym / _DEG_LAT_M
    lon = c_lon + xm / m_per_lon
    return {"lat": lat, "lon": lon, "alt": alt_m}


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


def _point_in_polygon(px, py, polygon_xy):
    """Ray-casting test: is point (px, py) inside polygon_xy?"""
    n = len(polygon_xy)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon_xy[i]
        xj, yj = polygon_xy[j]
        if ((yi > py) != (yj > py)) and \
                (px < (xj - xi) * (py - yi) / (yj - yi + 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def _polygon_apothem(poly_m, cx, cy):
    """
    Approximate inradius: minimum distance from (cx, cy) to any edge.
    Used as the max radius for the spiral / expanding-square patterns.
    """
    n = len(poly_m)
    min_d = float("inf")
    for i in range(n):
        x1, y1 = poly_m[i]
        x2, y2 = poly_m[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        if dx == dy == 0:
            d = math.hypot(cx - x1, cy - y1)
        else:
            t = max(0.0, min(1.0,
                             ((cx - x1) * dx + (cy - y1) * dy)
                             / (dx * dx + dy * dy)))
            d = math.hypot(cx - (x1 + t * dx), cy - (y1 + t * dy))
        min_d = min(min_d, d)
    return min_d


# ══════════════════════════════════════════════════════════════
#  Algorithm 0 — Lawnmower (boustrophedon)
# ══════════════════════════════════════════════════════════════

def _generate_lawnmower_m(poly_m, spacing_m, heading_deg=0):
    """
    Core lawnmower logic in local metres.
    Returns list of (x, y) waypoints in metres.
    """
    poly_rot = _rotate(poly_m, -heading_deg, 0, 0)
    ys = [p[1] for p in poly_rot]
    y_min, y_max = min(ys), max(ys)

    # Offset first line by half-spacing so the pattern is centred
    y = y_min + spacing_m / 2.0
    scan_segments = []
    while y < y_max:
        xs = _clip_scanline(poly_rot, y)
        for k in range(0, len(xs) - 1, 2):
            scan_segments.append((xs[k], y, xs[k + 1], y))
        y += spacing_m

    if not scan_segments:
        raise ValueError("No scan lines generated — polygon may be too "
                         "small for the given spacing.")

    # Connect in serpentine (alternating direction) order
    waypoints_rot = []
    for i, (x1, y1, x2, y2) in enumerate(scan_segments):
        if i % 2 == 0:
            waypoints_rot += [(x1, y1), (x2, y2)]
        else:
            waypoints_rot += [(x2, y2), (x1, y1)]

    # Rotate back to original orientation
    waypoints_m = []
    for xr, yr in waypoints_rot:
        xm, ym = _unrotate(xr, yr, -heading_deg, 0, 0)
        waypoints_m.append((xm, ym))

    return waypoints_m


# ══════════════════════════════════════════════════════════════
#  Algorithm 1 — Inward Spiral
# ══════════════════════════════════════════════════════════════

def _generate_spiral_m(poly_m, spacing_m):
    """
    Inward Archimedean spiral clipped to the polygon.

    Strategy: sample a dense Archimedean spiral (r decreases by
    spacing_m per full revolution) starting at max radius and
    spiralling inward.  Keep only points inside the polygon.
    Works well for convex and mildly concave polygons.

    Returns list of (x, y) in metres.
    """
    cx = sum(p[0] for p in poly_m) / len(poly_m)
    cy = sum(p[1] for p in poly_m) / len(poly_m)
    max_r = _polygon_apothem(poly_m, cx, cy)

    # Angular step sized so arc-length ≈ spacing_m / 4 (smooth sampling)
    arc_step = spacing_m / 4.0

    waypoints_m = []
    theta = 0.0

    while True:
        r = max_r - (spacing_m / (2 * math.pi)) * theta
        if r <= spacing_m / 2.0:
            break

        px = cx + r * math.cos(theta)
        py = cy + r * math.sin(theta)

        if _point_in_polygon(px, py, poly_m):
            waypoints_m.append((px, py))

        theta += arc_step / r   # adaptive step keeps arc-length constant

    if not waypoints_m:
        raise ValueError("Spiral generated no waypoints — polygon may be "
                         "too small for the given spacing.")
    return waypoints_m


# ══════════════════════════════════════════════════════════════
#  Algorithm 2 — Contracting Polygon Orbit
# ══════════════════════════════════════════════════════════════

def _offset_polygon_inward(poly_m, offset_m):
    """
    Shrink a convex polygon inward by offset_m metres.

    For each edge, moves it inward (toward the centroid) by offset_m,
    then recomputes corners as intersections of adjacent offset edges.

    Returns the new vertex list in the same order as the input,
    or None if the polygon has collapsed (parallel edges or zero area).
    """
    n = len(poly_m)
    cx = sum(p[0] for p in poly_m) / n
    cy = sum(p[1] for p in poly_m) / n

    # Build one offset line per edge: (base_x, base_y, dir_x, dir_y)
    offset_lines = []
    for i in range(n):
        x1, y1 = poly_m[i]
        x2, y2 = poly_m[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 1e-10:
            continue
        # Right-perpendicular unit normal: (dy/L, -dx/L)
        nx, ny = dy / length, -dx / length
        # Flip if the normal points away from the centroid
        mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
        if (cx - mid_x) * nx + (cy - mid_y) * ny < 0:
            nx, ny = -nx, -ny
        offset_lines.append((x1 + nx * offset_m, y1 + ny * offset_m, dx, dy))

    if len(offset_lines) < 3:
        return None

    # New corners = intersections of adjacent offset lines
    new_poly = []
    m = len(offset_lines)
    for i in range(m):
        j = (i + 1) % m
        ox1, oy1, dx1, dy1 = offset_lines[i]
        ox2, oy2, dx2, dy2 = offset_lines[j]
        denom = dx1 * dy2 - dy1 * dx2
        if abs(denom) < 1e-10:
            return None  # parallel edges — polygon has collapsed
        t = ((ox2 - ox1) * dy2 - (oy2 - oy1) * dx2) / denom
        new_poly.append((ox1 + t * dx1, oy1 + t * dy1))

    # Confirm the shrunken polygon is still viable
    ncx = sum(p[0] for p in new_poly) / len(new_poly)
    ncy = sum(p[1] for p in new_poly) / len(new_poly)
    if _polygon_apothem(new_poly, ncx, ncy) <= 0:
        return None

    return new_poly


def _generate_contracting_polygon_m(poly_m, spacing_m):
    """
    Contracting Polygon Orbit.

    Orbits the search-area boundary ring by ring, shrinking inward by
    spacing_m each orbit.  The polygon shape (not a square) is used
    directly: each ring contributes exactly N waypoints — one per
    vertex — which are the only turning points for the drone.
    For a quadrilateral search area that is exactly 4 waypoints per ring.

    Ring 0  : the original polygon corners  (outermost orbit)
    Ring 1  : polygon shrunk inward by spacing_m
    Ring 2  : shrunk by 2 × spacing_m
    …
    The pattern terminates when the offset polygon collapses.

    Returns list of (x, y) in metres.
    """
    waypoints_m = []
    current_poly = list(poly_m)

    while True:
        cx = sum(p[0] for p in current_poly) / len(current_poly)
        cy = sum(p[1] for p in current_poly) / len(current_poly)
        if _polygon_apothem(current_poly, cx, cy) <= 0:
            break

        # One waypoint per corner — drone turns here, no intermediate points
        waypoints_m.extend(current_poly)

        # Shrink inward for the next ring
        next_poly = _offset_polygon_inward(current_poly, spacing_m)
        if next_poly is None:
            break
        current_poly = next_poly

    if not waypoints_m:
        raise ValueError(
            "Contracting polygon orbit generated no waypoints — "
            "polygon may be too small for the given spacing.")
    return waypoints_m


# ══════════════════════════════════════════════════════════════
#  Dispatcher — Public API
# ══════════════════════════════════════════════════════════════

#: Human-readable names for each mode code (use in logs / GUI)
PATTERN_NAMES = {
    0: "Lawnmower (boustrophedon)",
    1: "Inward Spiral",
    2: "Contracting Polygon Orbit",
}


def generate_pattern(polygon, spacing_m, alt_m, heading_deg=0, mode=0):
    """
    Generate a search pattern inside a polygon.

    Parameters
    ----------
    polygon : list of (lat, lon)
        Vertices of the search area.  Must have ≥ 3 points.
        Do NOT include a duplicate closing vertex.
    spacing_m : float
        Distance between scan lines / spiral arms in metres.
    alt_m : float
        Altitude for every waypoint (metres AGL).
    heading_deg : float
        Scan-line orientation — only used for mode 0 (Lawnmower).
        0 = East–West lines, 90 = North–South lines.
    mode : int
        0 → Lawnmower (boustrophedon)
        1 → Inward Spiral
        2 → Contracting Polygon Orbit

    Returns
    -------
    list of dict
        [{"lat": float, "lon": float, "alt": float}, ...]
    """
    if len(polygon) < 3:
        raise ValueError(f"Polygon needs ≥ 3 vertices, got {len(polygon)}")
    if mode not in PATTERN_NAMES:
        raise ValueError(
            f"Unknown mode {mode!r}.  Valid modes: {list(PATTERN_NAMES)}")

    poly_m, c_lat, c_lon, m_per_lon = _poly_to_metres(polygon)

    if mode == 0:
        wps_m = _generate_lawnmower_m(poly_m, spacing_m, heading_deg)
    elif mode == 1:
        wps_m = _generate_spiral_m(poly_m, spacing_m)
    elif mode == 2:
        wps_m = _generate_contracting_polygon_m(poly_m, spacing_m)

    return [_metres_to_latlon(x, y, c_lat, c_lon, m_per_lon, alt_m)
            for x, y in wps_m]


def generate_lawnmower(polygon, spacing_m, alt_m, heading_deg=0):
    """
    Backward-compatible wrapper — identical to generate_pattern(..., mode=0).
    Existing code that calls generate_lawnmower() continues to work unchanged.
    """
    return generate_pattern(polygon, spacing_m, alt_m,
                             heading_deg=heading_deg, mode=0)
