"""Shared geometry helpers for path planning algorithms."""

import math

EARTH_R_M = 6_371_000.0
DEG_LAT_M = EARTH_R_M * math.pi / 180.0


def deg_lon_m(lat_deg):
    """Meters per degree longitude at latitude."""
    return DEG_LAT_M * math.cos(math.radians(lat_deg))


def ll_to_xy(lat, lon, origin_lat, origin_lon, dlat_m, dlon_m):
    """Convert lat/lon to local ENU-like x/y (meters)."""
    x = (lon - origin_lon) * dlon_m
    y = (lat - origin_lat) * dlat_m
    return x, y


def xy_to_ll(x, y, origin_lat, origin_lon, dlat_m, dlon_m):
    """Convert local x/y (meters) back to lat/lon."""
    lat = origin_lat + y / dlat_m
    lon = origin_lon + x / dlon_m
    return lat, lon


def _polygon_area2(poly_xy):
    """Twice signed area. Positive means CCW."""
    n = len(poly_xy)
    s = 0.0
    for i in range(n):
        j = (i + 1) % n
        s += poly_xy[i][0] * poly_xy[j][1] - poly_xy[j][0] * poly_xy[i][1]
    return s


def ensure_ccw(poly_xy):
    """Return polygon in CCW order."""
    if _polygon_area2(poly_xy) < 0:
        return list(reversed(poly_xy))
    return list(poly_xy)


def build_local_frame(polygon):
    """Build centroid-based local frame from polygon (lat, lon) list."""
    if len(polygon) < 3:
        raise ValueError("Polygon needs at least 3 vertices")

    lat_mean = sum(p[0] for p in polygon) / len(polygon)
    lon_mean = sum(p[1] for p in polygon) / len(polygon)
    dlat_m = DEG_LAT_M
    dlon_m = deg_lon_m(lat_mean)

    xs = []
    ys = []
    for lat, lon in polygon:
        x = (lon - lon_mean) * dlon_m
        y = (lat - lat_mean) * dlat_m
        xs.append(x)
        ys.append(y)

    area2 = 0.0
    cx = 0.0
    cy = 0.0
    n = len(polygon)
    for i in range(n):
        j = (i + 1) % n
        cross = xs[i] * ys[j] - xs[j] * ys[i]
        area2 += cross
        cx += (xs[i] + xs[j]) * cross
        cy += (ys[i] + ys[j]) * cross

    if abs(area2) < 1e-12:
        center_x = sum(xs) / n
        center_y = sum(ys) / n
    else:
        center_x = cx / (3.0 * area2)
        center_y = cy / (3.0 * area2)

    center_lat = lat_mean + center_y / dlat_m
    center_lon = lon_mean + center_x / dlon_m

    dlon_center_m = deg_lon_m(center_lat)
    return center_lat, center_lon, dlat_m, dlon_center_m


def polygon_to_xy(polygon, center_lat, center_lon, dlat_m, dlon_m):
    """Convert polygon (lat, lon) to local x/y coordinates."""
    return [ll_to_xy(lat, lon, center_lat, center_lon, dlat_m, dlon_m) for lat, lon in polygon]


def rotate_points(points, angle_deg):
    """Rotate list of points around origin by angle in degrees."""
    a = math.radians(angle_deg)
    ca = math.cos(a)
    sa = math.sin(a)
    return [(x * ca - y * sa, x * sa + y * ca) for x, y in points]


def unrotate_point(x, y, angle_deg):
    """Inverse rotate point around origin."""
    a = math.radians(-angle_deg)
    ca = math.cos(a)
    sa = math.sin(a)
    return x * ca - y * sa, x * sa + y * ca


def segment_intersect_y(p1, p2, y):
    """Intersection x of segment p1->p2 with horizontal line y."""
    (x1, y1), (x2, y2) = p1, p2
    if (y1 - y) * (y2 - y) > 0:
        return None
    if abs(y2 - y1) < 1e-14:
        return None
    t = (y - y1) / (y2 - y1)
    if t < 0.0 or t > 1.0:
        return None
    return x1 + t * (x2 - x1)


def clip_scanline(poly_xy, y):
    """Return sorted x-intersections between polygon and scanline y."""
    xs = []
    n = len(poly_xy)
    for i in range(n):
        j = (i + 1) % n
        x = segment_intersect_y(poly_xy[i], poly_xy[j], y)
        if x is not None:
            xs.append(x)
    xs.sort()
    return xs


def line_intersection(p1, d1, p2, d2):
    """Intersect lines p1+t*d1 and p2+s*d2."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < 1e-12:
        return None
    t = (dx * d2[1] - dy * d2[0]) / cross
    return p1[0] + t * d1[0], p1[1] + t * d1[1]


def offset_line_ccw(poly_xy, edge_idx, offset_m):
    """Inset line for a CCW polygon edge."""
    n = len(poly_xy)
    ax, ay = poly_xy[edge_idx]
    bx, by = poly_xy[(edge_idx + 1) % n]
    ex = bx - ax
    ey = by - ay
    elen = math.hypot(ex, ey)
    if elen < 1e-12:
        return None
    ex /= elen
    ey /= elen
    nx = -ey
    ny = ex
    return (ax + nx * offset_m, ay + ny * offset_m), (ex, ey)


def inset_polygon_raw(poly_xy, offset_m):
    """Inset polygon by offset; return (vertices, area2) or (None, 0)."""
    n = len(poly_xy)
    lines = []
    for i in range(n):
        line = offset_line_ccw(poly_xy, i, offset_m)
        if line is None:
            return None, 0.0
        lines.append(line)

    verts = []
    for i in range(n):
        pt = line_intersection(lines[(i - 1) % n][0], lines[(i - 1) % n][1], lines[i][0], lines[i][1])
        if pt is None:
            return None, 0.0
        verts.append(pt)

    area2 = _polygon_area2(verts)
    if abs(area2) < 1e-9:
        return None, 0.0
    return verts, area2


def reorder_ring(points, start_idx):
    """Return cyclically shifted ring points[start_idx:] + points[:start_idx]."""
    return points[start_idx:] + points[:start_idx]


def nearest_index(points, ref_point):
    """Index of point closest to ref_point."""
    best_i = 0
    best_d = float("inf")
    for i, p in enumerate(points):
        d = math.hypot(p[0] - ref_point[0], p[1] - ref_point[1])
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def append_unique(points, p, tol=1e-6):
    """Append p unless it duplicates the previous point."""
    if not points:
        points.append(p)
        return
    if math.hypot(points[-1][0] - p[0], points[-1][1] - p[1]) > tol:
        points.append(p)


def corner_map_p0_p4_latlon(polygon):
    """Return {'P0': ..., 'P1': ..., ...} when polygon has 5 vertices."""
    if len(polygon) != 5:
        return {}
    return {f"P{i}": polygon[i] for i in range(5)}


def corner_map_p0_p4_xy(poly_xy):
    """Return {'P0': ..., 'P1': ..., ...} for a 5-vertex local polygon."""
    if len(poly_xy) != 5:
        return {}
    return {f"P{i}": poly_xy[i] for i in range(5)}


def entry_point_near_p3(poly_xy, enter_offset_m=10.0):
    """Entry point east of P3 when P0..P4 exist; fallback to northernmost."""
    if len(poly_xy) == 5:
        p3_x, p3_y = poly_xy[3]
        return p3_x + enter_offset_m, p3_y

    north = max(poly_xy, key=lambda p: (p[1], p[0]))
    return north[0] + enter_offset_m, north[1]
