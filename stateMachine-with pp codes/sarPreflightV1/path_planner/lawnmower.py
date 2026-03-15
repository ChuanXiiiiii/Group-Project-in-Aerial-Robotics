"""Lawnmower planner adapted for state machine parameters."""

import math

from .common import (
    append_unique,
    build_local_frame,
    clip_scanline,
    entry_point_near_p3,
    ensure_ccw,
    inset_polygon_raw,
    polygon_to_xy,
    rotate_points,
    unrotate_point,
    xy_to_ll,
)


def _point_on_segment(p, a, b, tol=1e-7):
    px, py = p
    ax, ay = a
    bx, by = b
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    cross = abs(abx * apy - aby * apx)
    if cross > tol:
        return False
    dot = apx * abx + apy * aby
    if dot < -tol:
        return False
    ab2 = abx * abx + aby * aby
    if dot - ab2 > tol:
        return False
    return True


def _locate_on_boundary(poly, p, tol=1e-7):
    hits = []
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        if _point_on_segment(p, a, b, tol=tol):
            ax, ay = a
            bx, by = b
            den = (bx - ax) * (bx - ax) + (by - ay) * (by - ay)
            if den < 1e-12:
                continue
            t = ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / den
            t = max(0.0, min(1.0, t))
            hits.append((i, t))
    return hits


def _forward_boundary_path(poly, start, end, e_start, t_start, e_end, t_end):
    n = len(poly)
    pts = [start]
    if e_start == e_end and t_start <= t_end:
        pts.append(end)
        return pts

    i = e_start
    pts.append(poly[(i + 1) % n])
    i = (i + 1) % n
    while i != e_end:
        pts.append(poly[(i + 1) % n])
        i = (i + 1) % n
    pts.append(end)
    return pts


def _backward_boundary_path(poly, start, end, e_start, t_start, e_end, t_end):
    n = len(poly)
    pts = [start]
    if e_start == e_end and t_start >= t_end:
        pts.append(end)
        return pts

    i = e_start
    pts.append(poly[i])
    i = (i - 1) % n
    while i != e_end:
        pts.append(poly[i])
        i = (i - 1) % n
    pts.append(end)
    return pts


def _polyline_length(pts):
    if len(pts) < 2:
        return 0.0
    s = 0.0
    for i in range(1, len(pts)):
        s += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
    return s


def _boundary_path(poly, start, end):
    loc_s = _locate_on_boundary(poly, start)
    loc_e = _locate_on_boundary(poly, end)
    if not loc_s or not loc_e:
        return [start, end]

    best_path = None
    best_len = float("inf")
    for e_start, t_start in loc_s:
        for e_end, t_end in loc_e:
            p_f = _forward_boundary_path(poly, start, end, e_start, t_start, e_end, t_end)
            p_b = _backward_boundary_path(poly, start, end, e_start, t_start, e_end, t_end)
            lf = _polyline_length(p_f)
            lb = _polyline_length(p_b)
            cand = p_f if lf <= lb else p_b
            clen = lf if lf <= lb else lb
            if clen < best_len:
                best_len = clen
                best_path = cand

    return best_path if best_path is not None else [start, end]


def generate_path(polygon, spacing_m, alt_m, heading_deg=0.0, enter_offset_m=10.0):
    """Generate lawnmower waypoints.

    Inputs match state machine parameters.
    """
    if len(polygon) < 3:
        raise ValueError("Polygon needs at least 3 vertices")
    if spacing_m <= 0:
        raise ValueError("spacing_m must be > 0")

    center_lat, center_lon, dlat_m, dlon_m = build_local_frame(polygon)
    poly_xy_raw = polygon_to_xy(polygon, center_lat, center_lon, dlat_m, dlon_m)
    poly_xy = ensure_ccw(poly_xy_raw)

    half_swath = spacing_m / 2.0
    inset_verts, _ = inset_polygon_raw(poly_xy, half_swath)
    if inset_verts is None:
        raise ValueError("Inset polygon collapsed; spacing too large for this area")

    # Enter from east of P3 using original P0..P4 ordering when available.
    enter_xy = entry_point_near_p3(poly_xy_raw, enter_offset_m)

    poly_rot = rotate_points(inset_verts, -heading_deg)
    ys = [p[1] for p in poly_rot]
    y_min = min(ys)
    y_max = max(ys)

    y = y_max
    scan_segments = []
    while y >= y_min - 1e-9:
        xs = clip_scanline(poly_rot, y)
        for k in range(0, len(xs) - 1, 2):
            scan_segments.append((xs[k], y, xs[k + 1], y))
        y -= spacing_m

    if not scan_segments:
        raise ValueError("No scan lines generated for given polygon and spacing")

    ordered_lines = []
    for i, (x1, y1, x2, y2) in enumerate(scan_segments):
        if i % 2 == 0:
            ordered_lines.append(((x2, y2), (x1, y1)))
        else:
            ordered_lines.append(((x1, y1), (x2, y2)))

    waypoints_rot = []
    s0, e0 = ordered_lines[0]
    append_unique(waypoints_rot, s0)
    append_unique(waypoints_rot, e0)
    prev_end = e0

    for i in range(1, len(ordered_lines)):
        start_i, end_i = ordered_lines[i]
        trans = _boundary_path(poly_rot, prev_end, start_i)
        for p in trans[1:]:
            append_unique(waypoints_rot, p)
        append_unique(waypoints_rot, end_i)
        prev_end = end_i

    mission_xy = [enter_xy]
    for xr, yr in waypoints_rot:
        xm, ym = unrotate_point(xr, yr, -heading_deg)
        append_unique(mission_xy, (xm, ym))

    waypoints = []
    for x, y in mission_xy:
        lat, lon = xy_to_ll(x, y, center_lat, center_lon, dlat_m, dlon_m)
        waypoints.append({"lat": float(lat), "lon": float(lon), "alt": float(alt_m)})

    return waypoints
