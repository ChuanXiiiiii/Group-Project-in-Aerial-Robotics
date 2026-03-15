"""
lawnmower_planner.py — Lawnmower (boustrophedon) path planner
              for the convex polygon search area (Survey Area).
================================================================

Algorithm  (Algorithm 0, verified on real hardware)
---------
The polygon is first inset by ``HALF_SWATH`` to form the survey boundary
(the camera centreline should stay at least ``HALF_SWATH`` inside the
perimeter).  The inset polygon is then rotated by ``-HEADING_DEG`` so
that scan lines run horizontally.  Scan lines are generated at
intervals of ``SWATH`` from the top edge of the inset polygon to its
bottom edge; each line is clipped against the inset polygon to find the
left and right endpoints.  Consecutive lines are joined in alternating
direction (serpentine / boustrophedon) to minimise transit distances.
The resulting points are rotated back and converted to (lat, lon).

Parameters
----------
    HALF_SWATH   : half the camera footprint width in metres (adjust freely).
    SWATH        : 2 × HALF_SWATH — the spacing between parallel scan lines.
    HEADING_DEG  : compass bearing (°) of the scan-line direction.
                   0 = E–W lines, 90 = N–S lines.
"""

from __future__ import annotations
import math
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import _DEG_LAT_M, deg_lon_m, haversine_m
from search_area import (
    CORNERS, N_VERTS, CENTER_LAT, CENTER_LON,
    _dlon_m, _dlat_m,
)
from perimeter_planner import _inset_polygon_raw

# ── Swath / heading parameters ─────────────────────────────────
HALF_SWATH  = 5.5          # metres — adjust to change line spacing
SWATH       = HALF_SWATH * 2 # full swath width = scan-line spacing (metres)
HEADING_DEG = 0.0            # 0 → E–W horizontal scan lines, sweeping top→bottom

# Enter point: 10 m east of P3 (UAV approaches from the right side)
ENTER_OFFSET_M = 10.0   # metres east of P3

# Take-Off Location  (kept for reference / plotting only)
TAKEOFF_LAT =  51.42340640206451
TAKEOFF_LON = -2.671446029622069

# ── Coordinate helpers ─────────────────────────────────────────
def _ll2xy(lat: float, lon: float) -> tuple[float, float]:
    """(lat, lon) → (x_east_m, y_north_m) relative to centroid."""
    return (lon - CENTER_LON) * _dlon_m, (lat - CENTER_LAT) * _dlat_m

def _xy2ll(x: float, y: float) -> tuple[float, float]:
    """(x_east_m, y_north_m) → (lat, lon)."""
    return CENTER_LAT + y / _dlat_m, CENTER_LON + x / _dlon_m

# Polygon vertices in local XY (centroid at origin)
_POLY_XY: list[tuple[float, float]] = [_ll2xy(lat, lon) for lat, lon in CORNERS]


# ── Geometry helpers (Algorithm 0 core — from pattern_generator.py) ───────

def _rotate(points: list[tuple[float, float]],
            angle_deg: float) -> list[tuple[float, float]]:
    """Rotate a list of (x, y) around the origin by angle_deg degrees."""
    a = math.radians(angle_deg)
    cos_a, sin_a = math.cos(a), math.sin(a)
    return [(x * cos_a - y * sin_a,
             x * sin_a + y * cos_a)
            for x, y in points]


def _unrotate(x: float, y: float, angle_deg: float) -> tuple[float, float]:
    """Inverse rotation of a single point around the origin."""
    a = math.radians(-angle_deg)
    cos_a, sin_a = math.cos(a), math.sin(a)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a


def _segment_intersect_y(p1: tuple[float, float],
                          p2: tuple[float, float],
                          y: float) -> float | None:
    """
    Intersection x-coordinate of segment p1→p2 with horizontal line y.
    Returns None if no intersection within the segment.
    """
    (x1, y1), (x2, y2) = p1, p2
    if (y1 - y) * (y2 - y) > 0:     # both vertices on same side
        return None
    if abs(y2 - y1) < 1e-14:         # horizontal segment
        return None
    t = (y - y1) / (y2 - y1)
    if t < 0.0 or t > 1.0:
        return None
    return x1 + t * (x2 - x1)


def _clip_scanline(polygon_xy: list[tuple[float, float]],
                   y: float) -> list[float]:
    """
    Find all x-intersections of horizontal line y with the polygon edges.
    Returns a sorted list of x values (even count for a simple polygon).
    """
    xs: list[float] = []
    n = len(polygon_xy)
    for i in range(n):
        j = (i + 1) % n
        x = _segment_intersect_y(polygon_xy[i], polygon_xy[j], y)
        if x is not None:
            xs.append(x)
    xs.sort()
    return xs


def _point_on_segment(p: tuple[float, float],
                      a: tuple[float, float],
                      b: tuple[float, float],
                      tol: float = 1e-7) -> bool:
    """Return True if point p lies on segment a→b (within tolerance)."""
    px, py = p
    ax, ay = a
    bx, by = b
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
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


def _locate_on_boundary(poly: list[tuple[float, float]],
                        p: tuple[float, float],
                        tol: float = 1e-7) -> list[tuple[int, float]]:
    """
    Locate point p on polygon boundary.
    Returns all matching (edge_idx, t) with p = v_i + t*(v_{i+1}-v_i), 0<=t<=1.
    """
    hits: list[tuple[int, float]] = []
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


def _forward_boundary_path(poly: list[tuple[float, float]],
                           start: tuple[float, float],
                           end: tuple[float, float],
                           e_start: int, t_start: float,
                           e_end: int, t_end: float) -> list[tuple[float, float]]:
    """Boundary path from start to end following increasing edge index."""
    n = len(poly)
    pts: list[tuple[float, float]] = [start]

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


def _backward_boundary_path(poly: list[tuple[float, float]],
                            start: tuple[float, float],
                            end: tuple[float, float],
                            e_start: int, t_start: float,
                            e_end: int, t_end: float) -> list[tuple[float, float]]:
    """Boundary path from start to end following decreasing edge index."""
    n = len(poly)
    pts: list[tuple[float, float]] = [start]

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


def _polyline_length(pts: list[tuple[float, float]]) -> float:
    """Total Euclidean length of a polyline."""
    if len(pts) < 2:
        return 0.0
    s = 0.0
    for i in range(1, len(pts)):
        s += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
    return s


def _boundary_path(poly: list[tuple[float, float]],
                   start: tuple[float, float],
                   end: tuple[float, float]) -> list[tuple[float, float]]:
    """
    Shorter boundary path between two boundary points on a convex polygon.
    Returns [start, ..., end].
    """
    loc_s = _locate_on_boundary(poly, start)
    loc_e = _locate_on_boundary(poly, end)
    if not loc_s or not loc_e:
        return [start, end]

    best_path: list[tuple[float, float]] | None = None
    best_len = float("inf")

    for e_start, t_start in loc_s:
        for e_end, t_end in loc_e:
            p_f = _forward_boundary_path(poly, start, end, e_start, t_start, e_end, t_end)
            p_b = _backward_boundary_path(poly, start, end, e_start, t_start, e_end, t_end)
            lf = _polyline_length(p_f)
            lb = _polyline_length(p_b)
            cand = p_f if lf <= lb else p_b
            lc = lf if lf <= lb else lb
            if lc < best_len:
                best_len = lc
                best_path = cand

    return best_path if best_path is not None else [start, end]


def _append_point_unique(pts: list[tuple[float, float]],
                         p: tuple[float, float],
                         tol: float = 1e-6) -> None:
    """Append point if it is not a near-duplicate of the previous point."""
    if not pts:
        pts.append(p)
        return
    x0, y0 = pts[-1]
    if math.hypot(p[0] - x0, p[1] - y0) > tol:
        pts.append(p)


# ── Public API ─────────────────────────────────────────────────

def plan(heading_deg: float = HEADING_DEG) -> list[dict]:
    """
    Generate lawnmower (boustrophedon) waypoints for the search area.

    The polygon boundary is first inset by ``HALF_SWATH`` so that scan
    lines never venture within ``HALF_SWATH`` of the original perimeter.
    Scan lines at spacing ``SWATH`` are clipped against this inset polygon,
    covering it from top edge to bottom edge.

    Parameters
    ----------
    heading_deg : float
        Scan-line orientation in degrees.
        0 → East–West lines (default), 90 → North–South lines.

    Returns
    -------
    list of dict
        Each dict has keys: ``index``, ``name``, ``lat``, ``lon``,
        ``x_m``, ``y_m``.
        The first entry is always the Take-Off Location (``"takeoff"``).
    """
    waypoints: list[dict] = []
    idx = 0

    def add(name: str, x: float, y: float) -> None:
        nonlocal idx
        lat, lon = _xy2ll(x, y)
        waypoints.append({
            "index": idx,
            "name":  name,
            "lat":   round(lat, 8),
            "lon":   round(lon, 8),
            "x_m":   round(x, 3),
            "y_m":   round(y, 3),
        })
        idx += 1

    # ── Enter point: 10 m east of P3 ───────────────────────────
    p3_x, p3_y = _POLY_XY[3]   # P3 in local XY
    enter_x    = p3_x + ENTER_OFFSET_M
    enter_y    = p3_y
    add("enter", enter_x, enter_y)

    # ── Inset polygon by HALF_SWATH — actual survey boundary ────────
    inset_verts, _ = _inset_polygon_raw(_POLY_XY, HALF_SWATH)
    if inset_verts is None:
        raise ValueError(
            "Inset polygon collapsed — HALF_SWATH is too large for the polygon."
        )
    # End target: inset-boundary point corresponding to original P1.
    # Choose the inset vertex closest to P1 to avoid depending on vertex order.
    p1_xy = _POLY_XY[1]
    p1_inset_xy = min(
        inset_verts,
        key=lambda p: (p[0] - p1_xy[0]) * (p[0] - p1_xy[0]) +
                      (p[1] - p1_xy[1]) * (p[1] - p1_xy[1])
    )

    # ── Rotate inset polygon to align scan lines with x-axis ──────────
    poly_rot = _rotate(inset_verts, -heading_deg)
    ys = [p[1] for p in poly_rot]
    y_min, y_max = min(ys), max(ys)

    # Sweep from the top edge of the inset polygon downward.
    # The inset boundary is already HALF_SWATH inside the original perimeter,
    # so no additional offset is needed for the first scan line.
    y = y_max
    scan_segments: list[tuple[float, float, float, float]] = []

    while y >= y_min - 1e-9:
        xs = _clip_scanline(poly_rot, y)
        # Each pair of x values defines one left–right scan segment
        for k in range(0, len(xs) - 1, 2):
            scan_segments.append((xs[k], y, xs[k + 1], y))
        y -= SWATH

    if not scan_segments:
        raise ValueError(
            "No scan lines generated — polygon may be too small "
            "for the given SWATH width."
        )

    # ── Connect segments in serpentine order with boundary-follow transitions ──
    # On each scan line we still sweep full width (east→west, then west→east).
    # Between consecutive lines, instead of cutting directly inside the polygon,
    # we move along the inset boundary to preserve edge coverage.
    ordered_lines: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for i, (x1, y1, x2, y2) in enumerate(scan_segments):
        if i % 2 == 0:
            ordered_lines.append(((x2, y2), (x1, y1)))  # east → west
        else:
            ordered_lines.append(((x1, y1), (x2, y2)))  # west → east

    waypoints_rot: list[tuple[float, float]] = []
    if ordered_lines:
        s0, e0 = ordered_lines[0]
        _append_point_unique(waypoints_rot, s0)
        _append_point_unique(waypoints_rot, e0)
        prev_end = e0

        for i in range(1, len(ordered_lines)):
            start_i, end_i = ordered_lines[i]
            trans = _boundary_path(poly_rot, prev_end, start_i)
            for p in trans[1:]:
                _append_point_unique(waypoints_rot, p)
            _append_point_unique(waypoints_rot, end_i)
            prev_end = end_i

    # Final leg: end at inset-boundary point corresponding to P1,
    # following the inset boundary.
    p1_inset_rot = _rotate([p1_inset_xy], -heading_deg)[0]
    if waypoints_rot:
        final_trans = _boundary_path(poly_rot, waypoints_rot[-1], p1_inset_rot)
        for p in final_trans[1:]:
            _append_point_unique(waypoints_rot, p)
    else:
        _append_point_unique(waypoints_rot, p1_inset_rot)

    # ── Rotate back and store waypoints ────────────────────────
    for i, (xr, yr) in enumerate(waypoints_rot):
        xm, ym = _unrotate(xr, yr, -heading_deg)
        add(f"WP{i}", xm, ym)

    return waypoints


def describe_waypoints(waypoints: list[dict]) -> str:
    """Return a formatted table of waypoint coordinates."""
    lines = [
        f"  {'#':>3}  {'Name':<12} {'Lat':>16} {'Lon':>16} "
        f"{'X (m)':>10} {'Y (m)':>10}",
        "  " + "-" * 72,
    ]
    for wp in waypoints:
        lines.append(
            f"  {wp['index']:>3}  {wp['name']:<12} "
            f"{wp['lat']:>16.8f} {wp['lon']:>16.8f} "
            f"{wp['x_m']:>10.2f} {wp['y_m']:>10.2f}"
        )
    return "\n".join(lines)
