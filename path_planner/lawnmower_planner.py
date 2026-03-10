"""
lawnmower_planner.py — Lawnmower (boustrophedon) path planner
              for the convex polygon search area (Survey Area).
================================================================

Algorithm  (Algorithm 0, verified on real hardware)
---------
The polygon is first rotated by ``-HEADING_DEG`` so that scan lines run
horizontally.  A set of horizontal scan lines is generated from
``y_min + SWATH/2`` to ``y_max`` at intervals of ``SWATH``.  For each
line the polygon edges are clipped to find the left and right endpoints.
Consecutive lines are joined in alternating direction (serpentine /
boustrophedon) to minimise transit distances.  The resulting points are
rotated back to the original orientation and converted to (lat, lon).

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

# ── Swath / heading parameters ─────────────────────────────────
HALF_SWATH  = 5.0            # metres — adjust to change line spacing
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


# ── Public API ─────────────────────────────────────────────────

def plan(heading_deg: float = HEADING_DEG) -> list[dict]:
    """
    Generate lawnmower (boustrophedon) waypoints for the search area.

    The scan-line spacing equals ``SWATH`` (= 2 × HALF_SWATH).  The first
    line is offset inward by ``HALF_SWATH`` from the polygon boundary so
    coverage starts at the correct half-swath distance.

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

    # ── Rotate polygon to align scan lines with x-axis ─────────
    poly_rot = _rotate(_POLY_XY, -heading_deg)
    ys = [p[1] for p in poly_rot]
    y_min, y_max = min(ys), max(ys)

    # Sweep from top (P3 side, y_max) downward, first line offset inward by HALF_SWATH
    y = y_max - HALF_SWATH
    scan_segments: list[tuple[float, float, float, float]] = []

    while y > y_min:
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

    # ── Connect segments in serpentine (boustrophedon) order ───
    # With HEADING_DEG=0, y_rot = y_orig.  Sweeping from y_max downward means
    # the first scan line is at the top (P3 side).
    # x1_rot < x2_rot → x1 is the western end, x2 is the eastern end.
    # Start each even line from x2 (east/right) → x1 (west/left),
    # odd lines from x1 (west) → x2 (east).
    waypoints_rot: list[tuple[float, float]] = []
    for i, (x1, y1, x2, y2) in enumerate(scan_segments):
        if i % 2 == 0:
            waypoints_rot += [(x2, y2), (x1, y1)]   # east → west
        else:
            waypoints_rot += [(x1, y1), (x2, y2)]   # west → east

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
