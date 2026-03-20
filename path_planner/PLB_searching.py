"""
PLB_searching.py - PLB region mission planner inside the search area.

This module generates a random convex PLB polygon with n vertices (3 <= n <= 10)
inside the global search area, picks a random UAV start point inside the search
area, then plans a mission that flies from start -> PLB and searches the PLB with:

    mode 0: lawnmower
    mode 1: perimeter spiral
"""

from __future__ import annotations

import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from search_area import CORNERS, CENTER_LAT, CENTER_LON, _dlat_m, _dlon_m


HALF_SWATH = 5.5
SWATH = HALF_SWATH * 2.0
LAWNMOWER_HEADING_DEG = 0.0


def _ll2xy(lat: float, lon: float) -> tuple[float, float]:
    return (lon - CENTER_LON) * _dlon_m, (lat - CENTER_LAT) * _dlat_m


def _xy2ll(x: float, y: float) -> tuple[float, float]:
    return CENTER_LAT + y / _dlat_m, CENTER_LON + x / _dlon_m


_SEARCH_POLY_XY: list[tuple[float, float]] = [_ll2xy(lat, lon) for lat, lon in CORNERS]


def _signed_area(poly: list[tuple[float, float]]) -> float:
    area2 = 0.0
    n = len(poly)
    for i in range(n):
        j = (i + 1) % n
        area2 += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1]
    return 0.5 * area2


def _ensure_ccw(poly: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return poly if _signed_area(poly) > 0 else list(reversed(poly))


def _centroid(poly: list[tuple[float, float]]) -> tuple[float, float]:
    cx = sum(p[0] for p in poly) / len(poly)
    cy = sum(p[1] for p in poly) / len(poly)
    return cx, cy


def _line_intersection(
    p1: tuple[float, float],
    d1: tuple[float, float],
    p2: tuple[float, float],
    d2: tuple[float, float],
) -> tuple[float, float] | None:
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < 1e-12:
        return None
    t = (dx * d2[1] - dy * d2[0]) / cross
    return p1[0] + t * d1[0], p1[1] + t * d1[1]


def _offset_line(
    poly: list[tuple[float, float]],
    edge_idx: int,
    offset: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    n = len(poly)
    ax, ay = poly[edge_idx]
    bx, by = poly[(edge_idx + 1) % n]
    ex, ey = bx - ax, by - ay
    length = math.hypot(ex, ey)
    ex, ey = ex / length, ey / length
    nx, ny = -ey, ex
    return (ax + nx * offset, ay + ny * offset), (ex, ey)


def _inset_polygon(
    poly: list[tuple[float, float]],
    offset: float,
) -> list[tuple[float, float]] | None:
    poly = _ensure_ccw(poly)
    n = len(poly)
    lines = [_offset_line(poly, i, offset) for i in range(n)]
    verts: list[tuple[float, float]] = []
    for i in range(n):
        pt = _line_intersection(
            lines[(i - 1) % n][0],
            lines[(i - 1) % n][1],
            lines[i][0],
            lines[i][1],
        )
        if pt is None:
            return None
        verts.append(pt)
    area = _signed_area(verts)
    # For a valid inward inset of a CCW polygon, area must stay positive.
    if area <= 1e-6:
        return None
    return verts


def _rotate_point(x: float, y: float, angle_deg: float) -> tuple[float, float]:
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    return x * c - y * s, x * s + y * c


def _rotate_points(
    points: list[tuple[float, float]],
    angle_deg: float,
) -> list[tuple[float, float]]:
    return [_rotate_point(x, y, angle_deg) for x, y in points]


def _segment_intersect_y(
    p1: tuple[float, float],
    p2: tuple[float, float],
    y: float,
) -> float | None:
    (x1, y1), (x2, y2) = p1, p2
    if (y1 - y) * (y2 - y) > 0:
        return None
    if abs(y2 - y1) < 1e-12:
        return None
    t = (y - y1) / (y2 - y1)
    if t < 0.0 or t > 1.0:
        return None
    return x1 + t * (x2 - x1)


def _clip_scanline(poly: list[tuple[float, float]], y: float) -> list[float]:
    xs: list[float] = []
    n = len(poly)
    for i in range(n):
        x = _segment_intersect_y(poly[i], poly[(i + 1) % n], y)
        if x is not None:
            xs.append(x)
    xs.sort()
    return xs


def _append_unique(
    pts: list[tuple[float, float]],
    p: tuple[float, float],
    tol: float = 1e-6,
) -> None:
    if not pts:
        pts.append(p)
        return
    if math.hypot(pts[-1][0] - p[0], pts[-1][1] - p[1]) > tol:
        pts.append(p)


def _compact_cycle_short_edges(
    points: list[tuple[float, float]],
    min_len: float,
) -> list[tuple[float, float]]:
    """Remove vertices that form very short edges in a cyclic polygon."""
    if len(points) < 3:
        return list(points)

    pts = list(points)
    changed = True
    while changed and len(pts) > 3:
        changed = False
        n = len(pts)
        remove_idx: int | None = None
        for i in range(n):
            a = pts[i]
            b = pts[(i + 1) % n]
            if math.hypot(b[0] - a[0], b[1] - a[1]) < min_len:
                remove_idx = (i + 1) % n
                break
        if remove_idx is not None:
            pts.pop(remove_idx)
            changed = True

    return pts


def _point_in_convex(poly: list[tuple[float, float]], p: tuple[float, float]) -> bool:
    poly = _ensure_ccw(poly)
    px, py = p
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        rx, ry = px - ax, py - ay
        cross = ex * ry - ey * rx
        if cross < -1e-9:
            return False
    return True


def _sample_point_in_convex(
    poly: list[tuple[float, float]],
    rng: random.Random,
) -> tuple[float, float]:
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)

    for _ in range(5000):
        p = (rng.uniform(x_min, x_max), rng.uniform(y_min, y_max))
        if _point_in_convex(poly, p):
            return p
    raise RuntimeError("Failed to sample a point inside the search polygon.")


def _distance_to_edge_line(
    p: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    ex, ey = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ex, ey)
    ex, ey = ex / length, ey / length
    nx, ny = -ey, ex
    return (p[0] - a[0]) * nx + (p[1] - a[1]) * ny


def _min_inward_clearance(
    poly: list[tuple[float, float]],
    p: tuple[float, float],
) -> float:
    poly = _ensure_ccw(poly)
    n = len(poly)
    d_min = float("inf")
    for i in range(n):
        d = _distance_to_edge_line(p, poly[i], poly[(i + 1) % n])
        d_min = min(d_min, d)
    return d_min


def _generate_random_convex_plb(
    search_poly: list[tuple[float, float]],
    n: int,
    rng: random.Random,
) -> list[tuple[float, float]]:
    if not 3 <= n <= 10:
        raise ValueError("n must satisfy 3 <= n <= 10")

    search_poly = _ensure_ccw(search_poly)

    for _ in range(1000):
        center = _sample_point_in_convex(search_poly, rng)
        clearance = _min_inward_clearance(search_poly, center)

        if clearance < 8.0:
            continue

        radius_low = max(6.0, clearance * 0.35)
        radius_high = max(radius_low + 1.0, clearance * 0.75)
        radius = rng.uniform(radius_low, radius_high)

        base = rng.uniform(0.0, 2.0 * math.pi)
        jitter = 0.35 * (2.0 * math.pi / n)
        angles = []
        for i in range(n):
            a = base + i * (2.0 * math.pi / n) + rng.uniform(-jitter, jitter)
            angles.append(a)
        angles.sort()

        verts = [
            (center[0] + radius * math.cos(a), center[1] + radius * math.sin(a))
            for a in angles
        ]

        if all(_point_in_convex(search_poly, v) for v in verts):
            if abs(_signed_area(verts)) > 1.0:
                return _ensure_ccw(verts)

    raise RuntimeError("Failed to generate a random convex PLB polygon.")


def _plan_lawnmower(
    plb_poly: list[tuple[float, float]],
    start_hint: tuple[float, float],
    swath: float = SWATH,
    half_swath: float = HALF_SWATH,
    heading_deg: float = LAWNMOWER_HEADING_DEG,
) -> list[tuple[float, float]]:
    plb_poly = _ensure_ccw(plb_poly)

    # Scan the original PLB polygon and place line centers so the swath
    # footprint (half_swath on each side) reaches both top and bottom edges.
    rot_poly = _rotate_points(plb_poly, -heading_deg)
    ys = [p[1] for p in rot_poly]
    y_min, y_max = min(ys), max(ys)

    scan_segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    height = y_max - y_min
    if height <= 2.0 * half_swath + 1e-9:
        line_ys = [(y_min + y_max) * 0.5]
    else:
        line_ys = []
        y = y_max - half_swath
        while y >= y_min + half_swath - 1e-9:
            line_ys.append(y)
            y -= swath
        bottom_line = y_min + half_swath
        if abs(line_ys[-1] - bottom_line) > 1e-6:
            line_ys.append(bottom_line)

    for y in line_ys:
        xs = _clip_scanline(rot_poly, y)
        if len(xs) >= 2 and (xs[-1] - xs[0]) > 1e-6:
            scan_segments.append(((xs[0], y), (xs[-1], y)))

    if not scan_segments:
        return [_centroid(plb_poly)]

    ordered: list[tuple[float, float]] = []
    for i, (left, right) in enumerate(scan_segments):
        s, e = (right, left) if i % 2 == 0 else (left, right)
        _append_unique(ordered, s)
        _append_unique(ordered, e)

    # Choose the direction with shorter transit from start point.
    start_rot = _rotate_point(start_hint[0], start_hint[1], -heading_deg)
    d0 = math.hypot(ordered[0][0] - start_rot[0], ordered[0][1] - start_rot[1])
    d1 = math.hypot(ordered[-1][0] - start_rot[0], ordered[-1][1] - start_rot[1])
    if d1 < d0:
        ordered.reverse()

    return _rotate_points(ordered, heading_deg)


def _plan_perimeter_spiral(
    plb_poly: list[tuple[float, float]],
    start_hint: tuple[float, float],
    swath: float = SWATH,
    half_swath: float = HALF_SWATH,
) -> list[tuple[float, float]]:
    plb_poly = _ensure_ccw(plb_poly)

    laps: list[list[tuple[float, float]]] = []
    base_area = abs(_signed_area(plb_poly))
    prev_area = base_area
    cx0, cy0 = _centroid(plb_poly)
    max_r0 = max(math.hypot(x - cx0, y - cy0) for x, y in plb_poly)
    min_edge_keep = max(1.0, swath * 0.05)
    k = 0
    while k < 200:
        lap = _inset_polygon(plb_poly, half_swath + k * swath)
        if lap is None:
            break
        lap = _compact_cycle_short_edges(lap, min_len=min_edge_keep)
        area = abs(_signed_area(lap))
        if len(lap) < 3 or area < 1.0:
            break
        # Invalid inset expansion after collapse should terminate the spiral.
        if area >= prev_area - 1e-6:
            break
        if any(math.hypot(x - cx0, y - cy0) > max_r0 * 1.2 for x, y in lap):
            break
        laps.append(lap)
        prev_area = area
        k += 1

    if not laps:
        return [_centroid(plb_poly)]

    path: list[tuple[float, float]] = []
    current = start_hint

    for lap in laps:
        start_idx = min(
            range(len(lap)),
            key=lambda i: math.hypot(lap[i][0] - current[0], lap[i][1] - current[1]),
        )

        ccw = lap[start_idx:] + lap[:start_idx]
        cw = [ccw[0]] + list(reversed(ccw[1:]))

        if len(ccw) > 1:
            next_ccw = math.hypot(ccw[1][0] - current[0], ccw[1][1] - current[1])
            next_cw = math.hypot(cw[1][0] - current[0], cw[1][1] - current[1])
            ordered = ccw if next_ccw <= next_cw else cw
        else:
            ordered = ccw

        for p in ordered:
            _append_unique(path, p, tol=1.0)
        _append_unique(path, ordered[0], tol=1.0)
        current = path[-1]

    return path


def _xy_waypoint(index: int, name: str, x: float, y: float) -> dict:
    lat, lon = _xy2ll(x, y)
    return {
        "index": index,
        "name": name,
        "lat": round(lat, 8),
        "lon": round(lon, 8),
        "x_m": round(x, 3),
        "y_m": round(y, 3),
    }


def plan_plb_search(
    n: int,
    seed: int | None = None,
    algorithm_mode: int | None = None,
) -> dict:
    """
    Plan a PLB mission:
    1) randomly generate an n-vertex convex PLB polygon in search area
    2) sample a random UAV start point inside search area
    3) fly to PLB and search using selected algorithm mode

    Parameters
    ----------
    n : int
        PLB polygon vertex count, must satisfy 3 <= n <= 10.
    seed : int | None
        Random seed for reproducibility.
    algorithm_mode : int | None
        Optional override:
            0 = lawnmower
            1 = perimeter spiral

    Returns
    -------
    dict
        {
            "n": int,
            "algorithm_mode": int,
            "start_point": {...},
            "plb_polygon": [...],
            "waypoints": [...]
        }
    """
    if not 3 <= n <= 10:
        raise ValueError("n must satisfy 3 <= n <= 10")

    rng = random.Random(seed)

    start_xy = _sample_point_in_convex(_SEARCH_POLY_XY, rng)
    plb_poly_xy = _generate_random_convex_plb(_SEARCH_POLY_XY, n, rng)

    # Function-internal selector variable requested by user spec.
    mode = 0 if algorithm_mode is None else algorithm_mode

    if mode == 0:
        search_path_xy = _plan_lawnmower(plb_poly_xy, start_xy)
    elif mode == 1:
        search_path_xy = _plan_perimeter_spiral(plb_poly_xy, start_xy)
    else:
        raise ValueError("algorithm_mode must be 0 (lawnmower) or 1 (perimeter spiral)")

    if not search_path_xy:
        search_path_xy = [_centroid(plb_poly_xy)]

    waypoints: list[dict] = []
    idx = 0

    waypoints.append(_xy_waypoint(idx, "start", start_xy[0], start_xy[1]))
    idx += 1

    entry_xy = search_path_xy[0]
    if math.hypot(entry_xy[0] - start_xy[0], entry_xy[1] - start_xy[1]) > 1e-6:
        waypoints.append(_xy_waypoint(idx, "plb_entry", entry_xy[0], entry_xy[1]))
        idx += 1
        path_start = 0 if len(search_path_xy) == 1 else 1
    else:
        path_start = 0

    for i in range(path_start, len(search_path_xy)):
        x, y = search_path_xy[i]
        waypoints.append(_xy_waypoint(idx, f"search_{i - path_start}", x, y))
        idx += 1

    plb_polygon_ll = []
    for i, (x, y) in enumerate(plb_poly_xy):
        lat, lon = _xy2ll(x, y)
        plb_polygon_ll.append(
            {
                "name": f"plb_p{i}",
                "lat": round(lat, 8),
                "lon": round(lon, 8),
                "x_m": round(x, 3),
                "y_m": round(y, 3),
            }
        )

    return {
        "n": n,
        "algorithm_mode": mode,
        "start_point": waypoints[0],
        "plb_polygon": plb_polygon_ll,
        "waypoints": waypoints,
    }


def describe_waypoints(waypoints: list[dict]) -> str:
    lines = [
        f"  {'#':>3}  {'Name':<12} {'Lat':>16} {'Lon':>16} {'X (m)':>10} {'Y (m)':>10}",
        "  " + "-" * 72,
    ]
    for wp in waypoints:
        lines.append(
            f"  {wp['index']:>3}  {wp['name']:<12} "
            f"{wp['lat']:>16.8f} {wp['lon']:>16.8f} "
            f"{wp['x_m']:>10.2f} {wp['y_m']:>10.2f}"
        )
    return "\n".join(lines)


if __name__ == "__main__":
    demo = plan_plb_search(n=6, seed=42, algorithm_mode=0)
    print("PLB polygon vertices:")
    for p in demo["plb_polygon"]:
        print(
            f"  {p['name']}: lat={p['lat']:.8f} lon={p['lon']:.8f} "
            f"(x={p['x_m']:.2f}, y={p['y_m']:.2f})"
        )
    print("\nWaypoints:")
    print(describe_waypoints(demo["waypoints"]))