"""
perimeter_planner.py — Inward-spiral perimeter path planner
            for a convex polygon search area (Survey Area).
============================================================

Algorithm
---------
The polygon has N vertices P0…P_{N-1} in CCW order.
For each lap k the working polygon is inset by

    offset_k = HALF_SWATH + k × SWATH

**Edge-degeneration rule**
    Before building lap k, compute every edge length of the inset
    polygon.  Any edge whose length ≤ 0 is "collapsed"; its two
    vertices merge and that vertex is dropped from the working polygon
    for this and all subsequent laps.  This is how the pentagon
    degrades naturally into a quad, triangle, etc.

**Waypoint rule (user spec)**
    For lap k with working polygon Q0…Q_{M-1}  (M ≤ N):

    W0_k = intersection of
               • the ORIGINAL last edge  Q_{M-1}→Q0  shifted by
                 offset_k  (its inset line)    ← on this boundary line
               • inset edge 0  Q0→Q1  shifted by offset_k

    Wj_k (j = 1…M-1) = intersection of
               • inset edge (j-1) and inset edge j  for this lap

    This places W0_k on the inset-P_{last}→P0 boundary line, so
    the UAV approach from W_{M-1}_{k} simply follows that boundary
    until it reaches the offset threshold of the first edge —
    exactly ONE new waypoint per lap transition, shortest path.

**Start of mission**
    The UAV departs from the Take-Off Location.
    The very first waypoint inserted is the intersection of:
        • inset-HALF_SWATH of the LAST edge (edge N-1, i.e. P_{N-1}→P0)
        • inset-HALF_SWATH of the SECOND-TO-LAST edge (edge N-2)
    This is the corner of the first lap nearest the approach direction
    (P3P4 ∩ P4P0 inset by 5 m = W4_0 = the existing inset vertex).
    A "takeoff" entry in the waypoint list marks the Take-Off Location.

UAV FOV
-------
    HALF_SWATH = 4 m  →  ±4 m  →  full swath 8 m
"""

from __future__ import annotations
import math, sys, os

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import _DEG_LAT_M, deg_lon_m, haversine_m
from search_area import (
    CORNERS, N_VERTS, CENTER_LAT, CENTER_LON,
    _dlon_m, _dlat_m,
)

# ── Swath / threshold parameters ──────────────────────────────
HALF_SWATH = 5.0
SWATH      = HALF_SWATH * 2      # full swath width in metres

# Take-Off Location  (from KML <Placemark name="Take-Off Location">)
# Kept for reference / plotting only — UAV actually enters from the east.
TAKEOFF_LAT =  51.42340640206451
TAKEOFF_LON = -2.671446029622069

# Enter point: 10 m east of P3 (UAV approaches from the right side)
# P3 XY ≈ (56.41, 78.44) in local metres; +10 m east → (66.41, 78.44)
ENTER_OFFSET_M = 10.0   # metres east of P3

# ── Coordinate helpers ─────────────────────────────────────────
def _ll2xy(lat: float, lon: float) -> tuple[float, float]:
    """(lat, lon) → (x_east_m, y_north_m) relative to centroid."""
    return (lon - CENTER_LON) * _dlon_m, (lat - CENTER_LAT) * _dlat_m

def _xy2ll(x: float, y: float) -> tuple[float, float]:
    """(x_east_m, y_north_m) → (lat, lon)."""
    return CENTER_LAT + y / _dlat_m, CENTER_LON + x / _dlon_m

# Original polygon vertices in local XY (CCW)
_POLY_XY: list[tuple[float, float]] = [_ll2xy(lat, lon) for lat, lon in CORNERS]

# ── Geometry primitives ────────────────────────────────────────

def _line_intersection(p1: tuple[float, float], d1: tuple[float, float],
                       p2: tuple[float, float], d2: tuple[float, float],
                       ) -> tuple[float, float] | None:
    """Intersect lines  L1 = p1+t·d1  and  L2 = p2+s·d2."""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < 1e-12:
        return None
    t = (dx * d2[1] - dy * d2[0]) / cross
    return p1[0] + t * d1[0], p1[1] + t * d1[1]


def _segments_cross(a1: tuple[float, float], a2: tuple[float, float],
                    b1: tuple[float, float], b2: tuple[float, float],
                    tol: float = 1e-6) -> bool:
    """
    Return True if open segment a1→a2 properly crosses open segment b1→b2.
    'Properly' means the intersection is strictly interior to both segments
    (endpoints touching is NOT counted as a crossing).
    """
    def cross2(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    d1 = cross2(b1, b2, a1)
    d2 = cross2(b1, b2, a2)
    d3 = cross2(a1, a2, b1)
    d4 = cross2(a1, a2, b2)
    if ((d1 > tol and d2 < -tol) or (d1 < -tol and d2 > tol)) and \
       ((d3 > tol and d4 < -tol) or (d3 < -tol and d4 > tol)):
        return True
    return False


def _offset_line(poly: list[tuple[float, float]], edge_idx: int,
                 offset: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """
    Inset line for edge edge_idx of a CCW polygon shifted inward by
    `offset` metres.  Returns (point_on_line, unit_direction).
    Inward normal for CCW = left-hand perp = (-ey, ex).
    """
    n  = len(poly)
    ax, ay = poly[edge_idx]
    bx, by = poly[(edge_idx + 1) % n]
    ex, ey = bx - ax, by - ay
    length = math.hypot(ex, ey)
    ex, ey = ex / length, ey / length
    nx, ny = -ey, ex                   # inward normal (CCW)
    return (ax + nx * offset, ay + ny * offset), (ex, ey)


def _inset_polygon(poly: list[tuple[float, float]],
                   offset: float) -> list[tuple[float, float]] | None:
    """
    Full inset of `poly` by `offset` metres.
    Returns None if the result has non-positive signed area (collapsed or
    wound CW, which indicates over-inset / self-intersection).
    """
    n     = len(poly)
    lines = [_offset_line(poly, i, offset) for i in range(n)]
    verts: list[tuple[float, float]] = []
    for i in range(n):
        pt = _line_intersection(lines[(i - 1) % n][0], lines[(i - 1) % n][1],
                                lines[i][0],            lines[i][1])
        if pt is None:
            return None
        verts.append(pt)
    # signed area check — positive means CCW (valid inset)
    area2 = sum(verts[i][0] * verts[(i+1)%n][1] - verts[(i+1)%n][0] * verts[i][1]
                for i in range(n))
    return verts if area2 > 0 else None


def _inset_polygon_raw(poly: list[tuple[float, float]],
                       offset: float) -> tuple[list[tuple[float, float]] | None, float]:
    """
    Like _inset_polygon but also returns the signed area*2 of the result,
    and does NOT reject negative-area results.
    Used by plan() to detect the final valid-but-CW lap after degeneration.
    Returns (None, 0.0) if any intersection is undefined.
    """
    n     = len(poly)
    lines = [_offset_line(poly, i, offset) for i in range(n)]
    verts: list[tuple[float, float]] = []
    for i in range(n):
        pt = _line_intersection(lines[(i - 1) % n][0], lines[(i - 1) % n][1],
                                lines[i][0],            lines[i][1])
        if pt is None:
            return None, 0.0
        verts.append(pt)
    area2 = sum(verts[i][0] * verts[(i+1)%n][1] - verts[(i+1)%n][0] * verts[i][1]
                for i in range(n))
    return (verts if abs(area2) > 1e-6 else None), area2


def _edge_length_at_offset(poly: list[tuple[float, float]],
                            edge_idx: int, offset: float) -> float:
    """
    Return the length of edge `edge_idx` of `poly` when inset by `offset`.
    Works by measuring the original edge length minus the reduction caused
    by the neighbouring inset edges (via their mutual angle).
    """
    n = len(poly)
    # Inset lines for this edge and its neighbours
    line_prev = _offset_line(poly, (edge_idx - 1) % n, offset)
    line_cur  = _offset_line(poly, edge_idx,            offset)
    line_next = _offset_line(poly, (edge_idx + 1) % n, offset)
    # Start vertex = intersection of prev and cur inset lines
    p_start = _line_intersection(line_prev[0], line_prev[1],
                                  line_cur[0],  line_cur[1])
    # End vertex = intersection of cur and next inset lines
    p_end   = _line_intersection(line_cur[0],  line_cur[1],
                                  line_next[0], line_next[1])
    if p_start is None or p_end is None:
        return 0.0
    return math.hypot(p_end[0] - p_start[0], p_end[1] - p_start[1])


def _build_inset_polygon_with_degeneration(
        base_poly: list[tuple[float, float]],
        offset: float,
        ) -> tuple[list[tuple[float, float]], list[int]] | tuple[None, None]:
    """
    Inset `base_poly` by `offset`, remove any edge whose inset length ≤ 0,
    and return the pruned vertex list together with the surviving vertex
    indices in `base_poly`.

    The inset is ALWAYS computed from `base_poly` (the original polygon
    for this degeneration stage), not from a previous inset result.
    Returns (None, None) if fully collapsed.
    """
    n = len(base_poly)
    COLLAPSE_THRESHOLD = 0.0   # strictly positive edge length required

    # Compute each edge's inset length
    surviving_edges: list[int] = []
    for i in range(n):
        L = _edge_length_at_offset(base_poly, i, offset)
        if L > COLLAPSE_THRESHOLD:
            surviving_edges.append(i)

    if len(surviving_edges) < 3:
        return None, None

    # If nothing collapsed, return the full inset polygon
    if len(surviving_edges) == n:
        inset = _inset_polygon(base_poly, offset)
        if inset is None:
            return None, None
        return inset, list(range(n))

    # Some edges collapsed: rebuild a reduced polygon using only the
    # surviving edges.  Merge vertices at the junction of two surviving
    # edges: vertex i is kept iff both edges (i-1) and i survive.
    surviving_verts: list[int] = []
    for v in range(n):
        if (v - 1) % n in surviving_edges and v in surviving_edges:
            surviving_verts.append(v)

    if len(surviving_verts) < 3:
        return None, None

    reduced_poly = [base_poly[v] for v in surviving_verts]

    # Now inset the REDUCED polygon
    inset = _inset_polygon(reduced_poly, offset)
    if inset is None:
        return None, None
    return inset, surviving_verts


# ──────────────────────────────────────────────────────────────
def plan() -> list[dict]:
    """
    Generate the full inward-spiral perimeter waypoint list.

    Mission start
    -------------
    The list begins with an 'enter' waypoint (10 m east of P3).
    The polygon vertices are kept in the original CCW order [P0…P4].
    Each lap is built the same way as before, but emitted starting from
    W{N-2}_k (i.e. W3_k, the inset corner on the P2–P3 / P3–P4 edges
    which is the closest computed point to P3) rather than W{N-1}_k.

    Emission order per lap k (pentagon phase, M = 5):
        W3_k → W4_k → W0_k → W1_k → W2_k
    This means the UAV enters near P3, flies CCW:
        P3 side → P4 side → P0P1 edge → P1P2 edge → P2P3 edge
    then steps inward to the next lap's W3_{k+1} entry.

    Lap waypoint positions  (pentagon phase, M = N = 5)
    ----------------------
        W0_k  = P_{N-1}P_0-inset-offset_k  ∩  P_0P_1-inset-offset_k
        Wj_k  = P_{j-1}Pj-inset-offset_k  ∩  PjP_{j+1}-inset-offset_k
        W{N-2}_k = P_{N-3}P_{N-2}-inset-offset_k ∩ P_{N-2}P_{N-1}-inset-offset_k
        W{N-1}_k = P_{N-2}P_{N-1}-inset-offset_{k-1} ∩ P_{N-1}P_0-inset-offset_k

    P4P0-edge degeneration
    ----------------------
    Before building lap k, compute the signed length of the segment
    W0_k → W{N-1}_k along the P_{N-1}P_0 inset line.  When that length
    drops to ≤ 0 the P_{N-1}P_0 edge has fully collapsed, and the polygon
    degenerates from N to N-1 vertices by dropping P_{N-1}.

    From that lap onward:
      • orig_poly loses its last vertex (P_{N-1} is removed)
      • N_orig decreases by 1
      • The new "last edge" is P_{N-2}→P_0 (was the second-to-last)
      • Waypoint naming resets to W0…W{N-2} for M = N-1
      • Further collapses are handled recursively the same way
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

    # ── Build all lap data first ─────────────────────────────
    # Keep the original polygon vertex order [P0,P1,P2,P3,P4].
    # The emission order is changed (see below) so the UAV starts near P3.
    orig_poly  = list(_POLY_XY)
    penta_poly = list(_POLY_XY)   # original pentagon, preserved for inter-lap lines
    N_penta    = len(penta_poly)  # = 5 always
    N_orig     = len(orig_poly)
    k_in_phase = 0

    # lap_wps[k] = list of (name, x, y); last element is always W{M-1}_k
    lap_wps: list[list[tuple[str, float, float]]] = []

    # survey_segs: only the actual survey flight segments (W4→W1, W1→W2,
    # W2→W3 …).  Transition segments (W3_{k-1}→W4_k, pure navigation) are
    # intentionally excluded so they cannot falsely trigger the crossing
    # check for inner-lap survey segments.
    survey_segs: list[tuple[tuple[float,float], tuple[float,float]]] = []

    def _lap_segments(lap: list[tuple[str, float, float]]
                      ) -> list[tuple[tuple[float,float], tuple[float,float]]]:
        """Return the ordered flight segments for a lap (W3-first order)."""
        segs = []
        order = [lap[-2], lap[-1]] + list(lap[:-2])   # W{M-2}=W3, W{M-1}=W4, W0, …, W{M-3}
        for i in range(len(order) - 1):
            segs.append(((order[i][1], order[i][2]),
                         (order[i+1][1], order[i+1][2])))
        return segs

    def _crosses_history(a1: tuple[float,float], a2: tuple[float,float],
                         history: list[tuple[tuple[float,float],tuple[float,float]]]) -> bool:
        """True if segment a1→a2 crosses any segment in the given history list."""
        for seg_a, seg_b in history:
            if _segments_cross(a1, a2, seg_a, seg_b):
                return True
        return False

    # Max radius of the original polygon (centroid → farthest vertex).
    # Any inset vertex further than this indicates an over-inset / blown-up
    # intersection and is treated as a degeneration signal.
    _max_poly_r = max(math.hypot(x, y) for x, y in _POLY_XY)

    k = 0
    while True:
        offset = HALF_SWATH + k * SWATH

        # Use raw inset (accepts CW results) so degenerated quads are not
        # prematurely rejected; termination is now driven by crossing check.
        raw_verts, raw_area2 = _inset_polygon_raw(orig_poly, offset)
        if raw_verts is None:
            break
        # Sanity check: if any inset vertex is further from the centroid than
        # the original polygon's max radius, the inset has over-extended due
        # to a nearly-collapsed edge.  Terminate cleanly instead of generating
        # a lap with explosion-coordinate waypoints.
        if any(math.hypot(x, y) > _max_poly_r for x, y in raw_verts):
            break
        work = raw_verts
        m = len(work)

        # ── W0_k: last-edge-inset ∩ first-edge-inset ──────────────────────
        orig_last_line = _offset_line(orig_poly, N_orig - 1, offset)
        wx0, wy0 = work[0]
        wx1, wy1 = work[1 % m]
        ex_ = wx1 - wx0; ey_ = wy1 - wy0
        L_  = math.hypot(ex_, ey_)
        if L_ < 1e-9:
            break
        first_edge_line = ((wx0, wy0), (ex_ / L_, ey_ / L_))
        w0 = _line_intersection(orig_last_line[0], orig_last_line[1],
                                 first_edge_line[0], first_edge_line[1])
        if w0 is None:
            break

        # ── W{M-1}_k: standard corner near P4 (P3P4-inset ∩ P4P0-inset) ──
        # W4_k = P3P4-inset(off_k) ∩ P4P0-inset(off_k).
        # Pentagon phase: work[m-1] is exactly this corner.
        # Post-degeneration: the quad has no P4 vertex, so compute from penta_poly.
        if N_orig < N_penta:
            e_P3P4_w4 = _offset_line(penta_poly, N_penta - 2, offset)
            e_P4P0_w4 = _offset_line(penta_poly, N_penta - 1, offset)
            pt_w4 = _line_intersection(e_P3P4_w4[0], e_P3P4_w4[1],
                                       e_P4P0_w4[0], e_P4P0_w4[1])
            last_x, last_y = pt_w4 if pt_w4 is not None else work[m - 1]
        else:
            last_x, last_y = work[m - 1]

        # ── Degeneration check ─────────────────────────────────────────────
        ldir = orig_last_line[1]
        signed_len = ((w0[0] - last_x) * ldir[0] +
                      (w0[1] - last_y) * ldir[1])

        lap: list[tuple[str, float, float]] = []

        if signed_len <= 0:
            # Degeneration lap: W0_k omitted; W3 and W4 from pentagon lines.
            # W numbering is fixed to the original pentagon vertex indices:
            #   W1 … W{N_penta-3}: from quad inset work[1] … work[N_penta-3-1]
            #   W{N_penta-2} = W3: P2P3-inset(prev) ∩ P3P4-inset(cur) from pentagon
            #   W{N_penta-1} = W4: P3P4-inset(cur) ∩ P4P0-inset(cur) standard corner
            prev_off_dg = (HALF_SWATH + (k - 1) * SWATH) if k > 0 else offset
            e_P2P3  = _offset_line(penta_poly, N_penta - 3, prev_off_dg)
            e_P3P4c = _offset_line(penta_poly, N_penta - 2, offset)
            w3_pt = _line_intersection(e_P2P3[0], e_P2P3[1],
                                        e_P3P4c[0], e_P3P4c[1])
            # W1 … W{N_penta-3} from quad inset (indices 1 … N_penta-3 in work)
            for j in range(1, N_penta - 2):
                lap.append((f"W{j}_{k}", work[j][0], work[j][1]))
            # W3 (= W{N_penta-2}) from pentagon
            w3x = w3_pt[0] if w3_pt is not None else work[N_penta - 3][0]
            w3y = w3_pt[1] if w3_pt is not None else work[N_penta - 3][1]
            lap.append((f"W{N_penta - 2}_{k}", w3x, w3y))
            # W4 (= W{N_penta-1}) standard corner
            lap.append((f"W{N_penta - 1}_{k}", last_x, last_y))

            # ── Crossing check — segment-by-segment commit ─────────────────
            # Each new segment is checked against a PRE-LAP snapshot of
            # all_segments.  This prevents the transition segment (just
            # committed as candidate_segs[0]) from being treated as a
            # "previous-lap obstacle" when checking W1→W2, W2→W3 etc.
            lap_segs = _lap_segments(lap)
            pre_lap_survey = list(survey_segs)     # snapshot: only survey segs, no transitions
            has_trans = bool(pre_lap_survey)
            candidate_segs: list[tuple[tuple[float,float],tuple[float,float]]] = []
            if has_trans:
                prev_pt  = pre_lap_survey[-1][1]   # last survey endpoint = departure
                entry_pt = (w3x, w3y)              # W3_k: first emitted nav-point
                candidate_segs.append((prev_pt, entry_pt))
            candidate_segs.extend(lap_segs)

            n_committed = 0
            crossing_found = False
            for i_seg, seg in enumerate(candidate_segs):
                if _crosses_history(seg[0], seg[1], pre_lap_survey):
                    crossing_found = True
                    break
                # Transitions (i_seg==0 when has_trans) are NOT added to survey_segs;
                # they are pure navigation and must not block inner survey segments.
                if not (i_seg == 0 and has_trans):
                    survey_segs.append(seg)
                n_committed += 1

            if crossing_found:
                break

            lap_wps.append(lap)
            # Degenerate polygon for all subsequent laps
            orig_poly  = orig_poly[:-1]
            N_orig     = len(orig_poly)
            k_in_phase = 0
            k         += 1
            if N_orig < 3:
                break
        else:
            # Normal lap: [W0_k, W1_k, …, W{M-2}_k, W{M-1}_k]
            if N_orig < N_penta:
                # Post-degeneration quad lap: W0 is omitted (same as degeneration
                # lap rule). W1/W2 from quad inset; W3 from pentagon P2P3∩P3P4;
                # W4 entry already computed above via pentagon edges.
                # work indices for quad: [0]=P3P0∩P0P1, [1]=P0P1∩P1P2, [2]=P1P2∩P2P3
                lap.append((f"W1_{k}", work[1][0], work[1][1]))
                lap.append((f"W2_{k}", work[2][0], work[2][1]))
                prev_off_pd2 = HALF_SWATH + (k - 1) * SWATH
                e_P2P3pd = _offset_line(penta_poly, N_penta - 3, prev_off_pd2)
                e_P3P4pd = _offset_line(penta_poly, N_penta - 2, offset)
                w3_pd = _line_intersection(e_P2P3pd[0], e_P2P3pd[1],
                                           e_P3P4pd[0], e_P3P4pd[1])
                w3x = w3_pd[0] if w3_pd is not None else work[2][0]
                w3y = w3_pd[1] if w3_pd is not None else work[2][1]
                lap.append((f"W3_{k}", w3x, w3y))
                lap.append((f"W4_{k}", last_x, last_y))
            else:
                # Pentagon phase: full W0…W{m-1}
                lap.append((f"W0_{k}", w0[0], w0[1]))
                for j in range(1, m - 2):
                    lap.append((f"W{j}_{k}", work[j][0], work[j][1]))
                # W3_k: inter-lap corner on P2P3-inset(prev_off) ∩ P3P4-inset(off)
                if k_in_phase == 0:
                    w3x, w3y = work[m - 2]
                else:
                    prev_off_pp = HALF_SWATH + (k_in_phase - 1) * SWATH
                    e_P2P3pp = _offset_line(penta_poly, N_penta - 3, prev_off_pp)
                    e_P3P4pp = _offset_line(penta_poly, N_penta - 2, offset)
                    w3_pp = _line_intersection(e_P2P3pp[0], e_P2P3pp[1],
                                               e_P3P4pp[0], e_P3P4pp[1])
                    w3x = w3_pp[0] if w3_pp is not None else work[m - 2][0]
                    w3y = w3_pp[1] if w3_pp is not None else work[m - 2][1]
                lap.append((f"W{m-2}_{k}", w3x, w3y))
                lap.append((f"W{m-1}_{k}", last_x, last_y))

            # ── Crossing check — segment-by-segment commit ─────────────────
            lap_segs = _lap_segments(lap)
            pre_lap_survey2 = list(survey_segs)    # snapshot: only survey segs, no transitions
            has_trans2 = bool(pre_lap_survey2)
            candidate_segs2: list[tuple[tuple[float,float],tuple[float,float]]] = []
            if has_trans2:
                prev_pt  = pre_lap_survey2[-1][1]
                entry_pt = (w3x, w3y)              # W3_k: first emitted nav-point
                candidate_segs2.append((prev_pt, entry_pt))
            candidate_segs2.extend(lap_segs)

            n_committed2 = 0
            crossing_found2 = False
            for i_seg2, seg in enumerate(candidate_segs2):
                if _crosses_history(seg[0], seg[1], pre_lap_survey2):
                    crossing_found2 = True
                    break
                if not (i_seg2 == 0 and has_trans2):
                    survey_segs.append(seg)
                n_committed2 += 1

            if crossing_found2:
                break

            lap_wps.append(lap)
            k         += 1
            k_in_phase += 1

    # ── Always end with W3_k (entry of the aborted lap) before centre ────
    # W3 = P2P3-inset ∩ P3P4-inset from the original pentagon, independent
    # of any polygon degeneration.  This gives a clean terminus near P3.
    if lap_wps and k > 0:
        off_k      = HALF_SWATH + k * SWATH
        off_prev_k = HALF_SWATH + (k - 1) * SWATH
        e_P2P3f = _offset_line(penta_poly, N_penta - 3, off_prev_k)
        e_P3P4f = _offset_line(penta_poly, N_penta - 2, off_k)
        w3_end = _line_intersection(e_P2P3f[0], e_P2P3f[1],
                                     e_P3P4f[0], e_P3P4f[1])
        if w3_end is not None and math.hypot(w3_end[0], w3_end[1]) <= _max_poly_r:
            lap_wps.append([(f"W3_{k}", w3_end[0], w3_end[1])])

    if not lap_wps:
        add("centre", 0.0, 0.0)
        return waypoints

    # ── Emit waypoints ─────────────────────────────────────────
    # Emission order per lap: W3_k → W4_k → W0_k → W1_k → W2_k
    # Lap list is stored as [W0, W1, W2, W3, W4]; W3=lap[M-2], W4=lap[M-1]
    # Reorder to: W{M-2}, W{M-1}, W0, W1, …, W{M-3}

    for lap in lap_wps:
        m = len(lap)
        if m == 1:
            name, x, y = lap[0]
            add(name, x, y)
            continue
        # W3_k (second-to-last) → W4_k (last) → W0 … W{M-3}
        near_p3 = lap[m - 2]
        near_p4 = lap[m - 1]
        add(near_p3[0], near_p3[1], near_p3[2])
        add(near_p4[0], near_p4[1], near_p4[2])
        for j in range(m - 2):
            name, x, y = lap[j]
            add(name, x, y)

    # ── Centre waypoint ────────────────────────────────────────
    add("centre", 0.0, 0.0)

    return waypoints


# ──────────────────────────────────────────────────────────────
def describe_waypoints(waypoints: list[dict]) -> str:
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
