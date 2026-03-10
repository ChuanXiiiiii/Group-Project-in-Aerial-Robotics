"""
search_area.py — Define and inspect the real convex search area.
================================================================

Vertices are taken directly from the KML "Survey Area" polygon
(AENGM0074.kml).  The five vertices are stored in counter-clockwise
order (verified by positive shoelace signed area):

    P0  SW vertex
    P1  SE vertex  (southernmost)
    P2  E  vertex
    P3  N  vertex  (northernmost)
    P4  NW vertex

Side lengths (haversine)
------------------------
    P0 → P1   76.75 m
    P1 → P2  141.29 m
    P2 → P3  104.28 m
    P3 → P4  186.82 m
    P4 → P0   37.82 m
    Perimeter 546.97 m
    Area      17 379.2 m²  (1.74 ha)
"""

import math
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from geo_utils import _DEG_LAT_M, deg_lon_m, haversine_m

# ── KML Survey Area vertices: (lat, lon) in CCW order ──────────
#   Source: AENGM0074.kml  <Placemark name="Survey Area">
CORNERS = [
    (51.42326956502679, -2.670948345438704),  # P0
    (51.42287025017865, -2.670045428650557),  # P1
    (51.42336622593724, -2.668169295906676),  # P2
    (51.42421477437771, -2.668809768621569),  # P3
    (51.42354069739116, -2.671277780473196),  # P4
]
CORNER_NAMES = ["P0", "P1", "P2", "P3", "P4"]
N_VERTS = len(CORNERS)

# ── Geometric centroid (polygon centroid formula) ──────────────
# Step 1: arithmetic-mean origin used as a temporary projection reference
_lat_mean = sum(lat for lat, _ in CORNERS) / N_VERTS
_lon_mean = sum(lon for _, lon in CORNERS) / N_VERTS

# Step 2: scale factors at the temporary origin
_dlon_m = deg_lon_m(_lat_mean)
_dlat_m = _DEG_LAT_M

# Step 3: project vertices to local flat-Earth XY
_xs_tmp = [(lon - _lon_mean) * _dlon_m for _, lon in CORNERS]
_ys_tmp = [(lat - _lat_mean) * _dlat_m for lat, _ in CORNERS]

# Step 4: shoelace-based polygon centroid
_A2 = 0.0
_cx = 0.0
_cy = 0.0
for _i in range(N_VERTS):
    _j = (_i + 1) % N_VERTS
    _cross = _xs_tmp[_i] * _ys_tmp[_j] - _xs_tmp[_j] * _ys_tmp[_i]
    _A2   += _cross
    _cx   += (_xs_tmp[_i] + _xs_tmp[_j]) * _cross
    _cy   += (_ys_tmp[_i] + _ys_tmp[_j]) * _cross
_cx /= (3.0 * _A2)
_cy /= (3.0 * _A2)

# Step 5: convert the XY centroid back to lat/lon
CENTER_LAT = _lat_mean + _cy / _dlat_m
CENTER_LON = _lon_mean + _cx / _dlon_m

# ── Scale factors at true centroid ─────────────────────────────
_dlon_m = deg_lon_m(CENTER_LAT)
_dlat_m = _DEG_LAT_M

# ── Side lengths ───────────────────────────────────────────────
SIDE_LENGTHS_M: list[float] = []
for _i in range(N_VERTS):
    _lat1, _lon1 = CORNERS[_i]
    _lat2, _lon2 = CORNERS[(_i + 1) % N_VERTS]
    SIDE_LENGTHS_M.append(haversine_m(_lat1, _lon1, _lat2, _lon2))

# ── Area via shoelace (local flat-Earth) ───────────────────────
def _to_xy(lat: float, lon: float) -> tuple[float, float]:
    return (lon - CENTER_LON) * _dlon_m, (lat - CENTER_LAT) * _dlat_m

_xs = [_to_xy(lat, lon)[0] for lat, lon in CORNERS]
_ys = [_to_xy(lat, lon)[1] for lat, lon in CORNERS]
_area = 0.0
for _i in range(N_VERTS):
    _j = (_i + 1) % N_VERTS
    _area += _xs[_i] * _ys[_j] - _xs[_j] * _ys[_i]
AREA_M2 = abs(_area) / 2.0


def describe() -> str:
    """Return a human-readable summary of the search area geometry."""
    lines = [
        "=" * 66,
        "  Search Area — Fenswood Farm  (KML Survey Area, convex pentagon)",
        "=" * 66,
    ]
    for i in range(N_VERTS):
        n0 = CORNER_NAMES[i]
        n1 = CORNER_NAMES[(i + 1) % N_VERTS]
        lines.append(f"  Side {n0} → {n1}: {SIDE_LENGTHS_M[i]:>8.2f} m")
    lines += [
        f"  Area                          : {AREA_M2:>10.1f} m²"
        f"  ({AREA_M2 / 1e4:.4f} ha)",
        f"  Centroid : lat={CENTER_LAT:.8f}  lon={CENTER_LON:.8f}",
        "",
        "  Vertices:",
    ]
    for name, (lat, lon) in zip(CORNER_NAMES, CORNERS):
        lines.append(f"    {name}: lat={lat:.8f}  lon={lon:.8f}")
    lines.append("=" * 66)
    return "\n".join(lines)


if __name__ == "__main__":
    print(describe())
