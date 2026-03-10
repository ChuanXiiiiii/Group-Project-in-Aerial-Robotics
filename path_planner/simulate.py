"""
simulate.py — Compare Perimeter Spiral vs Lawnmower path plans.
================================================================

Run:
    python simulate.py   (from the path_planner/ directory)

The script:
  1. Prints the convex-pentagon search area geometry and side lengths.
  2. Plans the full inward-spiral (perimeter) waypoint list.
  3. Plans the lawnmower (boustrophedon) waypoint list.
  4. Prints a side-by-side comparison table (waypoints, total distance,
     coverage stats).
  5. Plots both plans side-by-side → mission_preview.png.
  6. Saves perimeter waypoints to waypoints_full.txt.
     Saves lawnmower waypoints to waypoints_lawnmower.txt.
"""

import math
import sys
import os

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# ── Load modules (avoid package-name clash with the folder itself) ─
import importlib.util as _ilu

def _load(name: str):
    spec = _ilu.spec_from_file_location(name, os.path.join(HERE, f"{name}.py"))
    mod  = _ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

_sa  = _load("search_area")
_pp  = _load("perimeter_planner")
_lm  = _load("lawnmower_planner")
_gu  = _load("geo_utils")

# search_area exports
describe_area  = _sa.describe
CORNERS        = _sa.CORNERS
CORNER_NAMES   = _sa.CORNER_NAMES
N_VERTS        = _sa.N_VERTS
CENTER_LAT     = _sa.CENTER_LAT
CENTER_LON     = _sa.CENTER_LON
AREA_M2        = _sa.AREA_M2
SIDE_LENGTHS_M = _sa.SIDE_LENGTHS_M

# perimeter_planner exports
plan_perimeter     = _pp.plan
describe_waypoints = _pp.describe_waypoints
PP_HALF_SWATH      = _pp.HALF_SWATH
PP_SWATH           = _pp.SWATH
PP_ENTER_OFFSET    = _pp.ENTER_OFFSET_M
PP_TAKEOFF_LAT     = _pp.TAKEOFF_LAT
PP_TAKEOFF_LON     = _pp.TAKEOFF_LON
_POLY_XY           = _pp._POLY_XY
_inset_polygon     = _pp._inset_polygon

# lawnmower_planner exports
plan_lawnmower  = _lm.plan
LM_HALF_SWATH   = _lm.HALF_SWATH
LM_SWATH        = _lm.SWATH
LM_HEADING_DEG  = _lm.HEADING_DEG
LM_ENTER_OFFSET = _lm.ENTER_OFFSET_M

# geo_utils exports
haversine_m = _gu.haversine_m
_DEG_LAT_M  = _gu._DEG_LAT_M


# ── Distance helper ────────────────────────────────────────────
def total_distance_m(wps: list) -> float:
    """Sum of haversine distances between consecutive mission waypoints.
    The enter point (index 0) is excluded: counting starts from the
    second navigation waypoint so enter→first-nav transit is not included."""
    total = 0.0
    # Skip segment 0→1 (enter → first nav point); start accumulating at index 2
    for i in range(2, len(wps)):
        total += haversine_m(
            wps[i - 1]["lat"], wps[i - 1]["lon"],
            wps[i]["lat"],     wps[i]["lon"],
        )
    return total


# ───────────────────────────────────────────────────────────────
#  1. Geometry summary
# ───────────────────────────────────────────────────────────────
print(describe_area())
print()

# ───────────────────────────────────────────────────────────────
#  2. Perimeter (inward spiral) — full plan
# ───────────────────────────────────────────────────────────────
print(f"  Perimeter spiral parameters:")
print(f"    HALF_SWATH = {PP_HALF_SWATH} m  |  SWATH = {PP_SWATH} m")
print()

wps_perimeter = plan_perimeter()
dist_perimeter = total_distance_m(wps_perimeter)

print(f"=== Perimeter Spiral: {len(wps_perimeter) - 1} waypoints, "
      f"total distance {dist_perimeter:.1f} m ===")
print(describe_waypoints(wps_perimeter))
print()

with open(os.path.join(HERE, "waypoints_full.txt"), "w") as f:
    f.write("index  name        lat               lon              x_m        y_m\n")
    f.write("-" * 80 + "\n")
    for wp in wps_perimeter:
        f.write(f"{wp['index']:>5}  {wp['name']:<10}  "
                f"{wp['lat']:>16.8f}  {wp['lon']:>16.8f}  "
                f"{wp['x_m']:>9.2f}  {wp['y_m']:>9.2f}\n")
print("  Saved: waypoints_full.txt")
print()

# ───────────────────────────────────────────────────────────────
#  3. Lawnmower (boustrophedon) — full plan
# ───────────────────────────────────────────────────────────────
print(f"  Lawnmower parameters:")
print(f"    HALF_SWATH = {LM_HALF_SWATH} m  |  SWATH = {LM_SWATH} m  "
      f"|  HEADING = {LM_HEADING_DEG}°")
print()

wps_lawnmower = plan_lawnmower()
dist_lawnmower = total_distance_m(wps_lawnmower)

print(f"=== Lawnmower: {len(wps_lawnmower) - 1} waypoints, "
      f"total distance {dist_lawnmower:.1f} m ===")
print(_lm.describe_waypoints(wps_lawnmower))
print()

with open(os.path.join(HERE, "waypoints_lawnmower.txt"), "w") as f:
    f.write("index  name        lat               lon              x_m        y_m\n")
    f.write("-" * 80 + "\n")
    for wp in wps_lawnmower:
        f.write(f"{wp['index']:>5}  {wp['name']:<10}  "
                f"{wp['lat']:>16.8f}  {wp['lon']:>16.8f}  "
                f"{wp['x_m']:>9.2f}  {wp['y_m']:>9.2f}\n")
print("  Saved: waypoints_lawnmower.txt")
print()

# ───────────────────────────────────────────────────────────────
#  4. Comparison table
# ───────────────────────────────────────────────────────────────
print("=" * 58)
print("  ALGORITHM COMPARISON")
print("=" * 58)
print(f"  {'Metric':<28} {'Perimeter Spiral':>12}  {'Lawnmower':>12}")
print(f"  {'-'*28}  {'-'*12}  {'-'*12}")
print(f"  {'Waypoints (excl. enter)':<28} {len(wps_perimeter)-1:>12}  {len(wps_lawnmower)-1:>12}")
print(f"  {'Total distance (m)':<28} {dist_perimeter:>12.1f}  {dist_lawnmower:>12.1f}")
print(f"  {'HALF_SWATH (m)':<28} {PP_HALF_SWATH:>12.1f}  {LM_HALF_SWATH:>12.1f}")
print(f"  {'SWATH / line spacing (m)':<28} {PP_SWATH:>12.1f}  {LM_SWATH:>12.1f}")
print("=" * 58)
print()

# ───────────────────────────────────────────────────────────────
#  5. Visualise — side-by-side comparison
# ───────────────────────────────────────────────────────────────
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("  matplotlib not available — skipping plot.")


def _plot_perimeter(ax, wps: list) -> None:
    """Plot the inward-spiral perimeter plan."""
    # Original boundary
    bx = [v[0] for v in _POLY_XY] + [_POLY_XY[0][0]]
    by = [v[1] for v in _POLY_XY] + [_POLY_XY[0][1]]
    ax.plot(bx, by, "k--", linewidth=1.5, label="Flight Area boundary")

    # Corner labels
    for name, (x, y) in zip(CORNER_NAMES, _POLY_XY):
        ax.text(x, y, f"  {name}", fontsize=7, color="dimgrey",
                va="center", ha="left")

    # Lap-0 inset polygon (first spiral ring at HALF_SWATH inside)
    p0 = _inset_polygon(_POLY_XY, PP_HALF_SWATH)
    if p0:
        px = [v[0] for v in p0] + [p0[0][0]]
        py = [v[1] for v in p0] + [p0[0][1]]
        ax.plot(px, py, color="cyan", linewidth=0.8, linestyle=":",
                alpha=0.7, label=f"Lap-0 inset ({PP_HALF_SWATH} m)")

    if wps:
        xs = [wp["x_m"] for wp in wps]
        ys = [wp["y_m"] for wp in wps]
        ax.plot(xs, ys, "b-", linewidth=0.9, alpha=0.75, label="UAV path")
        for wp in wps:
            colour = "red"        if wp["name"] == "centre" else \
                     "darkorange" if wp["name"] == "enter"  else "dodgerblue"
            ms     = 9  if wp["name"] in ("centre", "enter") else 4
            ax.plot(wp["x_m"], wp["y_m"], "o", color=colour, markersize=ms)
            ax.annotate(wp["name"], (wp["x_m"], wp["y_m"]),
                        textcoords="offset points", xytext=(3, 3),
                        fontsize=5, color="navy")

    # Also plot takeoff (reference only, not in mission)
    to_x, to_y = _pp._ll2xy(PP_TAKEOFF_LAT, PP_TAKEOFF_LON)
    ax.plot(to_x, to_y, "^", color="gold", markersize=8, label="Takeoff (ref)")
    ax.annotate("takeoff", (to_x, to_y), textcoords="offset points",
                xytext=(3, 3), fontsize=5, color="goldenrod")

    ax.plot(0, 0, "r*", markersize=12, label="Centroid")
    ax.set_title(
        f"Perimeter Spiral  —  {len(wps) - 1} waypoints\n"
        f"Total distance: {dist_perimeter:.1f} m   "
        f"(HALF_SWATH = {PP_HALF_SWATH} m)",
        fontsize=9,
    )
    ax.set_xlabel("East (m from centroid)")
    ax.set_ylabel("North (m from centroid)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.5)
    ax.legend(fontsize=6, loc="lower right")


def _plot_lawnmower(ax, wps: list) -> None:
    """Plot the lawnmower (boustrophedon) plan."""
    # Original boundary
    bx = [v[0] for v in _POLY_XY] + [_POLY_XY[0][0]]
    by = [v[1] for v in _POLY_XY] + [_POLY_XY[0][1]]
    ax.plot(bx, by, "k--", linewidth=1.5, label="Flight Area boundary")

    # Corner labels
    for name, (x, y) in zip(CORNER_NAMES, _POLY_XY):
        ax.text(x, y, f"  {name}", fontsize=7, color="dimgrey",
                va="center", ha="left")

    if wps:
        xs = [wp["x_m"] for wp in wps]
        ys = [wp["y_m"] for wp in wps]
        ax.plot(xs, ys, "g-", linewidth=0.9, alpha=0.75, label="UAV path")
        for wp in wps:
            colour = "darkorange" if wp["name"] == "enter"   else "limegreen"
            ms     = 7            if wp["name"] == "enter"   else 4
            ax.plot(wp["x_m"], wp["y_m"], "o", color=colour, markersize=ms)
            ax.annotate(wp["name"], (wp["x_m"], wp["y_m"]),
                        textcoords="offset points", xytext=(3, 3),
                        fontsize=5, color="darkgreen")

    # Also plot takeoff (reference only, not in mission)
    to_x, to_y = _pp._ll2xy(PP_TAKEOFF_LAT, PP_TAKEOFF_LON)
    ax.plot(to_x, to_y, "^", color="gold", markersize=8, label="Takeoff (ref)")
    ax.annotate("takeoff", (to_x, to_y), textcoords="offset points",
                xytext=(3, 3), fontsize=5, color="goldenrod")

    ax.plot(0, 0, "r*", markersize=12, label="Centroid")
    ax.set_title(
        f"Lawnmower (Boustrophedon)  —  {len(wps) - 1} waypoints\n"
        f"Total distance: {dist_lawnmower:.1f} m   "
        f"(HALF_SWATH = {LM_HALF_SWATH} m, heading = {LM_HEADING_DEG}°)",
        fontsize=9,
    )
    ax.set_xlabel("East (m from centroid)")
    ax.set_ylabel("North (m from centroid)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.5)
    ax.legend(fontsize=6, loc="lower right")


if HAS_MPL:
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    side_str = "  |  ".join(
        f"{CORNER_NAMES[i]}→{CORNER_NAMES[(i+1) % N_VERTS]}: {SIDE_LENGTHS_M[i]:.1f} m"
        for i in range(N_VERTS)
    )
    fig.suptitle(
        "Path Planning Comparison — Fenswood Farm  "
        "(KML Survey Area, convex pentagon)\n"
        f"{side_str}\n"
        f"Area: {AREA_M2:.0f} m²   SWATH: {PP_SWATH} m   "
        f"HALF_SWATH: {PP_HALF_SWATH} m",
        fontsize=8,
    )

    _plot_perimeter(axes[0], wps_perimeter)
    _plot_lawnmower(axes[1], wps_lawnmower)

    out_path = os.path.join(HERE, "mission_preview.png")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"  Saved plot: {out_path}")
    print()

print("Done.")
