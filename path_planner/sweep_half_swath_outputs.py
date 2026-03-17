"""
Batch test generator for HALF_SWATH sweep.

Sweeps HALF_SWATH from 2.0 to 12.5 (step 0.5) for both planners,
and writes outputs for each case into:

    outputs/<SWATH value>/

Each folder contains:
    - waypoints_spiral.txt
    - waypoints_lawnmower.txt
    - comparison.txt
    - mission_preview.png
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

import perimeter_planner as pp
import lawnmower_planner as lm
import search_area as sa
from geo_utils import haversine_m


OUT_ROOT = HERE / "outputs"
HALF_START = 2.0
HALF_END = 12.5
HALF_STEP = 0.5


def total_distance_m(wps: list[dict]) -> float:
    """Match simulate.py: skip enter->first-nav segment."""
    if len(wps) < 3:
        return 0.0
    total = 0.0
    for i in range(2, len(wps)):
        total += haversine_m(
            wps[i - 1]["lat"],
            wps[i - 1]["lon"],
            wps[i]["lat"],
            wps[i]["lon"],
        )
    return total


def write_waypoints_txt(path: Path, wps: list[dict]) -> None:
    lines: list[str] = []
    lines.append("index  name        lat               lon              x_m        y_m")
    lines.append("-" * 80)
    for wp in wps:
        lines.append(
            f"{wp['index']:>5}  {wp['name']:<10}  "
            f"{wp['lat']:>16.8f}  {wp['lon']:>16.8f}  "
            f"{wp['x_m']:>9.2f}  {wp['y_m']:>9.2f}"
        )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_comparison_txt(
    path: Path,
    half_swath: float,
    swath: float,
    wps_perimeter: list[dict],
    wps_lawnmower: list[dict],
    dist_perimeter: float,
    dist_lawnmower: float,
) -> None:
    lines = [
        "=" * 58,
        "  ALGORITHM COMPARISON",
        "=" * 58,
        f"  {'Metric':<28} {'Perimeter Spiral':>12}  {'Lawnmower':>12}",
        f"  {'-'*28}  {'-'*12}  {'-'*12}",
        f"  {'Waypoints (excl. enter)':<28} {len(wps_perimeter)-1:>12}  {len(wps_lawnmower)-1:>12}",
        f"  {'Total distance (m)':<28} {dist_perimeter:>12.1f}  {dist_lawnmower:>12.1f}",
        f"  {'HALF_SWATH (m)':<28} {half_swath:>12.1f}  {half_swath:>12.1f}",
        f"  {'SWATH / line spacing (m)':<28} {swath:>12.1f}  {swath:>12.1f}",
        "=" * 58,
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_preview(
    path: Path,
    half_swath: float,
    swath: float,
    wps_perimeter: list[dict],
    wps_lawnmower: list[dict],
    dist_perimeter: float,
    dist_lawnmower: float,
) -> None:
    poly = pp._POLY_XY
    bx = [v[0] for v in poly] + [poly[0][0]]
    by = [v[1] for v in poly] + [poly[0][1]]

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # Perimeter panel
    ax = axes[0]
    ax.plot(bx, by, "k--", linewidth=1.3, label="Boundary")
    if wps_perimeter:
        xs = [w["x_m"] for w in wps_perimeter]
        ys = [w["y_m"] for w in wps_perimeter]
        # Draw the first leg (enter -> first mission point) as dashed.
        if len(xs) >= 2:
            ax.plot(xs[:2], ys[:2], "b--", linewidth=1.1, alpha=0.9,
                    label="Enter -> first WP")
        if len(xs) >= 3:
            ax.plot(xs[1:], ys[1:], "b-", linewidth=0.9, alpha=0.8,
                    label="Path")
        ax.scatter(xs, ys, s=10, c="dodgerblue")
    ax.set_title(
        f"Perimeter Spiral\\n{len(wps_perimeter)-1} WPs, dist={dist_perimeter:.1f} m",
        fontsize=10,
    )
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.5)
    ax.legend(fontsize=8, loc="lower right")

    # Lawnmower panel
    ax = axes[1]
    ax.plot(bx, by, "k--", linewidth=1.3, label="Boundary")
    if wps_lawnmower:
        xs = [w["x_m"] for w in wps_lawnmower]
        ys = [w["y_m"] for w in wps_lawnmower]
        # Draw the first leg (enter -> WP0) as dashed, remaining legs solid.
        if len(xs) >= 2:
            ax.plot(xs[:2], ys[:2], linestyle="--", color="orange", linewidth=1.1, alpha=0.9,
                    label="Enter -> WP0")
        if len(xs) >= 3:
            ax.plot(xs[1:], ys[1:], linestyle="-", color="orange", linewidth=0.9, alpha=0.8,
                    label="Path")
        ax.scatter(xs, ys, s=10, c="orange")
    ax.set_title(
        f"Lawnmower\\n{len(wps_lawnmower)-1} WPs, dist={dist_lawnmower:.1f} m",
        fontsize=10,
    )
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.5)
    ax.legend(fontsize=8, loc="lower right")

    fig.suptitle(
        "Path Planning Comparison — Fenswood Farm\n"
        f"HALF_SWATH={half_swath:.1f} m, SWATH={swath:.1f} m",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def run_sweep() -> None:
    OUT_ROOT.mkdir(parents=True, exist_ok=True)

    n_steps = int(round((HALF_END - HALF_START) / HALF_STEP))
    for i in range(n_steps + 1):
        half = round(HALF_START + i * HALF_STEP, 1)
        swath = round(2.0 * half, 1)

        # Keep both planners synchronized.
        pp.HALF_SWATH = half
        pp.SWATH = swath
        lm.HALF_SWATH = half
        lm.SWATH = swath

        wps_perimeter = pp.plan()
        wps_lawnmower = lm.plan()
        dist_perimeter = total_distance_m(wps_perimeter)
        dist_lawnmower = total_distance_m(wps_lawnmower)

        case_dir = OUT_ROOT / f"{swath:.1f}"
        case_dir.mkdir(parents=True, exist_ok=True)

        # New split waypoint files.
        write_waypoints_txt(case_dir / "waypoints_spiral.txt", wps_perimeter)
        write_waypoints_txt(case_dir / "waypoints_lawnmower.txt", wps_lawnmower)
        # Remove legacy combined output if present.
        legacy = case_dir / "waypoint.txt"
        if legacy.exists():
            legacy.unlink()
        write_comparison_txt(
            case_dir / "comparison.txt",
            half,
            swath,
            wps_perimeter,
            wps_lawnmower,
            dist_perimeter,
            dist_lawnmower,
        )
        plot_preview(
            case_dir / "mission_preview.png",
            half,
            swath,
            wps_perimeter,
            wps_lawnmower,
            dist_perimeter,
            dist_lawnmower,
        )

        print(
            f"[OK] HALF_SWATH={half:.1f}, SWATH={swath:.1f} -> {case_dir.relative_to(ROOT)}"
        )

    print(f"\\nDone. Outputs saved under: {OUT_ROOT}")


if __name__ == "__main__":
    run_sweep()
