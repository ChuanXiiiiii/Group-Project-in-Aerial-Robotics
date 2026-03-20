"""
simulate_plb.py - Simulate PLB-area search missions on a laptop.

Run examples (from repository root):
    .venv/bin/python path_planner/simulate_plb.py --n 6 --mode 0 --seed 42
    .venv/bin/python path_planner/simulate_plb.py --n 8 --mode 1 --seed 7 --show

mode:
    0 = lawnmower
    1 = perimeter spiral
"""

from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import PLB_searching as plb
import search_area as sa
from geo_utils import haversine_m


def _algorithm_name(mode: int) -> str:
    return "lawnmower" if mode == 0 else "perimeter_spiral"


def _total_distance_m(waypoints: list[dict]) -> float:
    if len(waypoints) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(waypoints)):
        a = waypoints[i - 1]
        b = waypoints[i]
        total += haversine_m(a["lat"], a["lon"], b["lat"], b["lon"])
    return total


def _save_waypoints_txt(path: str, waypoints: list[dict]) -> None:
    with open(path, "w") as f:
        f.write("index  name         lat               lon              x_m        y_m\n")
        f.write("-" * 82 + "\n")
        for wp in waypoints:
            f.write(
                f"{wp['index']:>5}  {wp['name']:<11}  "
                f"{wp['lat']:>16.8f}  {wp['lon']:>16.8f}  "
                f"{wp['x_m']:>9.2f}  {wp['y_m']:>9.2f}\n"
            )


def _save_polygon_txt(path: str, polygon: list[dict]) -> None:
    with open(path, "w") as f:
        f.write("name      lat               lon              x_m        y_m\n")
        f.write("-" * 72 + "\n")
        for p in polygon:
            f.write(
                f"{p['name']:<8}  {p['lat']:>16.8f}  {p['lon']:>16.8f}  "
                f"{p['x_m']:>9.2f}  {p['y_m']:>9.2f}\n"
            )


def _plot_mission(
    mission: dict,
    png_path: str,
    show: bool,
) -> None:
    try:
        import matplotlib

        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skip plotting.")
        return

    fig, ax = plt.subplots(figsize=(9, 7))

    # Search-area boundary (local XY from centroid).
    search_xy = [plb._ll2xy(lat, lon) for lat, lon in sa.CORNERS]
    sx = [p[0] for p in search_xy] + [search_xy[0][0]]
    sy = [p[1] for p in search_xy] + [search_xy[0][1]]
    ax.plot(sx, sy, "k--", linewidth=1.3, label="Search area")

    # PLB polygon.
    plb_xy = [(p["x_m"], p["y_m"]) for p in mission["plb_polygon"]]
    px = [p[0] for p in plb_xy] + [plb_xy[0][0]]
    py = [p[1] for p in plb_xy] + [plb_xy[0][1]]
    ax.fill(px, py, color="#c8f7c5", alpha=0.35, label="PLB region")
    ax.plot(px, py, color="#2e8b57", linewidth=1.5)

    # Mission path.
    waypoints = mission["waypoints"]
    xs = [w["x_m"] for w in waypoints]
    ys = [w["y_m"] for w in waypoints]
    if len(xs) >= 2:
        ax.plot(xs[:2], ys[:2], linestyle="--", color="#1f77b4", linewidth=1.2, label="Transit")
    if len(xs) >= 3:
        ax.plot(xs[1:], ys[1:], linestyle="-", color="#d62728", linewidth=1.2, label="Search path")

    for wp in waypoints:
        if wp["name"] == "start":
            color = "#1565c0"
            size = 55
        elif wp["name"] == "plb_entry":
            color = "#ef6c00"
            size = 42
        else:
            color = "#c62828"
            size = 20
        ax.scatter([wp["x_m"]], [wp["y_m"]], c=color, s=size, zorder=3)

    ax.set_title(
        f"PLB Search Simulation | n={mission['n']} | mode={mission['algorithm_mode']} "
        f"({_algorithm_name(mission['algorithm_mode'])})"
    )
    ax.set_xlabel("East (m from search-area centroid)")
    ax.set_ylabel("North (m from search-area centroid)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.45)
    ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(png_path, dpi=220, bbox_inches="tight")
    print(f"Saved plot: {png_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Simulate PLB area search mission.")
    parser.add_argument("--n", type=int, default=6, help="PLB vertex count, 3..10")
    parser.add_argument("--mode", type=int, default=0, choices=[0, 1], help="0=lawnmower, 1=perimeter spiral")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility")
    parser.add_argument("--show", action="store_true", help="Show matplotlib window")
    args = parser.parse_args()

    mission = plb.plan_plb_search(n=args.n, seed=args.seed, algorithm_mode=args.mode)

    out_dir = os.path.join(HERE, "outputs", "plb")
    os.makedirs(out_dir, exist_ok=True)

    suffix = f"n{args.n}_mode{args.mode}_seed{args.seed}"
    waypoints_path = os.path.join(out_dir, f"waypoints_{suffix}.txt")
    polygon_path = os.path.join(out_dir, f"plb_polygon_{suffix}.txt")
    figure_path = os.path.join(out_dir, f"simulation_{suffix}.png")

    _save_waypoints_txt(waypoints_path, mission["waypoints"])
    _save_polygon_txt(polygon_path, mission["plb_polygon"])
    _plot_mission(mission, figure_path, show=args.show)

    total_distance = _total_distance_m(mission["waypoints"])
    print()
    print("=== PLB Simulation Summary ===")
    print(f"algorithm_mode: {mission['algorithm_mode']} ({_algorithm_name(mission['algorithm_mode'])})")
    print(f"n_vertices    : {mission['n']}")
    print(f"seed          : {args.seed}")
    print(f"waypoint_count: {len(mission['waypoints'])}")
    print(f"path_distance : {total_distance:.1f} m")
    print(f"Saved waypoints: {waypoints_path}")
    print(f"Saved polygon  : {polygon_path}")
    print()
    print(plb.describe_waypoints(mission["waypoints"]))


if __name__ == "__main__":
    main()
