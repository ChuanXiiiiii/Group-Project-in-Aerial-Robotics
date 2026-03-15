"""
Simulate and visualize UAV coverage path planning results.

This script generates a synthetic experiment dataset for two algorithms:
- spiral      (Perimeter Spiral)
- lawnmower   (Boustrophedon)

Spacing (SWATH) is swept from 4.0 m to 25.0 m with step 0.1 m.
For each spacing and algorithm, the script generates:
- total_distance (m)
- waypoints (count)

Outputs:
- outputs/swath_continuous_comparison/swath_experiment_results.csv
- outputs/swath_continuous_comparison/distance_vs_spacing.png
- outputs/swath_continuous_comparison/waypoints_vs_spacing.png
- outputs/swath_continuous_comparison/pareto_distance_waypoints.png
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "outputs" / "swath_continuous_comparison"
CSV_PATH = OUTPUT_DIR / "swath_experiment_results.csv"

DISTANCE_FIG = OUTPUT_DIR / "distance_vs_spacing.png"
WAYPOINTS_FIG = OUTPUT_DIR / "waypoints_vs_spacing.png"
PARETO_FIG = OUTPUT_DIR / "pareto_distance_waypoints.png"

PLOT_STEP_M = 0.1


def generate_dataset() -> pd.DataFrame:
    """Generate synthetic experimental results with realistic trends."""
    rng = np.random.default_rng(42)

    spacing = np.round(np.arange(4.0, 25.0 + 1e-9, 0.1), 1)

    # Decreasing trend + light stochastic variation.
    # Spiral is generally shorter and has fewer waypoints than lawnmower.
    spiral_dist = 620.0 + 4300.0 / spacing + rng.normal(0.0, 18.0, size=spacing.size)
    lawn_dist = 760.0 + 5100.0 / spacing + rng.normal(0.0, 20.0, size=spacing.size)

    spiral_wp = 7.0 + 190.0 / spacing + rng.normal(0.0, 1.7, size=spacing.size)
    lawn_wp = 10.0 + 235.0 / spacing + rng.normal(0.0, 2.0, size=spacing.size)

    spiral_dist = np.clip(spiral_dist, 200.0, None)
    lawn_dist = np.clip(lawn_dist, 220.0, None)
    spiral_wp = np.clip(np.rint(spiral_wp), 3, None).astype(int)
    lawn_wp = np.clip(np.rint(lawn_wp), 4, None).astype(int)

    df_spiral = pd.DataFrame(
        {
            "spacing": spacing,
            "algorithm": "spiral",
            "total_distance": spiral_dist,
            "waypoints": spiral_wp,
        }
    )

    df_lawn = pd.DataFrame(
        {
            "spacing": spacing,
            "algorithm": "lawnmower",
            "total_distance": lawn_dist,
            "waypoints": lawn_wp,
        }
    )

    return pd.concat([df_spiral, df_lawn], ignore_index=True)


def sampled_for_plot(df: pd.DataFrame) -> pd.DataFrame:
    """Sample spacing for plotting only; 0.1 m keeps all generated points."""
    spacing_decimeter = np.rint(df["spacing"].to_numpy() * 10).astype(int)
    step_decimeter = max(1, int(round(PLOT_STEP_M * 10)))
    keep = spacing_decimeter % step_decimeter == 0
    return df.loc[keep].copy()


def plot_distance_vs_spacing(full_df: pd.DataFrame) -> None:
    df_plot = sampled_for_plot(full_df)

    spiral_full = full_df[full_df["algorithm"] == "spiral"].reset_index(drop=True)
    lawn_full = full_df[full_df["algorithm"] == "lawnmower"].reset_index(drop=True)

    spiral_plot = df_plot[df_plot["algorithm"] == "spiral"]
    lawn_plot = df_plot[df_plot["algorithm"] == "lawnmower"]

    # Full-data optimum (requested to use full 0.1 m sweep).
    spiral_opt_idx = spiral_full["total_distance"].idxmin()
    lawn_opt_idx = lawn_full["total_distance"].idxmin()

    spiral_opt = spiral_full.loc[spiral_opt_idx]
    lawn_opt = lawn_full.loc[lawn_opt_idx]

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(
        spiral_plot["spacing"],
        spiral_plot["total_distance"],
        color="tab:blue",
        linewidth=2.5,
        label="Perimeter Spiral",
    )
    ax.plot(
        lawn_plot["spacing"],
        lawn_plot["total_distance"],
        color="tab:orange",
        linewidth=2.5,
        label="Lawnmower",
    )

    ax.scatter(
        [spiral_opt["spacing"]],
        [spiral_opt["total_distance"]],
        color="tab:blue",
        s=40,
        zorder=5,
    )
    ax.scatter(
        [lawn_opt["spacing"]],
        [lawn_opt["total_distance"]],
        color="tab:orange",
        s=40,
        zorder=5,
    )

    ax.annotate(
        f"Optimal spacing = {spiral_opt['spacing']:.1f} m",
        xy=(spiral_opt["spacing"], spiral_opt["total_distance"]),
        xytext=(8, -16),
        textcoords="offset points",
        color="tab:blue",
        fontsize=11,
    )
    ax.annotate(
        f"Optimal spacing = {lawn_opt['spacing']:.1f} m",
        xy=(lawn_opt["spacing"], lawn_opt["total_distance"]),
        xytext=(8, 8),
        textcoords="offset points",
        color="tab:orange",
        fontsize=11,
    )

    ax.set_xlabel("Spacing / SWATH (m)")
    ax.set_ylabel("Total Path Distance (m)")
    ax.grid(True, alpha=0.35)
    ax.legend(frameon=False)

    fig.tight_layout()
    fig.savefig(DISTANCE_FIG, dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_waypoints_vs_spacing(full_df: pd.DataFrame) -> None:
    df_plot = sampled_for_plot(full_df)

    spiral_full = full_df[full_df["algorithm"] == "spiral"].reset_index(drop=True)
    lawn_full = full_df[full_df["algorithm"] == "lawnmower"].reset_index(drop=True)

    spiral_plot = df_plot[df_plot["algorithm"] == "spiral"]
    lawn_plot = df_plot[df_plot["algorithm"] == "lawnmower"]

    spiral_opt_idx = spiral_full["waypoints"].idxmin()
    lawn_opt_idx = lawn_full["waypoints"].idxmin()

    spiral_opt = spiral_full.loc[spiral_opt_idx]
    lawn_opt = lawn_full.loc[lawn_opt_idx]

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(
        spiral_plot["spacing"],
        spiral_plot["waypoints"],
        color="tab:blue",
        linewidth=2.5,
        label="Perimeter Spiral",
    )
    ax.plot(
        lawn_plot["spacing"],
        lawn_plot["waypoints"],
        color="tab:orange",
        linewidth=2.5,
        label="Lawnmower",
    )

    ax.scatter(
        [spiral_opt["spacing"]],
        [spiral_opt["waypoints"]],
        color="tab:blue",
        s=40,
        zorder=5,
    )
    ax.scatter(
        [lawn_opt["spacing"]],
        [lawn_opt["waypoints"]],
        color="tab:orange",
        s=40,
        zorder=5,
    )

    ax.annotate(
        f"Optimal spacing = {spiral_opt['spacing']:.1f} m",
        xy=(spiral_opt["spacing"], spiral_opt["waypoints"]),
        xytext=(8, -16),
        textcoords="offset points",
        color="tab:blue",
        fontsize=11,
    )
    ax.annotate(
        f"Optimal spacing = {lawn_opt['spacing']:.1f} m",
        xy=(lawn_opt["spacing"], lawn_opt["waypoints"]),
        xytext=(8, 8),
        textcoords="offset points",
        color="tab:orange",
        fontsize=11,
    )

    ax.set_xlabel("Spacing / SWATH (m)")
    ax.set_ylabel("Number of Waypoints")
    ax.grid(True, alpha=0.35)
    ax.legend(frameon=False)

    fig.tight_layout()
    fig.savefig(WAYPOINTS_FIG, dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_pareto(full_df: pd.DataFrame) -> None:
    df_plot = sampled_for_plot(full_df)

    spiral = df_plot[df_plot["algorithm"] == "spiral"].sort_values("spacing")
    lawn = df_plot[df_plot["algorithm"] == "lawnmower"].sort_values("spacing")

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(
        spiral["waypoints"],
        spiral["total_distance"],
        color="tab:blue",
        linewidth=2.5,
        label="Perimeter Spiral",
    )
    ax.scatter(
        spiral["waypoints"],
        spiral["total_distance"],
        color="tab:blue",
        s=18,
        alpha=0.75,
    )

    ax.plot(
        lawn["waypoints"],
        lawn["total_distance"],
        color="tab:orange",
        linewidth=2.5,
        label="Lawnmower",
    )
    ax.scatter(
        lawn["waypoints"],
        lawn["total_distance"],
        color="tab:orange",
        s=18,
        alpha=0.75,
    )

    ax.set_xlabel("Waypoints")
    ax.set_ylabel("Total Path Distance (m)")
    ax.grid(True, alpha=0.35)
    ax.legend(frameon=False)

    fig.tight_layout()
    fig.savefig(PARETO_FIG, dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    plt.rcParams.update(
        {
            "font.size": 12,
            "axes.labelsize": 13,
            "axes.titlesize": 14,
            "legend.fontsize": 11,
        }
    )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # 1) Auto-generate synthetic experiment data and save as CSV.
    generated_df = generate_dataset()
    generated_df.to_csv(CSV_PATH, index=False)

    # 2) Read with pandas for analysis/plotting (as requested).
    df = pd.read_csv(CSV_PATH)

    plot_distance_vs_spacing(df)
    plot_waypoints_vs_spacing(df)
    plot_pareto(df)

    print(f"Saved CSV: {CSV_PATH}")
    print(f"Saved figure: {DISTANCE_FIG}")
    print(f"Saved figure: {WAYPOINTS_FIG}")
    print(f"Saved figure: {PARETO_FIG}")


if __name__ == "__main__":
    main()
