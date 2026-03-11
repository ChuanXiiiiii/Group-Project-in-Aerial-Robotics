"""
preview.py — Render a pattern preview onto the Fenswood map tile.
==================================================================
Plots waypoints and polygon onto the geo-referenced background
image.  Flags any out-of-bounds waypoints with a red warning.

Public API
----------
    render_preview(polygon, waypoints, output_path) → dict
        Returns {"path": str, "wp_count": int,
                 "oob_count": int, "oob_indices": list}
"""

import os
import math

import matplotlib
matplotlib.use('Agg')                     # headless — no display needed
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as mpatches

import config


def render_preview(polygon, waypoints, output_path=None):
    """
    Render the search pattern preview image.

    Parameters
    ----------
    polygon : list of (lat, lon)
        Search area vertices.
    waypoints : list of dict
        [{"lat", "lon", "alt"}, ...] from pattern_generator.
    output_path : str or None
        Where to save the PNG.  Defaults to config.PATTERN_PREVIEW_PATH.

    Returns
    -------
    dict with keys: path, wp_count, oob_count, oob_indices
    """
    if output_path is None:
        output_path = config.PATTERN_PREVIEW_PATH

    # ── Map tile bounds ──────────────────────────────────────
    N = config.MAP_BOUNDS_NORTH
    S = config.MAP_BOUNDS_SOUTH
    W = config.MAP_BOUNDS_WEST
    E = config.MAP_BOUNDS_EAST

    # ── Check which waypoints fall outside tile bounds ───────
    oob_indices = []
    for i, wp in enumerate(waypoints):
        if wp["lat"] < S or wp["lat"] > N or wp["lon"] < W or wp["lon"] > E:
            oob_indices.append(i)

    # ── Create figure ────────────────────────────────────────
    fig, ax = plt.subplots(1, 1, figsize=(10, 10), dpi=128)

    # Load background tile if it exists
    tile_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        config.MAP_TILE_PATH
    )
    if os.path.exists(tile_path):
        bg = mpimg.imread(tile_path)
        ax.imshow(bg, extent=[W, E, S, N], aspect='auto', alpha=0.85)

    ax.set_xlim(W, E)
    ax.set_ylim(S, N)

    # ── Draw search polygon ──────────────────────────────────
    poly_lons = [p[1] for p in polygon] + [polygon[0][1]]
    poly_lats = [p[0] for p in polygon] + [polygon[0][0]]
    ax.fill(poly_lons, poly_lats, color='#0066cc', alpha=0.10)
    ax.plot(poly_lons, poly_lats, color='#0066cc', linewidth=2.0,
            label='Search Area')

    # ── Draw waypoint path ───────────────────────────────────
    wp_lons = [wp["lon"] for wp in waypoints]
    wp_lats = [wp["lat"] for wp in waypoints]

    # Connecting lines (flight path)
    ax.plot(wp_lons, wp_lats, color='#ff8800', linewidth=1.0,
            alpha=0.7, zorder=3)

    # In-bounds waypoints
    ib_lons = [wp_lons[i] for i in range(len(waypoints)) if i not in oob_indices]
    ib_lats = [wp_lats[i] for i in range(len(waypoints)) if i not in oob_indices]
    ax.scatter(ib_lons, ib_lats, c='#ff8800', s=15, zorder=4,
               edgecolors='white', linewidths=0.5, label='Waypoints')

    # Out-of-bounds waypoints — big red markers
    if oob_indices:
        ob_lons = [wp_lons[i] for i in oob_indices]
        ob_lats = [wp_lats[i] for i in oob_indices]
        ax.scatter(ob_lons, ob_lats, c='red', s=60, zorder=5,
                   marker='X', linewidths=1.5,
                   label=f'OUT OF BOUNDS ({len(oob_indices)})')

    # ── Start / end markers ──────────────────────────────────
    if waypoints:
        ax.scatter([wp_lons[0]], [wp_lats[0]], c='#00cc00', s=80,
                   marker='o', zorder=6, edgecolors='white',
                   linewidths=1.5, label='Start')
        ax.scatter([wp_lons[-1]], [wp_lats[-1]], c='#cc0000', s=80,
                   marker='s', zorder=6, edgecolors='white',
                   linewidths=1.5, label='End')

    # ── Warning banner ───────────────────────────────────────
    if oob_indices:
        ax.text(0.5, 0.97,
                f"⚠  {len(oob_indices)} WAYPOINT(S) OUTSIDE MAP BOUNDS  ⚠",
                transform=ax.transAxes, fontsize=14, fontweight='bold',
                color='white', ha='center', va='top',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='red',
                          alpha=0.9),
                zorder=10)

    # ── Info box ─────────────────────────────────────────────
    info_lines = [
        f"Waypoints: {len(waypoints)}",
        f"Altitude: {waypoints[0]['alt']:.0f} m AGL" if waypoints else "",
    ]
    # Estimate total distance
    if len(waypoints) >= 2:
        total_m = 0
        for i in range(len(waypoints) - 1):
            dlat = (waypoints[i+1]["lat"] - waypoints[i]["lat"]) * 111_320
            dlon = (waypoints[i+1]["lon"] - waypoints[i]["lon"]) * \
                   111_320 * math.cos(math.radians(waypoints[i]["lat"]))
            total_m += math.sqrt(dlat**2 + dlon**2)
        info_lines.append(f"Path length: {total_m:.0f} m")

    info_text = "\n".join(ln for ln in info_lines if ln)
    ax.text(0.02, 0.02, info_text,
            transform=ax.transAxes, fontsize=9,
            verticalalignment='bottom',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                      alpha=0.85),
            zorder=10)

    # ── Legend & labels ──────────────────────────────────────
    ax.legend(loc='upper left', fontsize=8, framealpha=0.85)
    ax.set_xlabel('Longitude', fontsize=9)
    ax.set_ylabel('Latitude', fontsize=9)
    ax.set_title('Search Pattern Preview — confirm before upload',
                 fontsize=11, fontweight='bold')
    ax.tick_params(labelsize=7)
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(output_path, dpi=128, bbox_inches='tight', pad_inches=0.05)
    plt.close()

    return {
        "path": output_path,
        "wp_count": len(waypoints),
        "oob_count": len(oob_indices),
        "oob_indices": oob_indices,
    }
