"""
preview.py — Render a pattern preview onto a satellite background.
==================================================================
Two rendering modes controlled by the `dual_pane` argument:

    dual_pane=False  (default — initial search pattern)
        Single plot zoomed to the polygon + waypoints with 15%
        padding.  The satellite image is shown as background.

    dual_pane=True   (PLB focus pattern)
        Two side-by-side subplots:
          Left  — full satellite extent with just the focus polygon
                  drawn on it, plus a dashed zoom-box rectangle
                  showing exactly what the right panel covers.
          Right — tight zoom around the polygon + waypoints showing
                  the full lawnmower detail.
        This mirrors the "inset zoom" style used in presentations
        so the operator can see both WHERE the pattern is and WHAT
        the pattern looks like.

Public API
----------
    render_preview(polygon, waypoints, output_path, dual_pane) → dict
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
import matplotlib.ticker as mticker

import config


# ── Shared helpers ───────────────────────────────────────────────

def _load_background():
    """
    Return (img_array, extent) for the best available background image.
    extent is [west, east, south, north] for matplotlib imshow.
    Returns (None, None) if no image is found.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))

    sat_path  = os.path.join(script_dir, config.SATELLITE_IMAGE)
    tile_path = os.path.join(script_dir, config.MAP_TILE_PATH)

    if os.path.exists(sat_path):
        return mpimg.imread(sat_path), [
            config.SATELLITE_BOUNDS_WEST,
            config.SATELLITE_BOUNDS_EAST,
            config.SATELLITE_BOUNDS_SOUTH,
            config.SATELLITE_BOUNDS_NORTH,
        ]
    if os.path.exists(tile_path):
        return mpimg.imread(tile_path), [
            config.MAP_BOUNDS_WEST,
            config.MAP_BOUNDS_EAST,
            config.MAP_BOUNDS_SOUTH,
            config.MAP_BOUNDS_NORTH,
        ]
    return None, None


def _oob_indices(waypoints):
    """Return indices of waypoints outside the flight-area map bounds."""
    N = config.MAP_BOUNDS_NORTH
    S = config.MAP_BOUNDS_SOUTH
    W = config.MAP_BOUNDS_WEST
    E = config.MAP_BOUNDS_EAST
    return [
        i for i, wp in enumerate(waypoints)
        if wp["lat"] < S or wp["lat"] > N or wp["lon"] < W or wp["lon"] > E
    ]


def _content_bounds(polygon, waypoints, pad_frac=0.15):
    """Compute view bounds that fit all polygon vertices + waypoints."""
    all_lats = [p[0] for p in polygon] + [wp["lat"] for wp in waypoints]
    all_lons = [p[1] for p in polygon] + [wp["lon"] for wp in waypoints]

    min_lat, max_lat = min(all_lats), max(all_lats)
    min_lon, max_lon = min(all_lons), max(all_lons)

    pad_lat = (max_lat - min_lat) * pad_frac or 0.0003
    pad_lon = (max_lon - min_lon) * pad_frac or 0.0003

    return (min_lat - pad_lat, max_lat + pad_lat,
            min_lon - pad_lon, max_lon + pad_lon)


def _style_axes(ax, dark=False):
    """
    Apply clean formatting to a geo-coordinate axes:
      - Disable scientific notation / offset on both axes
      - Format ticks as plain decimal degrees (6 dp lat, 5 dp lon)
      - Use geographic aspect ratio (cos-corrected, not 'equal')
      - Dark theme for dual-pane panels
    """
    # Full decimal coordinates — no scientific notation
    ax.ticklabel_format(useOffset=False, style='plain')
    ax.xaxis.set_major_formatter(mticker.FormatStrFormatter('%.5f'))
    ax.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.6f'))
    ax.tick_params(labelsize=7)

    # Geographic aspect: at latitude ~51.4°N, 1° lon ≈ 0.625 × 1° lat
    # in physical distance.  'equal' forces them 1:1 in pixel space,
    # wasting vertical space.  We set the correct cos-corrected ratio.
    ax.set_aspect(1.0 / math.cos(math.radians(51.423)))

    if dark:
        ax.tick_params(colors='white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        for spine in ax.spines.values():
            spine.set_edgecolor('#555')


def _draw_pattern(ax, polygon, waypoints, oob):
    """Draw the polygon + waypoint path onto an existing axes object."""
    poly_lons = [p[1] for p in polygon] + [polygon[0][1]]
    poly_lats = [p[0] for p in polygon] + [polygon[0][0]]
    ax.fill(poly_lons, poly_lats, color='#0066cc', alpha=0.10)
    ax.plot(poly_lons, poly_lats, color='#0066cc', linewidth=2.0,
            label='Focus Area')

    wp_lons = [wp["lon"] for wp in waypoints]
    wp_lats = [wp["lat"] for wp in waypoints]

    ax.plot(wp_lons, wp_lats, color='#ff8800', linewidth=1.0,
            alpha=0.7, zorder=3)

    ib_lons = [wp_lons[i] for i in range(len(waypoints)) if i not in oob]
    ib_lats = [wp_lats[i] for i in range(len(waypoints)) if i not in oob]
    ax.scatter(ib_lons, ib_lats, c='#ff8800', s=15, zorder=4,
               edgecolors='white', linewidths=0.5, label='Waypoints')

    if oob:
        ob_lons = [wp_lons[i] for i in oob]
        ob_lats = [wp_lats[i] for i in oob]
        ax.scatter(ob_lons, ob_lats, c='red', s=60, zorder=5,
                   marker='X', linewidths=1.5,
                   label=f'OUT OF BOUNDS ({len(oob)})')

    if waypoints:
        ax.scatter([wp_lons[0]], [wp_lats[0]], c='#00cc00', s=80,
                   marker='o', zorder=6, edgecolors='white',
                   linewidths=1.5, label='Start')
        ax.scatter([wp_lons[-1]], [wp_lats[-1]], c='#cc0000', s=80,
                   marker='s', zorder=6, edgecolors='white',
                   linewidths=1.5, label='End')


def _info_box(ax, waypoints):
    """Draw the info box (waypoint count, altitude, path length)."""
    lines = [f"Waypoints: {len(waypoints)}"]
    if waypoints:
        lines.append(f"Alt: {waypoints[0]['alt']:.0f} m AGL")
    if len(waypoints) >= 2:
        total_m = 0
        for i in range(len(waypoints) - 1):
            dlat = (waypoints[i+1]["lat"] - waypoints[i]["lat"]) * 111_320
            dlon = (waypoints[i+1]["lon"] - waypoints[i]["lon"]) * \
                   111_320 * math.cos(math.radians(waypoints[i]["lat"]))
            total_m += math.sqrt(dlat**2 + dlon**2)
        lines.append(f"Path: {total_m:.0f} m")
    ax.text(0.02, 0.02, "\n".join(lines),
            transform=ax.transAxes, fontsize=9,
            verticalalignment='bottom',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                      alpha=0.85),
            zorder=10)


# ── Public API ───────────────────────────────────────────────────

def render_preview(polygon, waypoints, output_path=None, dual_pane=False):
    """
    Render the search/focus pattern preview image.

    Parameters
    ----------
    polygon    : list of (lat, lon) — search / focus area vertices
    waypoints  : list of {"lat", "lon", "alt"} dicts
    output_path: save path (defaults to config.PATTERN_PREVIEW_PATH)
    dual_pane  : if True, render a zoomed-out overview on the left
                 and a tight zoom on the right (for PLB focus pattern)

    Returns
    -------
    dict with keys: path, wp_count, oob_count, oob_indices
    """
    if output_path is None:
        output_path = config.PATTERN_PREVIEW_PATH

    bg_img, bg_extent = _load_background()
    oob = _oob_indices(waypoints)

    if dual_pane:
        _render_dual(polygon, waypoints, oob, bg_img, bg_extent, output_path)
    else:
        _render_single(polygon, waypoints, oob, bg_img, bg_extent, output_path)

    return {
        "path":       output_path,
        "wp_count":   len(waypoints),
        "oob_count":  len(oob),
        "oob_indices": oob,
    }


# ── Single-pane (initial search) ────────────────────────────────

def _render_single(polygon, waypoints, oob, bg_img, bg_extent, output_path):
    view_S, view_N, view_W, view_E = _content_bounds(polygon, waypoints,
                                                      pad_frac=0.15)

    fig, ax = plt.subplots(1, 1, figsize=(10, 8), dpi=128)
    ax.set_facecolor('#0a0a1a')

    if bg_img is not None:
        ax.imshow(bg_img, extent=bg_extent, aspect='auto', alpha=0.85, zorder=0)

    ax.set_xlim(view_W, view_E)
    ax.set_ylim(view_S, view_N)

    _draw_pattern(ax, polygon, waypoints, oob)

    if oob:
        ax.text(0.5, 0.97,
                f"⚠  {len(oob)} WAYPOINT(S) OUTSIDE MAP BOUNDS  ⚠",
                transform=ax.transAxes, fontsize=14, fontweight='bold',
                color='white', ha='center', va='top',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='red', alpha=0.9),
                zorder=10)

    _info_box(ax, waypoints)

    ax.legend(loc='upper left', fontsize=8, framealpha=0.85)
    ax.set_xlabel('Longitude', fontsize=9)
    ax.set_ylabel('Latitude', fontsize=9)
    ax.set_title('Search Pattern Preview — confirm before upload',
                 fontsize=11, fontweight='bold')
    _style_axes(ax)

    plt.tight_layout()
    plt.savefig(output_path, dpi=128, bbox_inches='tight', pad_inches=0.05)
    plt.close()


# ── Dual-pane (PLB focus pattern) ───────────────────────────────

def _render_dual(polygon, waypoints, oob, bg_img, bg_extent, output_path):
    """
    Left panel:  full satellite extent — polygon outline + zoom box
    Right panel: tight zoom — full waypoint detail
    """
    # Tight bounds for the right (zoom) panel
    zoom_S, zoom_N, zoom_W, zoom_E = _content_bounds(polygon, waypoints,
                                                       pad_frac=0.20)

    # Wide bounds for the left (overview) panel — use satellite extent
    # if available, otherwise fall back to a generous pad around content
    if bg_extent is not None:
        ov_W, ov_E, ov_S, ov_N = bg_extent  # imshow extent: W,E,S,N
    else:
        ov_S, ov_N, ov_W, ov_E = _content_bounds(polygon, waypoints,
                                                   pad_frac=2.0)

    fig, (ax_ov, ax_zm) = plt.subplots(
        1, 2,
        figsize=(20, 8),
        dpi=128,
    )
    fig.patch.set_facecolor('#0a0a1a')

    # ── Left: overview ──────────────────────────────────────
    ax_ov.set_facecolor('#0a0a1a')
    if bg_img is not None:
        ax_ov.imshow(bg_img, extent=bg_extent, aspect='auto', alpha=0.85,
                     zorder=0)
    ax_ov.set_xlim(ov_W, ov_E)
    ax_ov.set_ylim(ov_S, ov_N)

    # Draw just the polygon outline on the overview (no waypoints)
    poly_lons = [p[1] for p in polygon] + [polygon[0][1]]
    poly_lats = [p[0] for p in polygon] + [polygon[0][0]]
    ax_ov.fill(poly_lons, poly_lats, color='#ff8800', alpha=0.25)
    ax_ov.plot(poly_lons, poly_lats, color='#ff8800', linewidth=2.5,
               label='Focus Area')

    # Zoom box — dashed rectangle showing the right panel's extent
    zoom_rect = mpatches.FancyBboxPatch(
        (zoom_W, zoom_S),
        zoom_E - zoom_W,
        zoom_N - zoom_S,
        boxstyle="square,pad=0",
        linewidth=2,
        edgecolor='#ffffff',
        facecolor='none',
        linestyle='--',
        zorder=5,
    )
    ax_ov.add_patch(zoom_rect)
    ax_ov.text(zoom_E, zoom_N, '  detail →',
               color='white', fontsize=10, fontweight='bold',
               va='bottom', ha='left', zorder=6)

    ax_ov.legend(loc='upper left', fontsize=9, framealpha=0.85)
    ax_ov.set_xlabel('Longitude', fontsize=9)
    ax_ov.set_ylabel('Latitude', fontsize=9)
    ax_ov.set_title('Overview — Focus Area location', fontsize=12,
                    fontweight='bold', color='white')
    _style_axes(ax_ov, dark=True)

    # ── Right: zoom ─────────────────────────────────────────
    ax_zm.set_facecolor('#0a0a1a')
    if bg_img is not None:
        ax_zm.imshow(bg_img, extent=bg_extent, aspect='auto', alpha=0.85,
                     zorder=0)
    ax_zm.set_xlim(zoom_W, zoom_E)
    ax_zm.set_ylim(zoom_S, zoom_N)

    _draw_pattern(ax_zm, polygon, waypoints, oob)

    if oob:
        ax_zm.text(0.5, 0.97,
                   f"⚠  {len(oob)} WAYPOINT(S) OUT OF BOUNDS  ⚠",
                   transform=ax_zm.transAxes, fontsize=12, fontweight='bold',
                   color='white', ha='center', va='top',
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='red',
                             alpha=0.9),
                   zorder=10)

    _info_box(ax_zm, waypoints)
    ax_zm.legend(loc='upper left', fontsize=9, framealpha=0.85)
    ax_zm.set_xlabel('Longitude', fontsize=9)
    ax_zm.set_ylabel('Latitude', fontsize=9)
    ax_zm.set_title('Detail — Waypoint pattern', fontsize=12,
                    fontweight='bold', color='white')
    _style_axes(ax_zm, dark=True)

    fig.suptitle('Focus Pattern Preview — confirm before upload',
                 fontsize=14, fontweight='bold', color='white')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(output_path, dpi=128, bbox_inches='tight', pad_inches=0.1,
                facecolor=fig.get_facecolor())
    plt.close()
