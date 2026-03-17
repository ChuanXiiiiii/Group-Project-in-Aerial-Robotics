"""
states/generate_pattern.py — GENERATE_PATTERN state.
=====================================================
Subroutine state: called from WAIT, returns to WAIT.

1. Reads search polygon from KML via kml_parser
2. Calls pattern_generator.generate_pattern()
3. Calls preview.render_preview() to produce a PNG
4. Stores waypoints + preview info in shared dict
5. Returns to WAIT with pending_pattern flag set

The dashboard in WAIT then shows the preview and
offers Confirm / Cancel buttons.
"""

import logging

import config
from kml_parser import parse_kml
from path_planner.common import corner_map_p0_p4_latlon
from pattern_generator import PATTERN_NAMES, generate_pattern
from preview import render_preview
from states.base import BaseState

logger = logging.getLogger("state.generate_pattern")


class GeneratePatternState(BaseState):

    name = "GENERATE_PATTERN"

    def _push_status(self, status, info=None):
        """Push pattern status to both shared dict and SharedStatus (GUI)."""
        self.shared["pattern_status"] = status
        if self.drone.status:
            self.drone.status.set_extra("pattern_status", status)
            self.drone.status.set_extra("pending_pattern_info", info)

    def enter(self):
        self.log("Entered GENERATE_PATTERN")
        self._push_status("running")

    def execute(self):
        """Do all the work in one shot, then return to WAIT."""

        try:
            # ── 1. Get search polygon from KML ───────────────
            kml_data = parse_kml(config.KML_FILE)

            polygon = None
            for poly_name in ("Survey Area", "Search Area"):
                if poly_name in kml_data:
                    polygon = kml_data[poly_name]
                    self.log(f"Using polygon: '{poly_name}' "
                             f"({len(polygon)} vertices)")
                    break

            if polygon is None:
                raise ValueError(
                    "No 'Survey Area' or 'Search Area' found in KML")

            # Introduce explicit P0..P4 naming for 5-vertex search areas.
            corner_map = corner_map_p0_p4_latlon(polygon)
            if corner_map:
                self.log(
                    "Corner map (P0..P4): "
                    + ", ".join(
                        f"{name}=({lat:.8f},{lon:.8f})"
                        for name, (lat, lon) in corner_map.items()
                    )
                )

            # ── 2. Generate selected pattern algorithm ───────
            waypoints = generate_pattern(
                polygon=polygon,
                spacing_m=config.SWATH_WIDTH_M,
                alt_m=config.SEARCH_ALT_M,
                heading_deg=config.SCAN_HEADING_DEG,
                algorithm_id=config.PATTERN_ALGORITHM,
            )
            pattern_name = PATTERN_NAMES.get(config.PATTERN_ALGORITHM, "unknown")
            self.log(f"Generated {len(waypoints)} waypoints "
                     f"(algorithm={config.PATTERN_ALGORITHM}:{pattern_name}, "
                     f"spacing={config.SWATH_WIDTH_M}m, "
                     f"alt={config.SEARCH_ALT_M}m)")

            # ── 3. Render preview image ──────────────────────
            info = render_preview(
                polygon=polygon,
                waypoints=waypoints,
                output_path=config.PATTERN_PREVIEW_PATH,
            )
            self.log(f"Preview saved: {info['path']} "
                     f"({info['wp_count']} WPs, "
                     f"{info['oob_count']} out-of-bounds)")

            if info["oob_count"] > 0:
                self.log(f"⚠ {info['oob_count']} waypoint(s) "
                         f"fall outside map bounds!")

            # ── 4. Stage for operator confirmation ───────────
            self.shared["pending_pattern"] = {
                "waypoints": waypoints,
                "polygon": [(p[0], p[1]) for p in polygon],
                "corners": corner_map,
                "preview_path": info["path"],
                "wp_count": info["wp_count"],
                "oob_count": info["oob_count"],
            }
            self._push_status("awaiting_confirmation", {
                "wp_count": info["wp_count"],
                "oob_count": info["oob_count"],
            })
            self.log("Pattern staged — awaiting operator confirmation")

        except Exception as e:
            logger.error(f"GENERATE_PATTERN failed: {e}")
            self.shared.pop("pending_pattern", None)
            self._push_status(f"error: {e}")

        return "WAIT"

    def exit(self):
        pass
