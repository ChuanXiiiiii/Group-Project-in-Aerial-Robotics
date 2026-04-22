"""
states/generate_pattern.py — GENERATE_PATTERN state.
=====================================================
Subroutine state: called from IDLE as part of the mission setup
chain, returns to IDLE.

Steps:
    1. Reads search polygon from KML via kml_parser
    2. Calls pattern_generator.generate_lawnmower()
    3. Calls preview.render_preview() to produce a PNG
    4. Stores waypoints + preview info in shared["pending_pattern"]
    5. Returns to IDLE with pattern_status = "awaiting_confirmation"

The dashboard in IDLE then shows the preview image and offers
Confirm / Cancel buttons.  Confirm triggers UPLOAD_MISSION.
"""

import logging

import config
from kml_parser import parse_kml
from pattern_generator import (
    generate_lawnmower,
    generate_lawnmower_greedy,
    generate_square_box,
)
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
        """Do all the work in one shot, then return to IDLE."""

        try:
            hardcoded = getattr(config, "SEARCH_WAYPOINTS", [])
            if hardcoded:
                waypoints, polygon = self._generate_hardcoded()
            elif getattr(config, "FLIGHT_DAY_SIMPLE", False):
                waypoints, polygon = self._generate_square_box()
            else:
                waypoints, polygon = self._generate_lawnmower()

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

        return "IDLE"

    # ------------------------------------------------------------------
    # Pattern generators
    # ------------------------------------------------------------------

    def _generate_hardcoded(self):
        """
        Use the hardcoded SEARCH_WAYPOINTS from config, optionally
        prepended with SSSI_NAV_TO_SEARCH for the safe outbound corridor.

        Returns (waypoints, polygon) where polygon is the convex hull
        of the search waypoints (for preview rendering).
        """
        nav_to = getattr(config, "SSSI_NAV_TO_SEARCH", [])
        search = config.SEARCH_WAYPOINTS
        alt = float(config.SEARCH_ALT_M)

        all_coords = list(nav_to) + list(search)
        waypoints = [
            {"lat": float(lat), "lon": float(lon), "alt": alt}
            for lat, lon in all_coords
        ]

        if nav_to:
            self.log(f"Outbound corridor: {len(nav_to)} waypoints")
        self.log(f"Search pattern: {len(search)} hardcoded waypoints")
        self.log(f"Total mission: {len(waypoints)} waypoints @ {alt}m AGL")

        # Polygon for preview — just the search area corners
        polygon = [(float(lat), float(lon)) for lat, lon in search]

        return waypoints, polygon

    def _generate_square_box(self):
        """
        Generate the flight day 1 box pattern.

        Priority:
            1. Use PREDEFINED_WAYPOINTS from config if populated —
               manually surveyed coordinates that avoid buildings.
            2. Fall back to GPS-centred square box if list is empty.

        Returns (waypoints, polygon).
        """
        predefined = getattr(config, "PREDEFINED_WAYPOINTS", [])

        if predefined:
            self.log(f"Using predefined waypoints ({len(predefined)} points)")
            waypoints = [
                {"lat": float(lat), "lon": float(lon), "alt": float(config.SEARCH_ALT_M)}
                for lat, lon in predefined
            ]
            # Polygon is the unique corners (drop closing repeat if present)
            unique = predefined[:-1] if (
                len(predefined) > 1 and predefined[0] == predefined[-1]
            ) else predefined
            polygon = [(float(lat), float(lon)) for lat, lon in unique]
            self.log(f"Predefined box: {len(waypoints)} waypoints, "
                     f"{len(polygon)} polygon vertices, alt={config.SEARCH_ALT_M}m")
            return waypoints, polygon

        # ── Fallback: GPS-centred square ──
        self.log(f"No predefined waypoints — generating GPS-centred "
                 f"{config.SQUARE_BOX_SIDE_M}×{config.SQUARE_BOX_SIDE_M}m box")
        telem = self.drone.get_telemetry()
        centre_lat = telem.get("lat")
        centre_lon = telem.get("lon")

        if centre_lat is None or centre_lon is None:
            raise ValueError(
                "No GPS position available — cannot centre square box. "
                "Either set PREDEFINED_WAYPOINTS in config.py or ensure "
                "GPS fix before generating pattern."
            )

        self.log(f"Box centre: {centre_lat:.6f}, {centre_lon:.6f}")
        waypoints = generate_square_box(
            centre_lat=centre_lat,
            centre_lon=centre_lon,
            side_m=config.SQUARE_BOX_SIDE_M,
            alt_m=config.SEARCH_ALT_M,
        )
        self.log(f"Generated {len(waypoints)} waypoints "
                 f"(side={config.SQUARE_BOX_SIDE_M}m, alt={config.SEARCH_ALT_M}m)")
        polygon = [(wp["lat"], wp["lon"]) for wp in waypoints[:4]]
        return waypoints, polygon

    def _generate_lawnmower(self):
        """
        Generate a lawnmower pattern from the KML search area polygon.

        Algorithm is selected by config.LAWNMOWER_ALGORITHM:
            "scanline" — Carter's scanline-clip boustrophedon (default)
            "greedy"   — Teammate's shapely strip + greedy ordering

        Returns (waypoints, polygon).
        """
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

        algorithm = getattr(config, "LAWNMOWER_ALGORITHM", "scanline")

        if algorithm == "greedy":
            self.log(f"Algorithm: greedy (shapely strip)")
            waypoints = generate_lawnmower_greedy(
                polygon=polygon,
                spacing_m=config.SWATH_WIDTH_M,
                alt_m=config.SEARCH_ALT_M,
                heading_deg=config.SCAN_HEADING_DEG,
            )
        else:
            self.log(f"Algorithm: scanline (boustrophedon)")
            waypoints = generate_lawnmower(
                polygon=polygon,
                spacing_m=config.SWATH_WIDTH_M,
                alt_m=config.SEARCH_ALT_M,
                heading_deg=config.SCAN_HEADING_DEG,
            )

        self.log(f"Generated {len(waypoints)} waypoints "
                 f"(spacing={config.SWATH_WIDTH_M}m, "
                 f"alt={config.SEARCH_ALT_M}m, "
                 f"algorithm={algorithm})")
        return waypoints, polygon

    def exit(self):
        pass
