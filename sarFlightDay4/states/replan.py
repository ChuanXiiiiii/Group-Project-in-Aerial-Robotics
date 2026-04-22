"""
states/replan.py — REPLAN state.
=================================
Receives the PLB Focus Area polygon and generates a new tighter
search pattern for the FOCUS phase.

Entry:
    SEARCH detects PLB activation → switches to GUIDED → transitions
    here.  The drone holds position in GUIDED while we replan.

Workflow:
    Phase 1 — Wait for operator to drop a Focus Area KML into
              PLB_KML_DIR and press "Load PLB KML" on the dashboard.
    Phase 2 — Parse the KML, extract the Focus Area polygon,
              generate a focus lawnmower via waypoint_generator,
              render a preview image, and wait for operator to
              confirm/cancel.
    Phase 3 — On confirm: re-upload fences (base + focus inclusion),
              upload the new mission, transition to FOCUS.

IMPORTANT — Fence re-upload:
    The MAVLink fence protocol replaces ALL fences on each upload.
    We must include the base fences (Flight Area inclusion + SSSI
    exclusion) alongside any new Focus Area fence.  The base fences
    were stored in shared["base_fence_polygons"] by UPLOAD_FENCE.

Transitions:
    → UPLOAD_FENCE   (with return_to=REPLAN, replan_phase=3)
      when fences need re-uploading with Focus Area
    → UPLOAD_MISSION (with return_to=FOCUS)
      after operator confirms the focus pattern
    → IDLE           on cancel or error
"""

import os
import glob
import time
from states.base import BaseState
from config import (
    HEARTBEAT_INTERVAL,
    PLB_KML_DIR,
    FOCUS_SWATH_WIDTH_M,
    FOCUS_SCAN_ANGLE_DEG,
    FOCUS_ALT_M,
    PATTERN_PREVIEW_PATH,
)


class ReplanState(BaseState):

    name = "REPLAN"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enter(self):
        phase = self.shared.get("replan_phase", 1)
        self.log(f"Entered REPLAN phase {phase} — drone holding in GUIDED")
        if phase == 1:
            self._push_status("waiting_for_kml")

    def execute(self):
        phase = self.shared.get("replan_phase", 1)
        if   phase == 1: return self._phase_wait_for_kml()
        elif phase == 2: return self._phase_generate_and_confirm()
        elif phase == 3: return self._phase_upload()
        else:
            self.log(f"ERROR: unknown replan phase {phase}")
            return "IDLE"

    def exit(self):
        # Only clean up replan keys when leaving REPLAN for good
        # (i.e. not on a REPLAN→REPLAN self-transition between phases).
        # We detect a "real" exit by checking whether replan_phase is
        # still set — if we're self-transitioning we leave it intact so
        # enter() picks up the correct phase on re-entry.
        # Keys that must survive self-transitions: replan_phase, plb_kml_path,
        # focus_polygon, pending_focus_pattern, _replan_fences_done.
        # They are explicitly popped inside _phase_upload() once the final
        # UPLOAD_MISSION transition fires, and by _clear_plb_flags() on cancel.
        if self.drone.status:
            self.drone.status.set_extra("replan_status", None)

    # ------------------------------------------------------------------
    # Phase 1 — Wait for KML file and operator trigger
    # ------------------------------------------------------------------

    def _phase_wait_for_kml(self):
        """
        Poll for the operator to press 'Load PLB KML' on the dashboard.
        The KML file must already be in PLB_KML_DIR.
        """
        self.log(f"Waiting for PLB Focus Area KML in {PLB_KML_DIR}")
        self._push_status("waiting_for_kml")

        # Ensure the directory exists
        os.makedirs(PLB_KML_DIR, exist_ok=True)

        while True:
            self.drone.drain_buffer()
            self.drone.get_mode()
            self.drone.get_telemetry()

            # Check for GUI command
            if self.drone.status:
                cmd = self.drone.status.consume_command()
                if cmd is not None:
                    cmd_type = cmd.get("type")

                    if cmd_type == "load_plb_kml":
                        # Look for a .kml file in the directory
                        kml_path = self._find_kml()
                        if kml_path is None:
                            self.log(f"No .kml file found in {PLB_KML_DIR}")
                            self._push_status("no_kml_found")
                            # Stay in phase 1
                        else:
                            self.log(f"Found KML: {kml_path}")
                            self.shared["plb_kml_path"] = kml_path
                            self.shared["replan_phase"] = 2
                            return "REPLAN"

                    elif cmd_type == "cancel_replan":
                        self.log("REPLAN cancelled by operator — resuming search")
                        self._clear_plb_flags()
                        return self._resume_or_idle()

            time.sleep(HEARTBEAT_INTERVAL)

    # ------------------------------------------------------------------
    # Phase 2 — Parse KML, generate pattern, await confirmation
    # ------------------------------------------------------------------

    def _phase_generate_and_confirm(self):
        """
        Parse the Focus Area KML, generate the focus lawnmower pattern
        using the teammate's waypoint_generator, render a preview, and
        wait for operator confirmation.
        """
        kml_path = self.shared.get("plb_kml_path")
        if not kml_path or not os.path.exists(kml_path):
            self.log(f"ERROR: KML file not found at {kml_path}")
            self._push_status("error")
            self._cleanup_replan_keys()
            return "IDLE"

        # ── Parse KML ──
        try:
            from kml_parser import parse_kml
            kml_data = parse_kml(kml_path)
        except Exception as e:
            self.log(f"ERROR parsing KML: {e}")
            self._push_status("error")
            self._cleanup_replan_keys()
            return "IDLE"

        # Find the Focus Area polygon (try common names)
        focus_polygon = None
        for name in ("Focus Area", "FocusArea", "focus area", "Focus area",
                      "PLB Area", "PLB", "Survey Area", "Search Area"):
            if name in kml_data:
                focus_polygon = kml_data[name]
                self.log(f"Using polygon: '{name}' ({len(focus_polygon)} vertices)")
                break

        if focus_polygon is None:
            self.log(f"ERROR: No Focus Area polygon found in KML. "
                     f"Available: {list(kml_data.keys())}")
            self._push_status("error")
            self._cleanup_replan_keys()
            return "IDLE"

        if len(focus_polygon) < 3:
            self.log(f"ERROR: Focus Area has {len(focus_polygon)} points (need >= 3)")
            self._push_status("error")
            self._cleanup_replan_keys()
            return "IDLE"

        # Store the polygon (lat,lon) for fence re-upload later
        self.shared["focus_polygon"] = focus_polygon

        # ── Generate focus pattern via teammate's planner ──
        # KML parser returns (lat, lon) — teammate's planner expects (lon, lat)
        polygon_lonlat = [(lon, lat) for lat, lon in focus_polygon]

        try:
            from waypoint_generator import generate_waypoints
            waypoints = generate_waypoints(
                polygon_coords=polygon_lonlat,
                width=FOCUS_SWATH_WIDTH_M,
                angle=FOCUS_SCAN_ANGLE_DEG,
                altitude=FOCUS_ALT_M,
            )
            self.log(f"Focus pattern generated: {len(waypoints)} waypoints "
                     f"(width={FOCUS_SWATH_WIDTH_M}m, angle={FOCUS_SCAN_ANGLE_DEG}deg, "
                     f"alt={FOCUS_ALT_M}m)")
        except Exception as e:
            self.log(f"ERROR generating focus pattern: {e}")
            self._push_status("error")
            self._cleanup_replan_keys()
            return "IDLE"

        if not waypoints:
            self.log("ERROR: waypoint generator returned empty list")
            self._push_status("error")
            self._cleanup_replan_keys()
            return "IDLE"

        # ── Render preview ──
        try:
            from preview import render_preview
            info = render_preview(
                polygon=focus_polygon,
                waypoints=waypoints,
                output_path=PATTERN_PREVIEW_PATH,
                dual_pane=True,
            )
            self.log(f"Focus preview saved: {info['path']} "
                     f"({info['wp_count']} WPs, {info['oob_count']} OOB)")
        except Exception as e:
            self.log(f"WARNING: preview render failed: {e} — continuing without preview")

        # ── Stage for confirmation ──
        self.shared["pending_focus_pattern"] = {
            "waypoints": waypoints,
            "polygon": focus_polygon,
            "wp_count": len(waypoints),
        }
        self._push_status("awaiting_confirmation", {
            "wp_count": len(waypoints),
        })
        self.log("Focus pattern staged — awaiting operator confirmation")

        # ── Wait for confirm/cancel ──
        while True:
            self.drone.drain_buffer()
            self.drone.get_mode()
            self.drone.get_telemetry()

            if self.drone.status:
                cmd = self.drone.status.consume_command()
                if cmd is not None:
                    cmd_type = cmd.get("type")

                    if cmd_type == "confirm_focus_pattern":
                        self.log("Focus pattern CONFIRMED by operator")
                        self.shared["replan_phase"] = 3
                        return "REPLAN"

                    elif cmd_type == "cancel_replan":
                        self.log("REPLAN cancelled — resuming search")
                        self.shared.pop("pending_focus_pattern", None)
                        self._clear_plb_flags()
                        return self._resume_or_idle()

            time.sleep(HEARTBEAT_INTERVAL)

    # ------------------------------------------------------------------
    # Phase 3 — Re-upload fences + upload focus mission → FOCUS
    # ------------------------------------------------------------------

    def _phase_upload(self):
        """
        Upload the confirmed focus pattern to the autopilot and
        re-upload fences with the Focus Area added as an inclusion zone.
        """
        pending = self.shared.get("pending_focus_pattern")
        if not pending:
            self.log("ERROR: no pending focus pattern — returning to IDLE")
            self._cleanup_replan_keys()
            return "IDLE"

        waypoints = pending["waypoints"]
        focus_polygon = self.shared.get("focus_polygon", [])

        # ── Re-upload fences: base fences + Focus Area inclusion ──
        # Skip if we already did this (returning from UPLOAD_FENCE)
        base_fences = self.shared.get("base_fence_polygons", [])
        fences_done = self.shared.get("_replan_fences_done", False)

        if base_fences and len(focus_polygon) >= 3 and not fences_done:
            all_fences = list(base_fences)  # copy
            all_fences.append({
                "type": "inclusion",
                "points": focus_polygon,
                "label": "Focus Area",
            })
            self.shared["fence_polygons"] = all_fences
            self.shared["return_to"] = "REPLAN"
            self.shared["replan_phase"] = 3
            self.shared["_replan_fences_done"] = True
            self.log(f"Re-uploading fences: {len(all_fences)} polygons "
                     f"(base + Focus Area)")
            return "UPLOAD_FENCE"

        # ── Fences done (or no base fences) — upload mission ──
        # Validate waypoints
        validated = []
        for i, wp in enumerate(waypoints):
            try:
                lat = float(wp["lat"])
                lon = float(wp["lon"])
                alt = float(wp["alt"])
            except (KeyError, TypeError, ValueError) as e:
                self.log(f"WP {i+1} invalid: {e}")
                self._cleanup_replan_keys()
                return "IDLE"
            validated.append({"lat": lat, "lon": lon, "alt": alt})

        self.shared["waypoints"] = validated
        self.shared["focus_waypoints"] = validated  # keep for reference
        self.shared["return_to"] = "FOCUS"

        # Expose focus waypoints and polygon to GUI/spectator via SharedStatus
        # so external displays can draw the replanned pattern dynamically.
        if self.drone.status:
            self.drone.status.set_extra("focus_waypoints",
                                        [[wp["lat"], wp["lon"]] for wp in validated])
            if focus_polygon:
                self.drone.status.set_extra("focus_polygon_coords", focus_polygon)

        self._cleanup_replan_keys()
        self.log(f"Uploading {len(validated)} focus waypoints → FOCUS")
        return "UPLOAD_MISSION"

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _cleanup_replan_keys(self):
        """Remove all replan working keys from shared. Call before any
        transition that leaves REPLAN for good (IDLE, SEARCH, UPLOAD_MISSION,
        or error returns). Do NOT call on REPLAN→REPLAN self-transitions."""
        for k in ("replan_phase", "plb_kml_path",
                  "pending_focus_pattern", "_replan_fences_done"):
            self.shared.pop(k, None)

    def _find_kml(self):
        """Find the first .kml file in PLB_KML_DIR."""
        pattern = os.path.join(PLB_KML_DIR, "*.kml")
        files = sorted(glob.glob(pattern))
        if files:
            return files[0]
        return None

    def _clear_plb_flags(self):
        """Reset PLB state so the search can resume cleanly."""
        self.shared.pop("plb_triggered", None)
        if self.drone.status:
            self.drone.status.set_extra("plb_button_pressed", False)

    def _resume_or_idle(self):
        """
        Resume the original search if a mission is still on the autopilot,
        otherwise fall back to IDLE.

        SEARCH re-entry with no search_phase key triggers phase 1
        (verify mission on autopilot → GUIDED gate → AUTO), so the
        mission will resume from wherever the autopilot's sequence
        counter is.
        """
        self._cleanup_replan_keys()
        try:
            wp_count = self.drone.query_mission_count(timeout=3)
        except Exception:
            wp_count = 0

        if wp_count >= 2:
            self.log(f"Mission still on autopilot ({wp_count} items) — resuming SEARCH")
            return "SEARCH"
        else:
            self.log("No mission on autopilot — returning to IDLE")
            return "IDLE"

    def _push_status(self, status, info=None):
        if self.drone.status:
            self.drone.status.set_extra("replan_status", {
                "status": status,
                "info": info,
            })
