"""
states/idle.py — IDLE state.
==============================
Primary resting and mission-setup state for flying day.

The drone is manually flown to hover height in LOITER mode by the
safety pilot before the state machine takes over here.  IDLE handles:

    1. Live telemetry monitoring (heartbeat, GPS, battery, mode)
    2. Mission setup sequence via GUI buttons:
         a. 'Setup Mission'   → UPLOAD_FENCE → GENERATE_PATTERN → IDLE
         b. 'Confirm Pattern' → UPLOAD_MISSION → IDLE
         c. 'Open Pre-AUTO Checklist' → PRE_AUTO_CHECK
    3. Display of PLB flag if set after returning from SEARCH

Transitions:
    → UPLOAD_FENCE       on 'Setup Mission' command
    → GENERATE_PATTERN   when UPLOAD_FENCE returns (via chain)
    → UPLOAD_MISSION     on 'Confirm Pattern' command
    → PRE_AUTO_CHECK     on 'open_checklist' command

All subroutine states use shared["return_to"] = "IDLE" so they
always return here after completion.

Trigger file commands accepted:
    FENCE              upload fences from KML
    PATTERN            generate search pattern
    CHECKLIST          open pre-AUTO checklist
"""

import os
import time
from states.base import BaseState
from config import (
    TRIGGER_FILE, HEARTBEAT_INTERVAL, KML_FILE,
)


class IdleState(BaseState):

    name = "IDLE"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enter(self):
        self.log("IDLE — monitoring telemetry, awaiting operator commands")
        if "idle_hb_miss" not in self.shared:
            self.shared["idle_hb_miss"] = 0
        # Persist manual checkbox flags across IDLE re-entries
        if "pre_auto_wp_verified" not in self.shared:
            self.shared["pre_auto_wp_verified"] = False
        if "pre_auto_pilot_ready" not in self.shared:
            self.shared["pre_auto_pilot_ready"] = False
        self._push_idle_status()

    def execute(self):
        while True:
            # ── 1. Heartbeat / mode pump ──────────────────────────────
            mode_name, _ = self.drone.get_mode()
            hb_ok = (mode_name != "UNKNOWN")
            if hb_ok:
                self.shared["idle_hb_miss"] = 0
            else:
                self.shared["idle_hb_miss"] = self.shared.get("idle_hb_miss", 0) + 1

            if hb_ok:
                self.log(f"mode={mode_name}")

            # ── 2. Push live telemetry to GUI ─────────────────────────
            self._push_idle_status()

            # ── 3. GUI command ────────────────────────────────────────
            if self.drone.status:
                cmd = self.drone.status.consume_command()
                if cmd is not None:
                    next_state = self._handle_gui_command(cmd)
                    if next_state is not None:
                        return next_state

            # ── 4. Trigger file ───────────────────────────────────────
            if os.path.exists(TRIGGER_FILE):
                next_state = self._read_trigger()
                if next_state is not None:
                    return next_state

            time.sleep(HEARTBEAT_INTERVAL)

    def exit(self):
        pass

    # ------------------------------------------------------------------
    # GUI command handler
    # ------------------------------------------------------------------

    def _handle_gui_command(self, cmd):
        """
        Dispatch a command dict received from the web dashboard.
        Returns next state name or None to stay in IDLE.
        """
        cmd_type = cmd.get("type")

        # ── Setup Mission: fence upload → pattern gen → idle ──
        if cmd_type == "setup_mission":
            self.log("GUI: Setup Mission — starting fence upload")
            return self._prepare_fence_upload(source="GUI")

        # ── Confirm staged pattern → mission upload ──
        elif cmd_type == "confirm_pattern":
            return self._confirm_pattern(source="GUI")

        # ── Cancel staged pattern ──
        elif cmd_type == "cancel_pattern":
            self.shared.pop("pending_pattern", None)
            self.shared["pattern_status"] = "cancelled"
            if self.drone.status:
                self.drone.status.set_extra("pattern_status", "cancelled")
                self.drone.status.set_extra("pending_pattern_info", None)
            self.log("GUI: pattern cancelled")
            return None

        # ── Open Pre-AUTO checklist ──
        elif cmd_type == "open_checklist":
            if not self.shared.get("mission_uploaded"):
                self.log("GUI: checklist rejected — mission not yet uploaded")
                return None
            self.log("GUI: opening Pre-AUTO checklist")
            return "PRE_AUTO_CHECK"

        # ── Guided takeoff (optional / SITL) ──
        elif cmd_type == "guided_takeoff":
            self.log("GUI: Guided Takeoff requested")
            return "GUIDED_TAKEOFF"

        # ── Manual checkbox toggles (forwarded from PRE_AUTO_CHECK) ──
        elif cmd_type == "pre_auto_toggle":
            field = cmd.get("field")
            if field == "wp_verified":
                self.shared["pre_auto_wp_verified"] = not self.shared.get(
                    "pre_auto_wp_verified", False)
                self.log(f"GUI: waypoints verified = {self.shared['pre_auto_wp_verified']}")
            elif field == "pilot_ready":
                self.shared["pre_auto_pilot_ready"] = not self.shared.get(
                    "pre_auto_pilot_ready", False)
                self.log(f"GUI: pilot ready = {self.shared['pre_auto_pilot_ready']}")
            return None

        self.log(f"GUI: unknown command type '{cmd_type}' — ignoring")
        return None

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _prepare_fence_upload(self, source="GUI"):
        """
        Parse KML and stage fence polygons for UPLOAD_FENCE.
        After fences are uploaded, UPLOAD_FENCE will return to
        GENERATE_PATTERN (not IDLE) so the pattern is generated
        immediately as part of the setup chain.
        """
        if not os.path.exists(KML_FILE):
            self.log(f"{source}: ERROR — KML file not found at '{KML_FILE}'")
            return None

        try:
            from kml_parser import get_fence_polygons
            flight_area, sssi = get_fence_polygons(KML_FILE)
        except Exception as e:
            self.log(f"{source}: ERROR parsing KML fences — {e}")
            return None

        if len(flight_area) < 3 or len(sssi) < 3:
            self.log(f"{source}: ERROR — KML polygons too small "
                     f"(Flight Area: {len(flight_area)} pts, "
                     f"SSSI: {len(sssi)} pts)")
            return None

        self.shared["fence_polygons"] = [
            {"type": "inclusion", "points": flight_area, "label": "Flight Area"},
            {"type": "exclusion", "points": sssi,        "label": "SSSI"},
        ]
        # After fence upload, go directly to pattern generation
        self.shared["return_to"] = "GENERATE_PATTERN"
        self.log(f"{source}: fence upload staged — chain: "
                 f"UPLOAD_FENCE → GENERATE_PATTERN → IDLE")
        return "UPLOAD_FENCE"

    def _confirm_pattern(self, source="GUI"):
        """
        Promote pending_pattern waypoints into shared['waypoints']
        and transition to UPLOAD_MISSION.
        """
        pending = self.shared.get("pending_pattern")
        if not pending or not pending.get("waypoints"):
            self.log(f"{source}: no pending pattern to confirm")
            return None

        waypoints = pending["waypoints"]
        validated = []
        for i, wp in enumerate(waypoints):
            try:
                lat = float(wp["lat"])
                lon = float(wp["lon"])
                alt = float(wp["alt"])
            except (KeyError, TypeError, ValueError) as e:
                self.log(f"{source}: WP {i+1} invalid — {e}")
                return None
            if not (-90 <= lat <= 90 and -180 <= lon <= 180 and 0 <= alt <= 50):
                self.log(f"{source}: WP {i+1} coordinates out of range")
                return None
            validated.append({"lat": lat, "lon": lon, "alt": alt})

        self.shared["waypoints"]          = validated
        self.shared["original_waypoints"] = validated   # kept for second-pass re-upload
        self.shared["return_to"]          = "IDLE"
        self.shared.pop("pending_pattern", None)
        if self.drone.status:
            self.drone.status.set_extra("pattern_status", "confirmed")
            self.drone.status.set_extra("pending_pattern_info", None)
        self.log(f"{source}: pattern confirmed — uploading {len(validated)} waypoints")
        # mission_uploaded flag is set by UPLOAD_MISSION on success;
        # pre-set it optimistically so the checklist gate responds immediately
        self.shared["mission_uploaded"] = True
        return "UPLOAD_MISSION"

    # ------------------------------------------------------------------
    # Trigger file
    # ------------------------------------------------------------------

    def _read_trigger(self):
        try:
            with open(TRIGGER_FILE, "r") as f:
                raw = f.read().strip()
            os.remove(TRIGGER_FILE)
            parts = raw.split()
            if not parts:
                return None
            cmd = parts[0].upper()

            if cmd == "FENCE":
                self.log("TRIGGER: fence upload")
                return self._prepare_fence_upload(source="TRIGGER")

            if cmd == "PATTERN":
                self.log("TRIGGER: generate pattern")
                self.shared["return_to"] = "IDLE"
                return "GENERATE_PATTERN"

            if cmd == "CHECKLIST":
                if not self.shared.get("mission_uploaded"):
                    self.log("TRIGGER: checklist rejected — mission not uploaded")
                    return None
                self.log("TRIGGER: open checklist")
                return "PRE_AUTO_CHECK"

            if cmd == "TAKEOFF":
                self.log("TRIGGER: guided takeoff")
                return "GUIDED_TAKEOFF"

            self.log(f"TRIGGER: unknown command '{raw}' — ignoring")
            return None
        except Exception as e:
            self.log(f"TRIGGER ERROR: {e}")
            return None

    # ------------------------------------------------------------------
    # GUI status push
    # ------------------------------------------------------------------

    def _push_idle_status(self):
        """Push live telemetry and setup flags to the dashboard."""
        if not self.drone.status:
            return
        telem = self.drone.get_telemetry()
        self.drone.status.set_extra("idle_status", {
            "fence_uploaded":   bool(self.shared.get("fence_uploaded")),
            "mission_uploaded": bool(self.shared.get("mission_uploaded")),
            "pattern_pending":  bool(self.shared.get("pending_pattern")),
            "plb_triggered":    bool(self.shared.get("plb_triggered")),
            "armed":        self.drone.get_armed(),
            "alt":        round(telem.get("alt") or 0.0, 1),
            "lat":        round(telem.get("lat") or 0.0, 6),
            "lon":        round(telem.get("lon") or 0.0, 6),
            "battery_pct":  telem.get("battery_pct"),
            "satellites":   telem.get("satellites"),
            "gps_fix":      telem.get("gps_fix"),
        })
