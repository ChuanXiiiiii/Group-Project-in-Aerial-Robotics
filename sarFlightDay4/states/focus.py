"""
states/focus.py — FOCUS state.
================================
Executes the focus search pattern in AUTO mode while monitoring
for casualty detections from the CV pipeline.

Entry:
    REPLAN uploads the focus mission → UPLOAD_MISSION returns here.
    The drone is in GUIDED after REPLAN.  FOCUS verifies the mission,
    requests AUTO via CHANGE_MODE (with the GUIDED gate), and enters
    the monitoring loop.

CV detection monitoring:
    The CV pipeline drops detection images into DETECTION_DIR.
    Filenames encode the geotag as: "NUM_LAT_LON.png"
    e.g. "3_51.4241_-2.6714.png" where NUM is the chronological
    detection index.  FOCUS scans this directory on every loop tick.
    When a file appears, it extracts the coordinates, stores them in
    shared, and transitions to DELIVER.

Exit conditions:
    a. Detection found        → GUIDED → IDLE (DELIVER in future)
    b. Focus pattern complete → GUIDED → IDLE (RTH in future)
    c. RC override            → pause, wait for GUIDED, resume
    d. Cancel command         → GUIDED → IDLE

Mode discipline:
    Same GUIDED gate as SEARCH: the state machine only requests AUTO
    when the RC is in GUIDED.  The pilot can always take over by
    flipping to LOITER; the state machine waits for GUIDED before
    resuming.
"""

import os
import glob
import time
from states.base import BaseState
from config import (
    HEARTBEAT_INTERVAL,
    DETECTION_DIR,
    FOCUS_SPEED_MPS,
    TRANSIT_SPEED_MPS,
)
from cv_comm import write_cv_mode

_MODE_LOSS_GRACE = 2
_OVERRIDE_TIMEOUT = 120
_MONITOR_TICK_S  = 0.5   # same as search.py — fast enough for responsive GUI


class FocusState(BaseState):

    name = "FOCUS"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enter(self):
        if "focus_phase" not in self.shared:
            self.shared["focus_phase"] = 1
            self.shared["focus_total_wps"] = 0
            self.shared["focus_current_wp"] = 0
            self.shared["focus_last_reached"] = -1
            self.shared["focus_status"] = "INITIALISING"
            self.shared["focus_start_time"] = None
        phase = self.shared["focus_phase"]
        self.log(f"Entered FOCUS phase {phase}")
        # Ensure detection directory exists
        os.makedirs(DETECTION_DIR, exist_ok=True)
        # Clear any stale detections from prior runs
        if phase == 1:
            self._clear_old_detections()
            write_cv_mode("DETECTING")
            self.log("CV mode → DETECTING")

    def execute(self):
        phase = self.shared["focus_phase"]
        if   phase == 1: return self._phase_verify()
        elif phase == 2: return self._phase_monitor()
        elif phase == 3: return self._phase_cleanup()
        else:
            self.log(f"ERROR: unknown focus phase {phase}")
            return "IDLE"

    def exit(self):
        if self.drone.status:
            self.drone.status.set_extra("focus_progress", None)

    # ------------------------------------------------------------------
    # Phase 1 — Verify mission on autopilot, GUIDED gate, request AUTO
    # ------------------------------------------------------------------

    def _phase_verify(self):
        wp_count = 0
        try:
            wp_count = self.drone.query_mission_count(timeout=5)
            self.log(f"Autopilot reports {wp_count} mission items")
        except Exception as e:
            self.log(f"ERROR querying mission count: {e}")

        if wp_count < 2:
            self.log("ERROR: no valid mission on autopilot — returning to IDLE")
            self.shared["focus_status"] = "NO MISSION"
            self._push_progress()
            self.shared["focus_phase"] = 3
            return "FOCUS"

        self.shared["focus_total_wps"] = wp_count - 1
        self.log(f"Mission verified: {self.shared['focus_total_wps']} nav waypoints")

        self.drone.drain_buffer()
        mode_name, _ = self.drone.get_mode(retries=5)

        if mode_name == "AUTO":
            self.log("Already in AUTO — proceeding to monitor")
            self.shared["focus_phase"] = 2
            self.shared["focus_start_time"] = time.time()
            self.shared["focus_status"] = "FLYING"
            self._push_progress()
            return "FOCUS"

        # ── GUIDED gate ──
        if mode_name != "GUIDED":
            self.log(f"Mode is {mode_name}, not GUIDED — waiting for pilot")
            self.shared["focus_status"] = "WAITING FOR GUIDED"
            self._push_progress()
            while True:
                time.sleep(_MONITOR_TICK_S)
                self.drone.drain_buffer()
                mode_name, _ = self.drone.get_mode(retries=2)
                self.drone.get_telemetry()
                self.log(f"[WAIT-GUIDED] mode={mode_name}")

                if mode_name == "GUIDED":
                    self.log("Pilot set GUIDED — proceeding")
                    break

                if self._check_cancel():
                    self.log("Cancel received while waiting for GUIDED")
                    self._cleanup_focus_keys()
                    return "IDLE"

        # Speed is set at the start of _phase_monitor(), AFTER AUTO is
        # confirmed.  DO_CHANGE_SPEED is ignored in GUIDED mode.

        # Request AUTO
        self.shared["requested_mode"] = "AUTO"
        self.shared["return_to"] = "FOCUS"
        self.shared["focus_phase"] = 2
        self.shared["focus_start_time"] = time.time()
        self.shared["focus_status"] = "SWITCHING TO AUTO"
        self._push_progress()
        self.log("Requesting AUTO via CHANGE_MODE")
        return "CHANGE_MODE"

    # ------------------------------------------------------------------
    # Phase 2 — Monitor loop: waypoints + CV detections
    # ------------------------------------------------------------------

    def _phase_monitor(self):
        self.shared["focus_status"] = "FLYING"
        self._push_progress()
        self.log("Focus monitor loop started")

        # ── Set transit speed for the journey TO the focus area ────
        # The focus mission only contains focus-area waypoints.  When
        # AUTO starts the drone flies from its current GUIDED position
        # to waypoint 1 — potentially across the entire map.  Flying
        # that transit leg at FOCUS_SPEED (2 m/s) is painfully slow.
        # Instead, start at TRANSIT speed and drop to FOCUS speed once
        # the first waypoint is reached (same pattern SEARCH uses for
        # the SSSI corridor → search transition).
        self._send_speed_change(TRANSIT_SPEED_MPS, label="transit to focus area")
        time.sleep(0.3)
        self._send_speed_change(TRANSIT_SPEED_MPS, label="transit to focus area (reinforce)")
        self.shared["focus_speed_reduced"] = False

        unknown_streak = 0

        while True:
            tick_start = time.time()

            # ── Buffer pump ──
            self.drone.drain_buffer()
            mode_name, _ = self.drone.get_mode(retries=2)
            telem = self.drone.get_telemetry()

            alt = telem.get("alt") or 0.0
            lat = telem.get("lat") or 0.0
            lon = telem.get("lon") or 0.0
            battery_pct = telem.get("battery_pct") or -1
            satellites = telem.get("satellites") or 0

            # ── Waypoint progress ──
            current_wp = self._read_mission_current()
            reached_wp = self._read_mission_reached()

            if current_wp is not None:
                self.shared["focus_current_wp"] = current_wp
            if (reached_wp is not None
                    and reached_wp > self.shared.get("focus_last_reached", -1)):
                self.shared["focus_last_reached"] = reached_wp
                self.log(f"Waypoint {reached_wp} REACHED")

            # ── Speed transition: transit → focus pattern ──
            # Once the first focus waypoint is reached (seq ≥ 1),
            # the drone is inside the focus area — switch to the
            # slower focus speed for the detailed search pattern.
            if (not self.shared.get("focus_speed_reduced")
                    and reached_wp is not None and reached_wp >= 1):
                self._send_speed_change(FOCUS_SPEED_MPS, label="focus pattern")
                self.shared["focus_speed_reduced"] = True

            self._push_progress()

            total = self.shared["focus_total_wps"]
            cur = self.shared["focus_current_wp"]
            self.log(
                f"[FOCUS] WP {cur}/{total} | "
                f"alt={alt:.1f}m | lat={lat:.6f} lon={lon:.6f} | "
                f"sats={satellites} | batt={battery_pct}% | "
                f"mode={mode_name} | elapsed={self._elapsed_str()}"
            )

            # ── RC override detection ──
            if mode_name == "UNKNOWN":
                unknown_streak += 1
                if unknown_streak >= _MODE_LOSS_GRACE:
                    self.log("WARNING: mode UNKNOWN multiple ticks")
            else:
                unknown_streak = 0
                if mode_name != "AUTO":
                    return self._handle_rc_override(mode_name)

            # ── CV detection check ──
            detection = self._check_detections()
            if detection is not None:
                _, _, det_file = detection
                self.log(f"DETECTION IMAGE FOUND: {det_file}")
                # Store only the file for operator confirmation.
                # Geotag coordinates will be obtained from a second
                # stationary-hover image in DELIVER phase 2.
                self.shared["detection_file"] = det_file
                self.shared["focus_status"] = "DETECTION FOUND"
                self._push_progress()
                self._cleanup_focus_keys()
                # Switch to GUIDED → DELIVER for confirmation + geotag + landing
                self.shared["requested_mode"] = "GUIDED"
                self.shared["return_to"] = "DELIVER"
                return "CHANGE_MODE"

            # ── Cancel command ──
            if self._check_cancel():
                self.log("CANCEL received — switching to GUIDED → IDLE")
                write_cv_mode("PASSIVE")
                self.shared["focus_status"] = "CANCELLED"
                self._push_progress()
                self._cleanup_focus_keys()
                self.shared["requested_mode"] = "GUIDED"
                self.shared["return_to"] = "IDLE"
                return "CHANGE_MODE"

            # ── Mission complete (no detection) ──
            if self._is_mission_complete():
                self.log("FOCUS COMPLETE — pattern finished, no detection")
                self.shared["focus_status"] = "COMPLETE (NO DETECTION)"
                self._push_progress()
                self.shared["focus_phase"] = 3
                return "FOCUS"

            # ── Sleep remainder of tick ──
            elapsed = time.time() - tick_start
            sleep_t = max(0.0, _MONITOR_TICK_S - elapsed)
            if sleep_t:
                time.sleep(sleep_t)

    # ------------------------------------------------------------------
    # Phase 3 — Cleanup
    # ------------------------------------------------------------------

    def _phase_cleanup(self):
        status = self.shared.get("focus_status", "UNKNOWN")
        self.log(f"Cleanup — final status: {status}")
        write_cv_mode("PASSIVE")
        self._cleanup_focus_keys()
        # Switch to GUIDED before returning to IDLE (RTH in future)
        self.shared["requested_mode"] = "GUIDED"
        self.shared["return_to"] = "IDLE"
        self.log("FOCUS complete — GUIDED → IDLE")
        return "CHANGE_MODE"

    # ------------------------------------------------------------------
    # RC override handler (identical logic to SEARCH)
    # ------------------------------------------------------------------

    def _handle_rc_override(self, current_mode):
        """
        Safety pilot took over.  Wait for GUIDED before resuming.
        """
        self.log(f"RC OVERRIDE — mode changed to {current_mode}")
        self.log("Focus paused. Waiting for GUIDED to resume.")
        self.shared["focus_status"] = "PAUSED (PILOT OVERRIDE)"
        self._push_progress()

        override_start = time.time()
        while True:
            time.sleep(_MONITOR_TICK_S)
            self.drone.drain_buffer()
            mode_name, _ = self.drone.get_mode(retries=2)
            self.drone.get_telemetry()
            self.log(f"[PAUSED] mode={mode_name}")
            self._push_progress()

            if mode_name == "GUIDED":
                self.log("Pilot returned RC to GUIDED — requesting AUTO to resume")
                self.shared["requested_mode"] = "AUTO"
                self.shared["return_to"] = "FOCUS"
                return "CHANGE_MODE"

            if self._check_cancel():
                self.log("Cancel received during override — "
                         "pilot has control, going straight to IDLE")
                write_cv_mode("PASSIVE")
                self._cleanup_focus_keys()
                return "IDLE"

            if time.time() - override_start > _OVERRIDE_TIMEOUT:
                self.log(f"Override timeout ({_OVERRIDE_TIMEOUT}s) — returning to IDLE")
                write_cv_mode("PASSIVE")
                self._cleanup_focus_keys()
                return "IDLE"

    # ------------------------------------------------------------------
    # CV detection monitoring
    # ------------------------------------------------------------------

    def _check_detections(self):
        """
        Scan DETECTION_DIR for new image files.  Filenames encode the
        geotag as: "NUM_LAT_LON.png" e.g. "3_51.4241_-2.6714.png"
        where NUM is the chronological detection index.

        Also handles the legacy two-field format "LAT_LON.ext".

        Returns (lat, lon, filename) on first detection, or None.
        Stores the detection index in shared["detection_index"] if present.
        """
        try:
            for ext in ("*.png", "*.jpg", "*.jpeg"):
                files = sorted(glob.glob(os.path.join(DETECTION_DIR, ext)))
                for filepath in files:
                    filename = os.path.basename(filepath)
                    name_part = os.path.splitext(filename)[0]

                    # Split on underscore — expect NUM_LAT_LON (3 parts)
                    # or legacy LAT_LON (2 parts).  Always take the last
                    # two parts as lat and lon so the format is forward-
                    # compatible with any future prefix fields.
                    parts = name_part.split("_")
                    if len(parts) >= 2:
                        try:
                            det_lat = float(parts[-2])
                            det_lon = float(parts[-1])
                            if -90 <= det_lat <= 90 and -180 <= det_lon <= 180:
                                # Store chronological index if present
                                if len(parts) >= 3:
                                    try:
                                        self.shared["detection_index"] = int(parts[0])
                                    except ValueError:
                                        pass
                                return (det_lat, det_lon, filename)
                        except ValueError:
                            continue
        except Exception as e:
            self.log(f"WARNING: detection scan error: {e}")

        return None

    def _clear_old_detections(self):
        """Remove any leftover detection files from prior runs."""
        try:
            for ext in ("*.jpg", "*.jpeg", "*.png"):
                for f in glob.glob(os.path.join(DETECTION_DIR, ext)):
                    os.remove(f)
                    self.log(f"Cleared old detection: {os.path.basename(f)}")
        except Exception as e:
            self.log(f"WARNING: could not clear old detections: {e}")

    # ------------------------------------------------------------------
    # MAVLink message readers (same as SEARCH)
    # ------------------------------------------------------------------

    def _read_mission_current(self):
        try:
            msg = self.drone.conn.messages.get("MISSION_CURRENT", None)
            if msg is not None:
                return msg.seq
        except Exception:
            pass
        return None

    def _read_mission_reached(self):
        try:
            msg = self.drone.conn.messages.get("MISSION_ITEM_REACHED", None)
            if msg is not None:
                return msg.seq
        except Exception:
            pass
        return None

    def _is_mission_complete(self):
        total = self.shared.get("focus_total_wps", 0)
        if total == 0:
            return False
        last_reached = self.shared.get("focus_last_reached", -1)
        current_wp = self.shared.get("focus_current_wp", 0)
        # Primary: last nav waypoint was reached
        if last_reached >= total:
            return True
        # Secondary: MISSION_CURRENT wraps to 0 after the last item.
        # Guard: require at least half the waypoints reached before
        # trusting the wrap-around heuristic.  This prevents a false
        # positive when stale MISSION_ITEM_REACHED cache from the
        # previous mission shows reached > 0 while MISSION_CURRENT is
        # still 0 from the waypoint index reset.
        if last_reached >= max(2, total // 2) and current_wp == 0:
            return True
        return False

    # ------------------------------------------------------------------
    # Command checker
    # ------------------------------------------------------------------

    def _check_cancel(self):
        if self.drone.status:
            cmd = self.drone.status.consume_command()
            if cmd and cmd.get("type") == "cancel_focus":
                return True
        return False

    # ------------------------------------------------------------------
    # GUI helpers
    # ------------------------------------------------------------------

    def _push_progress(self):
        if not self.drone.status:
            return
        total = self.shared.get("focus_total_wps", 0)
        current = self.shared.get("focus_current_wp", 0)
        last_reached = self.shared.get("focus_last_reached", 0)
        status = self.shared.get("focus_status", "UNKNOWN")

        progress_pct = min(100, int((max(0, last_reached) / total) * 100)) if total else 0
        telem = self.drone.get_telemetry()

        info = {
            "status": status,
            "current_wp": current,
            "total_wps": total,
            "progress_pct": progress_pct,
            "elapsed": self._elapsed_str(),
            "alt": round(telem.get("alt") or 0.0, 1),
            "lat": round(telem.get("lat") or 0.0, 6),
            "lon": round(telem.get("lon") or 0.0, 6),
            "battery_pct": telem.get("battery_pct") if telem.get("battery_pct") is not None
                           and telem.get("battery_pct") >= 0 else -1,
            "satellites": telem.get("satellites") or 0,
        }
        # Include detection coordinates if found
        if "detection_lat" in self.shared:
            info["detection_lat"] = self.shared["detection_lat"]
            info["detection_lon"] = self.shared["detection_lon"]

        self.drone.status.set_extra("focus_progress", info)

    def _elapsed_str(self):
        start = self.shared.get("focus_start_time")
        if start is None:
            return "0:00"
        elapsed = int(time.time() - start)
        return f"{elapsed // 60}:{elapsed % 60:02d}"

    def _send_speed_change(self, speed_mps, label=""):
        """
        Set ground speed for both AUTO and GUIDED modes.

        MAV_CMD_DO_CHANGE_SPEED (opcode 178) works in AUTO mission mode.
        WPNAV_SPEED is the ArduPilot parameter that governs GUIDED mode
        and persists across missions on the flight controller — if delivery
        left it at 100 cm/s (1 m/s), every subsequent mission will fly at
        that speed unless we explicitly reset it here.
        Both must be set together to ensure the correct speed applies
        regardless of which mode the drone is currently in.
        """
        tag = f" ({label})" if label else ""
        try:
            # AUTO mode: mission speed override
            self.drone.conn.mav.command_long_send(
                self.drone.conn.target_system,
                self.drone.conn.target_component,
                178,        # MAV_CMD_DO_CHANGE_SPEED
                0,          # confirmation
                1,          # param1: speed type 1 = ground speed
                float(speed_mps),
                -1,         # param3: throttle, -1 = no change
                0, 0, 0, 0,
            )
        except Exception as e:
            self.log(f"WARNING: DO_CHANGE_SPEED command failed: {e}")
        try:
            # GUIDED mode: persistent ArduPilot parameter (cm/s).
            # Fire-and-forget — the blocking set_parameter() waits up to
            # 15s for PARAM_VALUE echo which never arrives in SITL under
            # message flooding, stalling the state machine.
            self.drone.set_parameter_nowait("WPNAV_SPEED", speed_mps * 100)
            self.log(f"Speed change{tag}: {speed_mps} m/s (AUTO + GUIDED)")
        except Exception as e:
            self.log(f"WARNING: WPNAV_SPEED parameter set failed: {e}")

    def _cleanup_focus_keys(self):
        """Remove all focus_ prefixed keys from shared dict."""
        for k in [k for k in list(self.shared.keys()) if k.startswith("focus_")]:
            del self.shared[k]
