"""
states/search.py — SEARCH state (flying day).
===============================================
Monitors the drone during autonomous waypoint flight in AUTO mode.
Tracks mission progress, handles two operator exit conditions
(manual cancel and PLB stub), detects safety pilot override, and
manages a second pass if all waypoints are visited before any exit
condition is triggered.

Phase structure (shared["search_phase"]):
    Phase 1 — Verify mission on autopilot, request AUTO via CHANGE_MODE
    Phase 2 — Main monitoring loop (core of SEARCH)
    Phase 3 — Second-pass setup: reduce speed, re-upload, back to AUTO
    Phase 4 — Cleanup and return to IDLE

Full state chain (pass 1):
    PRE_AUTO_CHECK → SEARCH(1) → CHANGE_MODE → SEARCH(2) → ... → SEARCH(3)
    → CHANGE_MODE(LOITER) → UPLOAD_MISSION → CHANGE_MODE(AUTO) → SEARCH(2)
    → ... → SEARCH(4) → IDLE

Exit conditions from phase 2:
    a. Operator presses CANCEL MISSION  → CHANGE_MODE(LOITER) → IDLE
    b. Operator presses PLB ACTIVATED   → CHANGE_MODE(LOITER) → IDLE
       (plb_triggered=True stored in shared for future REPLAN)
    c. RC override (mode != AUTO)       → IDLE directly
    d. All WPs visited, pass 1          → phase 3 (second pass setup)
    e. All WPs visited, pass 2          → phase 4 (cleanup → IDLE)

Buffer pattern: get_mode() MUST be called first on every loop tick
to pump the pymavlink receive buffer before reading cached messages.

Author: Blue Co — AENGM0074 flying day
"""

import time
import mission_log
from states.base import BaseState
from config import HEARTBEAT_INTERVAL, WP_SPEED_SECOND_PASS_MPS

# How many consecutive UNKNOWN mode readings to tolerate before acting
_MODE_LOSS_GRACE = 2

# Max seconds to wait in pilot-override pause before returning to IDLE
_OVERRIDE_TIMEOUT = 120


class SearchState(BaseState):
    """AUTO mode flight monitor with second-pass and PLB stub."""

    name = "SEARCH"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enter(self):
        if "search_phase" not in self.shared:
            self.shared["search_phase"] = 1
            self.shared["search_start_time"] = None
            self.shared["search_total_wps"] = 0
            self.shared["search_current_wp"] = 0
            self.shared["search_last_reached"] = -1
            self.shared["search_status"] = "INITIALISING"
            self.shared["search_second_pass"] = False
        phase = self.shared["search_phase"]
        self.log(f"Entered SEARCH phase {phase} "
                 f"({'pass 2' if self.shared.get('search_second_pass') else 'pass 1'})")

    def execute(self):
        phase = self.shared["search_phase"]
        if   phase == 1: return self._phase_verify()
        elif phase == 2: return self._phase_monitor()
        elif phase == 3: return self._phase_second_pass_setup()
        elif phase == 4: return self._phase_cleanup()
        else:
            self.log(f"ERROR: unknown phase {phase}")
            self.shared["search_phase"] = 4
            return "SEARCH"

    def exit(self):
        pass

    # ------------------------------------------------------------------
    # Phase 1 — Verify mission, request AUTO
    # ------------------------------------------------------------------

    def _phase_verify(self):
        # Query mission count directly from autopilot.
        # NOTE: upload_mission.py pops shared["waypoints"] after upload,
        # so we CANNOT rely on shared state — query the autopilot.
        wp_count = 0
        try:
            wp_count = self.drone.query_mission_count(timeout=5)
            self.log(f"Autopilot reports {wp_count} mission items")
        except Exception as e:
            self.log(f"ERROR querying mission count: {e}")

        if wp_count < 2:
            self.log("ERROR: no valid mission on autopilot — returning to IDLE")
            self.shared["search_status"] = "NO MISSION"
            self._push_progress()
            self.shared["search_phase"] = 4
            return "SEARCH"

        # Item 0 = home, items 1..N = nav waypoints
        self.shared["search_total_wps"] = wp_count - 1
        self.log(f"Mission verified: {self.shared['search_total_wps']} nav waypoints")

        mode_name, _ = self.drone.get_mode(retries=5)
        if mode_name == "AUTO":
            self.log("Already in AUTO — proceeding to monitor phase")
            self.shared["search_phase"] = 2
            self.shared["search_start_time"] = time.time()
            self.shared["search_status"] = "FLYING"
            self._push_progress()
            return "SEARCH"

        # Delegate mode switch to CHANGE_MODE
        self.shared["requested_mode"] = "AUTO"
        self.shared["return_to"]      = "SEARCH"
        self.shared["search_phase"]   = 2
        self.shared["search_start_time"] = time.time()
        self.shared["search_status"]  = "SWITCHING TO AUTO"
        self._push_progress()
        self.log("Requesting AUTO mode via CHANGE_MODE")
        mission_log.mode_cmd("AUTO")
        return "CHANGE_MODE"

    # ------------------------------------------------------------------
    # Phase 2 — Main monitoring loop
    # ------------------------------------------------------------------

    def _phase_monitor(self):
        self.shared["search_status"] = "FLYING"
        self._push_progress()
        pass_label = "PASS 2" if self.shared.get("search_second_pass") else "PASS 1"
        self.log(f"Monitor loop started — {pass_label}")

        unknown_streak = 0

        while True:
            tick_start = time.time()

            # ── Buffer drain (MUST be first) ──
            self.drone.drain_buffer()

            # ── Mode (reads from cache after drain) ──
            mode_name, _ = self.drone.get_mode(retries=2)

            # ── Telemetry ──
            telem        = self.drone.get_telemetry()
            alt          = telem.get("alt") or 0.0
            lat          = telem.get("lat") or 0.0
            lon          = telem.get("lon") or 0.0
            battery_pct  = telem.get("battery_pct") or -1
            satellites   = telem.get("satellites") or 0

            # ── Waypoint progress from MAVLink cache ──
            current_wp = self._read_mission_current()
            reached_wp = self._read_mission_reached()

            if current_wp is not None:
                self.shared["search_current_wp"] = current_wp
            if (reached_wp is not None
                    and reached_wp > self.shared.get("search_last_reached", -1)):
                self.shared["search_last_reached"] = reached_wp
                self.log(f"Waypoint {reached_wp} REACHED")
                mission_log.waypoint(reached_wp, self.shared["search_total_wps"])

            self._push_progress()

            # ── Log summary ──
            total = self.shared["search_total_wps"]
            cur   = self.shared["search_current_wp"]
            self.log(
                f"[{pass_label}] WP {cur}/{total} | "
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

            # ── Cancel command ──
            if self._check_cancel():
                self.log("CANCEL MISSION received — switching to GUIDED → IDLE")
                self.shared["search_status"] = "CANCELLED"
                self._push_progress()
                self._cleanup_search_keys()
                self.shared["requested_mode"] = "GUIDED"
                self.shared["return_to"]      = "IDLE"
                import mission_log; mission_log.event("SEARCH", "Mission cancelled by operator")
                return "CHANGE_MODE"

            # ── PLB button ──
            if self._check_plb():
                self.log("PLB ACTIVATED — switching to GUIDED → IDLE (PLB flag stored)")
                self.shared["plb_triggered"]  = True
                self.shared["search_status"]  = "PLB ACTIVATED"
                self._push_progress()
                self._cleanup_search_keys()
                self.shared["requested_mode"] = "GUIDED"
                self.shared["return_to"]      = "IDLE"
                import mission_log; mission_log.event("SEARCH", "PLB activated — returning to IDLE in GUIDED")
                return "CHANGE_MODE"

            # ── Mission complete ──
            if self._is_mission_complete():
                if self.shared.get("search_second_pass"):
                    self.log("SEARCH COMPLETE — second pass finished")
                    self.shared["search_status"] = "COMPLETE"
                    self._push_progress()
                    mission_log.event("SEARCH", "Pass 2 complete — mission finished")
                    self.shared["search_phase"] = 4
                    return "SEARCH"
                else:
                    self.log("PASS 1 COMPLETE — beginning second pass setup")
                    self.shared["search_status"] = "PASS 1 COMPLETE"
                    self._push_progress()
                    mission_log.event("SEARCH", "Pass 1 complete — starting Pass 2")
                    self.shared["search_phase"] = 3
                    return "SEARCH"

            # ── Sleep remainder of tick ──
            elapsed = time.time() - tick_start
            sleep_t = max(0.0, HEARTBEAT_INTERVAL - elapsed)
            if sleep_t:
                time.sleep(sleep_t)

    # ------------------------------------------------------------------
    # Phase 3 — Second-pass setup
    # ------------------------------------------------------------------

    def _phase_second_pass_setup(self):
        """
        Re-upload the same mission at reduced speed, then return to AUTO.

        Speed change: send MAV_CMD_DO_CHANGE_SPEED before uploading so
        ArduPilot picks it up as the first mission item.  Falls back
        gracefully if the command is not supported on bench.

        Reset waypoint progress counters so phase 2 can reuse them.
        """
        self.log(f"Second pass setup — target speed {WP_SPEED_SECOND_PASS_MPS} m/s")

        # ── Notify operator ──
        self.shared["search_status"] = "STARTING PASS 2"
        self.shared["search_second_pass"] = True
        # Reset progress counters for the new pass
        self.shared["search_current_wp"]   = 0
        self.shared["search_last_reached"] = -1
        self._push_progress()

        # ── Retrieve stored waypoints for re-upload ──
        # GENERATE_PATTERN → IDLE stored them in shared before confirm
        # but UPLOAD_MISSION pops shared["waypoints"].
        # We re-query the autopilot and use stored polygon data if available.
        # The safest approach: store original waypoints in shared during confirm.
        # check shared["original_waypoints"] set by idle._confirm_pattern below.
        waypoints = self.shared.get("original_waypoints")
        if not waypoints:
            self.log("WARNING: no original_waypoints in shared — "
                     "re-querying autopilot item list (not yet implemented). "
                     "Skipping second pass.")
            self.shared["search_phase"] = 4
            return "SEARCH"

        # ── Request speed change via MAV_CMD_DO_CHANGE_SPEED ──
        try:
            self.drone.conn.mav.command_long_send(
                self.drone.conn.target_system,
                self.drone.conn.target_component,
                178,               # MAV_CMD_DO_CHANGE_SPEED
                0,                 # confirmation
                1,                 # speed type: 1 = ground speed
                WP_SPEED_SECOND_PASS_MPS,
                -1,                # throttle (-1 = no change)
                0, 0, 0, 0,
            )
            self.log(f"Speed change command sent: {WP_SPEED_SECOND_PASS_MPS} m/s")
        except Exception as e:
            self.log(f"WARNING: speed change command failed: {e}")

        # ── Re-upload mission ──
        # After UPLOAD_MISSION completes it returns to "SEARCH".
        # We set search_phase = 1 so SEARCH re-enters the verify step,
        # which will detect mode != AUTO and request it via CHANGE_MODE.
        self.shared["waypoints"]       = waypoints
        self.shared["return_to"]       = "SEARCH"
        self.shared["search_phase"]    = 1          # re-verify + re-request AUTO
        self.shared["search_start_time"] = time.time()
        self.log(f"Re-uploading {len(waypoints)} waypoints for second pass")
        return "UPLOAD_MISSION"

    # ------------------------------------------------------------------
    # Phase 4 — Cleanup
    # ------------------------------------------------------------------

    def _phase_cleanup(self):
        status = self.shared.get("search_status", "UNKNOWN")
        self.log(f"Cleanup — final status: {status}")
        if self.drone.status:
            self.drone.status.set_extra("search_progress", None)
        self._cleanup_search_keys()
        self.log("SEARCH complete — returning to IDLE")
        return "IDLE"

    # ------------------------------------------------------------------
    # RC override handler
    # ------------------------------------------------------------------

    def _handle_rc_override(self, current_mode):
        """
        Safety pilot switched away from AUTO.
        Never fight the pilot.  Wait up to OVERRIDE_TIMEOUT seconds
        for AUTO to return, then give up and return to IDLE.
        """
        self.log(f"RC OVERRIDE — mode changed to {current_mode}")
        self.log("Search paused. Safety pilot has control.")
        mission_log.mode_override("AUTO", current_mode)
        self.shared["search_status"] = "PAUSED (PILOT OVERRIDE)"
        self._push_progress()

        override_start = time.time()
        while True:
            time.sleep(HEARTBEAT_INTERVAL)
            self.drone.drain_buffer()
            mode_name, _ = self.drone.get_mode(retries=2)
            self.log(f"[PAUSED] mode={mode_name}")
            self._push_progress()

            if mode_name == "AUTO":
                self.log("Mode returned to AUTO — resuming")
                self.shared["search_status"] = "FLYING"
                return self._phase_monitor()

            if self._check_cancel():
                self.log("Cancel received during override")
                self._cleanup_search_keys()
                self.shared["requested_mode"] = "GUIDED"
                self.shared["return_to"]      = "IDLE"
                return "CHANGE_MODE"

            if time.time() - override_start > _OVERRIDE_TIMEOUT:
                self.log(f"Override timeout ({_OVERRIDE_TIMEOUT}s) — returning to IDLE")
                self._cleanup_search_keys()
                return "IDLE"

    # ------------------------------------------------------------------
    # MAVLink message readers
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

    # ------------------------------------------------------------------
    # Mission complete check
    # ------------------------------------------------------------------

    def _is_mission_complete(self):
        total = self.shared.get("search_total_wps", 0)
        if total == 0:
            return False
        last_reached = self.shared.get("search_last_reached", -1)
        current_wp   = self.shared.get("search_current_wp", 0)
        # Primary: last nav waypoint was reached
        if last_reached >= total:
            return True
        # Secondary: MISSION_CURRENT wraps to 0 after the last item
        if last_reached > 0 and current_wp == 0:
            return True
        return False

    # ------------------------------------------------------------------
    # Command checkers
    # ------------------------------------------------------------------

    def _check_cancel(self):
        if self.drone.status:
            cmd = self.drone.status.consume_command()
            if cmd and cmd.get("type") == "cancel_search":
                return True
        try:
            import os
            if os.path.exists("/tmp/sar_trigger.txt"):
                with open("/tmp/sar_trigger.txt") as f:
                    content = f.read().strip().upper()
                os.remove("/tmp/sar_trigger.txt")
                if content == "ABORT_SEARCH":
                    return True
        except Exception:
            pass
        return False

    def _check_plb(self):
        """Check for PLB_ACTIVATED command from the GUI."""
        if self.drone.status:
            # Peek at status extra without consuming the command queue
            plb_flag = self.drone.status.get_extra("plb_button_pressed", False)
            if plb_flag:
                # Clear the flag
                self.drone.status.set_extra("plb_button_pressed", False)
                return True
        return False

    # ------------------------------------------------------------------
    # GUI helpers
    # ------------------------------------------------------------------

    def _push_progress(self):
        if not self.drone.status:
            return
        total        = self.shared.get("search_total_wps", 0)
        current      = self.shared.get("search_current_wp", 0)
        last_reached = self.shared.get("search_last_reached", 0)
        status       = self.shared.get("search_status", "UNKNOWN")
        second_pass  = self.shared.get("search_second_pass", False)

        progress_pct = min(100, int((max(0, last_reached) / total) * 100)) if total else 0
        telem = self.drone.get_telemetry()

        self.drone.status.set_extra("search_progress", {
            "status":       status,
            "current_wp":   current,
            "total_wps":    total,
            "progress_pct": progress_pct,
            "second_pass":  second_pass,
            "elapsed":      self._elapsed_str(),
            "alt":          round(telem.get("alt") or 0.0, 1),
            "lat":          round(telem.get("lat") or 0.0, 6),
            "lon":          round(telem.get("lon") or 0.0, 6),
            "battery_pct":  telem.get("battery_pct") if telem.get("battery_pct") is not None
                            and telem.get("battery_pct") >= 0 else -1,
            "satellites":   telem.get("satellites") or 0,
        })

    def _elapsed_str(self):
        start = self.shared.get("search_start_time")
        if start is None:
            return "0:00"
        elapsed = int(time.time() - start)
        return f"{elapsed // 60}:{elapsed % 60:02d}"

    def _cleanup_search_keys(self):
        """Remove all search_ prefixed keys from shared dict."""
        for k in [k for k in list(self.shared.keys()) if k.startswith("search_")]:
            del self.shared[k]
