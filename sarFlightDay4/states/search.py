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

Buffer pattern: drain_buffer() MUST be called first on every loop tick
to pump the pymavlink receive queue, then get_mode() reads from cache.

Author: Blue Co — AENGM0074 flying day
"""

import time
from states.base import BaseState
from config import (
    HEARTBEAT_INTERVAL,
    WP_SPEED_SECOND_PASS_MPS,
    TRANSIT_SPEED_MPS,
    SEARCH_SPEED_MPS,
    SSSI_NAV_TO_SEARCH,
)

# Monitor loop tick: short enough for responsive GUI updates.
# HEARTBEAT_INTERVAL (2s) is too slow — MISSION_CURRENT only fires on
# waypoint change, so we need to drain and push progress frequently.
_MONITOR_TICK_S = 0.5

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

        self.drone.drain_buffer()
        mode_name, _ = self.drone.get_mode(retries=5)
        if mode_name == "AUTO":
            self.log("Already in AUTO — proceeding to monitor phase")
            self.shared["search_phase"] = 2
            self.shared["search_start_time"] = time.time()
            self.shared["search_status"] = "FLYING"
            self._push_progress()
            return "SEARCH"

        # ── GUIDED gate ──────────────────────────────────────────
        # The RC switch is LOITER / GUIDED / LOITER.  We must only
        # request AUTO when the RC is in GUIDED — this confirms the
        # pilot has deliberately handed control to the state machine.
        # If the mode is anything else (e.g. LOITER), the pilot has
        # not given consent, so we wait rather than fighting the RC.
        if mode_name != "GUIDED":
            self.log(f"Mode is {mode_name}, not GUIDED — "
                     f"waiting for pilot to set GUIDED before switching to AUTO")
            self.shared["search_status"] = "WAITING FOR GUIDED"
            self._push_progress()
            while True:
                time.sleep(_MONITOR_TICK_S)
                self.drone.drain_buffer()
                mode_name, _ = self.drone.get_mode(retries=2)
                self.drone.get_telemetry()
                self.log(f"[WAIT-GUIDED] mode={mode_name}")
                self._push_progress()

                if mode_name == "GUIDED":
                    self.log("Pilot set GUIDED — proceeding to request AUTO")
                    break

                if self._check_cancel():
                    self.log("Cancel received while waiting for GUIDED")
                    self._cleanup_search_keys()
                    return "IDLE"

        # Speed is set at the start of _phase_monitor(), AFTER AUTO is
        # confirmed.  DO_CHANGE_SPEED is ignored in GUIDED mode, so
        # sending it here (before the CHANGE_MODE→AUTO handoff) has no
        # effect.  WPNAV_SPEED (fire-and-forget) is safe to send in
        # GUIDED, but we keep both together for clarity.

        # Delegate mode switch to CHANGE_MODE
        self.shared["requested_mode"] = "AUTO"
        self.shared["return_to"]      = "SEARCH"
        self.shared["search_phase"]   = 2
        self.shared["search_start_time"] = time.time()
        self.shared["search_status"]  = "SWITCHING TO AUTO"
        self._push_progress()
        self.log("Requesting AUTO mode via CHANGE_MODE")
        return "CHANGE_MODE"

    # ------------------------------------------------------------------
    # Phase 2 — Main monitoring loop
    # ------------------------------------------------------------------

    def _phase_monitor(self):
        self.shared["search_status"] = "FLYING"
        self._push_progress()
        pass_label = "PASS 2" if self.shared.get("search_second_pass") else "PASS 1"
        self.log(f"Monitor loop started — {pass_label}")

        # ── Set speed NOW that AUTO is confirmed ──────────────────
        # DO_CHANGE_SPEED only works in AUTO mode.  Phase 1 cannot send
        # it because the drone is still in GUIDED at that point.
        # On pass 1 the drone flies the SSSI corridor first, so we start
        # at TRANSIT speed; the corridor→search transition in the loop
        # below will drop to SEARCH speed once the corridor is cleared.
        # On pass 2, _phase_second_pass_setup already sent a speed cmd.
        if not self.shared.get("search_second_pass"):
            self._send_speed_change(TRANSIT_SPEED_MPS, label="transit")

        unknown_streak = 0

        while True:
            tick_start = time.time()

            # ── Buffer pump (MUST be first) ──
            self.drone.drain_buffer()
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

            self._push_progress()

            # ── Speed transition: corridor → search pattern ──
            # The uploaded mission is SSSI_NAV_TO_SEARCH (N waypoints) followed
            # by SEARCH_WAYPOINTS.  Item 0 = home, nav items start at seq 1.
            # When MISSION_CURRENT.seq exceeds len(SSSI_NAV_TO_SEARCH) we have
            # entered the search pattern — drop from transit speed to search speed.
            if (not self.shared.get("search_speed_reduced")
                    and not self.shared.get("search_second_pass")
                    and current_wp is not None
                    and current_wp > len(SSSI_NAV_TO_SEARCH)):
                self._send_speed_change(SEARCH_SPEED_MPS, label="search pattern")
                self.shared["search_speed_reduced"] = True

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
                return "CHANGE_MODE"

            # ── PLB button ──
            if self._check_plb():
                self.log("PLB ACTIVATED — switching to GUIDED → REPLAN")
                self.shared["plb_triggered"]  = True
                self.shared["search_status"]  = "PLB ACTIVATED"
                self._push_progress()
                self._cleanup_search_keys()
                self.shared["requested_mode"] = "GUIDED"
                self.shared["return_to"]      = "REPLAN"
                return "CHANGE_MODE"

            # ── Mission complete ──
            if self._is_mission_complete():
                if self.shared.get("search_second_pass"):
                    self.log("SEARCH COMPLETE — second pass finished")
                    self.shared["search_status"] = "COMPLETE"
                    self._push_progress()
                    self.shared["search_phase"] = 4
                    return "SEARCH"
                else:
                    self.log("PASS 1 COMPLETE — beginning second pass setup")
                    self.shared["search_status"] = "PASS 1 COMPLETE"
                    self._push_progress()
                    self.shared["search_phase"] = 3
                    return "SEARCH"

            # ── Sleep remainder of tick ──
            elapsed = time.time() - tick_start
            sleep_t = max(0.0, _MONITOR_TICK_S - elapsed)
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

        # ── Request speed change for second pass ──
        # Use _send_speed_change for consistency — sets both
        # DO_CHANGE_SPEED (AUTO) and WPNAV_SPEED (GUIDED).
        self._send_speed_change(WP_SPEED_SECOND_PASS_MPS, label="second pass")

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
        Never fight the pilot — do NOT send any SET_MODE commands.

        Resume logic:
            The RC switch is LOITER / GUIDED / LOITER.  The pilot takes
            over by flipping to LOITER.  To hand back, the pilot returns
            the switch to GUIDED.  Only when we see GUIDED do we request
            AUTO via CHANGE_MODE and resume the search.

            We never resume on seeing AUTO directly (the state machine is
            the only thing that requests AUTO, so seeing AUTO here would
            mean we fought the pilot — which must not happen).

        Timeout:
            If GUIDED is not seen within OVERRIDE_TIMEOUT seconds, we
            give up and return to IDLE.
        """
        self.log(f"RC OVERRIDE — mode changed to {current_mode}")
        self.log("Search paused. Safety pilot has control.")
        self.log("Waiting for pilot to return RC to GUIDED before resuming.")
        self.shared["search_status"] = "PAUSED (PILOT OVERRIDE)"
        self._push_progress()

        override_start = time.time()
        while True:
            time.sleep(_MONITOR_TICK_S)
            self.drone.drain_buffer()
            mode_name, _ = self.drone.get_mode(retries=2)
            self.drone.get_telemetry()   # keep cache fresh
            self.log(f"[PAUSED] mode={mode_name}")
            self._push_progress()

            if mode_name == "GUIDED":
                self.log("Pilot returned RC to GUIDED — requesting AUTO to resume search")
                self.shared["requested_mode"] = "AUTO"
                self.shared["return_to"]      = "SEARCH"
                # Stay in current phase (2) so monitor loop resumes
                return "CHANGE_MODE"

            if self._check_cancel():
                self.log("Cancel received during override — "
                         "pilot has control, going straight to IDLE")
                self._cleanup_search_keys()
                return "IDLE"

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
        # Secondary: MISSION_CURRENT wraps to 0 after the last item.
        # Guard: require at least half the waypoints reached before
        # trusting the wrap-around heuristic.  This prevents a false
        # positive when stale MISSION_ITEM_REACHED cache from a
        # previous mission shows reached > 0 while MISSION_CURRENT is
        # still 0 from the waypoint index reset.
        if last_reached >= max(2, total // 2) and current_wp == 0:
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

    def _cleanup_search_keys(self):
        """Remove all search_ prefixed keys from shared dict."""
        for k in [k for k in list(self.shared.keys()) if k.startswith("search_")]:
            del self.shared[k]
