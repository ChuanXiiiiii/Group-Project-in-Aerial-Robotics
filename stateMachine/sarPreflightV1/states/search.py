"""
states/search.py — SEARCH state (Step 5: AUTO Mode Monitoring).
================================================================

Monitors the drone during autonomous waypoint flight in AUTO mode.
Tracks waypoint progress via MISSION_CURRENT / MISSION_ITEM_REACHED
MAVLink messages and pushes real-time progress to the dashboard.

Entry:  Operator has armed, taken off manually, and clicked START SEARCH.
        Mission must already be uploaded (query_mission_count confirms).
Exit:   Last waypoint reached → returns to WAIT.
        Safety pilot override (mode != AUTO) → pauses, then resumes or aborts.

Uses the phase-based re-entry pattern from TIMED_HOLD:
  Phase 1 — Verify mission & switch to AUTO via CHANGE_MODE
  Phase 2 — Monitor loop (core of SEARCH)
  Phase 3 — Cleanup and return to WAIT

Buffer pattern: get_mode() FIRST on every loop tick (pumps recv buffer),
then read telemetry + mission progress from pymavlink cache.

Full state chain:
    WAIT → SEARCH(1) → CHANGE_MODE → SEARCH(2) → ... → SEARCH(3) → WAIT

Author: Blue Co — AENGM0074
"""

import time
from states.base import BaseState
from config import HEARTBEAT_INTERVAL


# Search-specific constants
SEARCH_POLL_INTERVAL = HEARTBEAT_INTERVAL   # seconds between monitor ticks
MODE_LOSS_GRACE_TICKS = 2                   # allow N UNKNOWN readings before warning
OVERRIDE_TIMEOUT = 120                      # max seconds in PAUSED before auto-abort


class SearchState(BaseState):
    """AUTO mode flight monitor — tracks waypoint progress during search."""

    name = "SEARCH"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enter(self):
        """Initialise phase counter on first entry (not on re-entry)."""
        if "search_phase" not in self.shared:
            self.shared["search_phase"] = 1
            self.shared["search_start_time"] = None
            self.shared["search_total_wps"] = 0
            self.shared["search_current_wp"] = 0
            self.shared["search_last_reached"] = -1
            self.shared["search_status"] = "INITIALISING"
        phase = self.shared["search_phase"]
        self.log(f"Entered SEARCH phase {phase}")

    def execute(self):
        """Route to the current phase. Returns next state name string."""
        phase = self.shared["search_phase"]

        if phase == 1:
            return self._phase_verify_and_arm()
        elif phase == 2:
            return self._phase_monitor()
        elif phase == 3:
            return self._phase_cleanup()
        else:
            self.log(f"ERROR: Unknown phase {phase}, cleaning up")
            self.shared["search_phase"] = 3
            return "SEARCH"

    def exit(self):
        """Called on every transition out (including to CHANGE_MODE)."""
        pass

    # ------------------------------------------------------------------
    # Phase 1 — Verify mission uploaded, request AUTO mode
    # ------------------------------------------------------------------

    def _phase_verify_and_arm(self):
        """Check mission is on the autopilot, then delegate to CHANGE_MODE."""

        # --- Verify mission exists on the autopilot ---
        # NOTE: upload_mission.py pops shared["waypoints"] after upload,
        # so we can't rely on it persisting. Query the autopilot directly.
        wp_count = 0
        try:
            wp_count = self.drone.query_mission_count(timeout=5)
            self.log(f"Autopilot reports {wp_count} mission items")
        except Exception as e:
            self.log(f"ERROR: Failed to query mission count: {e}")
            wp_count = 0

        if wp_count < 2:
            # Need at least home (item 0) + 1 nav waypoint
            self.log("ERROR: No valid mission on autopilot. Returning to WAIT.")
            self.shared["search_status"] = "NO MISSION"
            self._push_progress()
            self.shared["search_phase"] = 3
            return "SEARCH"

        # Store total waypoints (excluding home item 0)
        # ArduPilot mission: item 0 = home, items 1..N = nav waypoints
        self.shared["search_total_wps"] = wp_count - 1
        self.log(f"Mission verified: {self.shared['search_total_wps']} nav waypoints")

        # --- Check current mode before requesting AUTO ---
        mode_name, _ = self.drone.get_mode(retries=5)
        self.log(f"Current mode: {mode_name}")

        if mode_name == "AUTO":
            # Already in AUTO — skip CHANGE_MODE, go straight to monitor
            self.log("Already in AUTO mode, proceeding to monitor phase")
            self.shared["search_phase"] = 2
            self.shared["search_start_time"] = time.time()
            self.shared["search_status"] = "FLYING"
            self._push_progress()
            return "SEARCH"

        # --- Delegate to CHANGE_MODE ---
        self.shared["requested_mode"] = "AUTO"
        self.shared["return_to"] = "SEARCH"
        self.shared["search_phase"] = 2        # next re-entry → monitor
        self.shared["search_start_time"] = time.time()
        self.shared["search_status"] = "SWITCHING TO AUTO"
        self._push_progress()
        self.log("Requesting AUTO mode via CHANGE_MODE")
        return "CHANGE_MODE"

    # ------------------------------------------------------------------
    # Phase 2 — Monitoring loop (core of SEARCH)
    # ------------------------------------------------------------------

    def _phase_monitor(self):
        """
        Poll loop: read mode, telemetry, waypoint progress every tick.
        Exits when last waypoint reached or unrecoverable error.
        """
        self.shared["search_status"] = "FLYING"
        self._push_progress()
        self.log("Monitor loop started — tracking waypoint progress")

        unknown_streak = 0

        while True:
            tick_start = time.time()

            # ---- 1. Pump buffer via get_mode() (MUST be first) ----
            mode_name, mode_num = self.drone.get_mode(retries=2)

            # ---- 2. Read telemetry from cache ----
            telem = self.drone.get_telemetry()
            alt = telem.get("alt") or 0.0
            lat = telem.get("lat") or 0.0
            lon = telem.get("lon") or 0.0
            battery_pct = telem.get("battery_pct") or -1
            satellites = telem.get("satellites") or 0

            # ---- 3. Read waypoint progress from pymavlink cache ----
            current_wp = self._read_mission_current()
            reached_wp = self._read_mission_reached()

            if current_wp is not None:
                self.shared["search_current_wp"] = current_wp
            if reached_wp is not None and reached_wp > self.shared.get("search_last_reached", -1):
                self.shared["search_last_reached"] = reached_wp
                self.log(f"Waypoint {reached_wp} REACHED")

            # ---- 4. Push progress to dashboard ----
            self._push_progress()

            # ---- 5. Log telemetry summary ----
            total = self.shared["search_total_wps"]
            cur = self.shared["search_current_wp"]
            elapsed = self._elapsed_str()
            self.log(
                f"WP {cur}/{total} | "
                f"alt={alt:.1f}m | lat={lat:.6f} lon={lon:.6f} | "
                f"sats={satellites} | batt={battery_pct}% | "
                f"mode={mode_name} | elapsed={elapsed}"
            )

            # ---- 6. Check for mode override (safety pilot) ----
            if mode_name == "UNKNOWN":
                unknown_streak += 1
                if unknown_streak >= MODE_LOSS_GRACE_TICKS:
                    self.log("WARNING: Mode UNKNOWN for multiple ticks")
            else:
                unknown_streak = 0
                if mode_name != "AUTO":
                    return self._handle_pilot_override(mode_name)

            # ---- 7. Check for mission completion ----
            if self._is_mission_complete():
                self.log("SEARCH COMPLETE — all waypoints reached")
                self.shared["search_status"] = "COMPLETE"
                self._push_progress()
                self.shared["search_phase"] = 3
                return "SEARCH"

            # ---- 8. Check for abort command from GUI ----
            if self._check_abort_command():
                self.log("SEARCH ABORTED by operator command")
                self.shared["search_status"] = "ABORTED"
                self._push_progress()
                self.shared["search_phase"] = 3
                return "SEARCH"

            # ---- Sleep remainder of tick ----
            tick_elapsed = time.time() - tick_start
            sleep_time = max(0, SEARCH_POLL_INTERVAL - tick_elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------
    # Phase 3 — Cleanup
    # ------------------------------------------------------------------

    def _phase_cleanup(self):
        """Remove all search_ prefixed keys from shared dict, return to WAIT."""
        status = self.shared.get("search_status", "UNKNOWN")
        self.log(f"Cleanup — final status: {status}")

        # Clear search progress from GUI extras
        if self.drone.status:
            self.drone.status.set_extra("search_progress", None)

        # Remove all search_ keys from shared dict
        keys_to_remove = [k for k in list(self.shared.keys())
                          if k.startswith("search_")]
        for k in keys_to_remove:
            del self.shared[k]

        self.log("Search state data cleared. Returning to WAIT.")
        return "WAIT"

    # ------------------------------------------------------------------
    # Helpers — Waypoint progress reading
    # ------------------------------------------------------------------

    def _read_mission_current(self):
        """
        Read MISSION_CURRENT.seq from pymavlink message cache.
        Returns the current target waypoint number, or None.

        MISSION_CURRENT is streamed by ArduPilot whenever the target
        waypoint changes.  Cached automatically by pymavlink since
        get_mode()'s recv_match() pumps the buffer.
        """
        try:
            msg = self.drone.conn.messages.get("MISSION_CURRENT", None)
            if msg is not None:
                return msg.seq
        except Exception:
            pass
        return None

    def _read_mission_reached(self):
        """
        Read MISSION_ITEM_REACHED.seq from pymavlink message cache.
        Returns the most recently reached waypoint number, or None.

        MISSION_ITEM_REACHED fires once per waypoint arrival.
        The cached version is the LAST one received.
        """
        try:
            msg = self.drone.conn.messages.get("MISSION_ITEM_REACHED", None)
            if msg is not None:
                return msg.seq
        except Exception:
            pass
        return None

    def _is_mission_complete(self):
        """
        Determine if the mission is complete.

        Total mission items = search_total_wps + 1 (including home).
        The last nav waypoint index = search_total_wps.
        """
        total = self.shared.get("search_total_wps", 0)
        if total == 0:
            return False

        last_reached = self.shared.get("search_last_reached", -1)
        current_wp = self.shared.get("search_current_wp", 0)

        # Primary: last waypoint in the mission was reached
        if last_reached >= total:
            return True

        # Secondary: MISSION_CURRENT wraps to 0 after the last item,
        # but only if we've actually flown at least one waypoint
        if last_reached > 0 and current_wp == 0:
            return True

        return False

    # ------------------------------------------------------------------
    # Helpers — Safety pilot override
    # ------------------------------------------------------------------

    def _handle_pilot_override(self, current_mode):
        """
        Safety pilot has switched away from AUTO.

        CRITICAL SAFETY RULE: Never fight the pilot. Log the override,
        update status, and wait. If AUTO returns, resume. Otherwise
        timeout and return to WAIT.
        """
        self.log(f"PILOT OVERRIDE — mode changed to {current_mode}")
        self.log("Search paused. Safety pilot has control.")
        self.shared["search_status"] = "PAUSED (PILOT OVERRIDE)"
        self._push_progress()

        override_start = time.time()

        while True:
            time.sleep(SEARCH_POLL_INTERVAL)

            mode_name, _ = self.drone.get_mode(retries=2)
            telem = self.drone.get_telemetry()   # keep cache fresh

            self.log(f"[PAUSED] mode={mode_name} | waiting for AUTO or abort")
            self._push_progress()

            # Resume if back to AUTO
            if mode_name == "AUTO":
                self.log("Mode returned to AUTO — resuming search")
                self.shared["search_status"] = "FLYING"
                self._push_progress()
                return self._phase_monitor()

            # Abort command from GUI
            if self._check_abort_command():
                self.log("Abort command received during pilot override")
                self.shared["search_status"] = "ABORTED"
                self._push_progress()
                self.shared["search_phase"] = 3
                return "SEARCH"

            # Timeout
            if time.time() - override_start > OVERRIDE_TIMEOUT:
                self.log(f"Override timeout ({OVERRIDE_TIMEOUT}s). "
                         f"Returning to WAIT.")
                self.shared["search_status"] = "TIMEOUT"
                self._push_progress()
                self.shared["search_phase"] = 3
                return "SEARCH"

    # ------------------------------------------------------------------
    # Helpers — GUI communication
    # ------------------------------------------------------------------

    def _push_progress(self):
        """
        Push search progress to SharedStatus for dashboard display.
        Uses set_extra("search_progress", {...}) which merges into
        the /api/status JSON response via get_status().
        """
        if not self.drone.status:
            return

        total = self.shared.get("search_total_wps", 0)
        current = self.shared.get("search_current_wp", 0)
        status = self.shared.get("search_status", "UNKNOWN")
        last_reached = self.shared.get("search_last_reached", 0)

        # Progress percentage based on reached waypoints
        if total > 0:
            progress_pct = min(100, int((max(0, last_reached) / total) * 100))
        else:
            progress_pct = 0

        telem = self.drone.get_telemetry()

        search_data = {
            "status": status,
            "current_wp": current,
            "total_wps": total,
            "progress_pct": progress_pct,
            "elapsed": self._elapsed_str(),
            "alt": round(telem.get("alt") or 0.0, 1),
            "lat": round(telem.get("lat") or 0.0, 6),
            "lon": round(telem.get("lon") or 0.0, 6),
            "battery_pct": telem.get("battery_pct") if telem.get("battery_pct") is not None and telem.get("battery_pct") >= 0 else -1,
            "satellites": telem.get("satellites") or 0,
        }

        self.drone.status.set_extra("search_progress", search_data)

    def _elapsed_str(self):
        """Return elapsed time string since search started."""
        start = self.shared.get("search_start_time")
        if start is None:
            return "0:00"
        elapsed = int(time.time() - start)
        minutes = elapsed // 60
        seconds = elapsed % 60
        return f"{minutes}:{seconds:02d}"

    def _check_abort_command(self):
        """
        Check SharedStatus for a stop_search command from the GUI.
        Also checks trigger file for ABORT_SEARCH.
        """
        # GUI command
        if self.drone.status:
            cmd = self.drone.status.consume_command()
            if cmd and cmd.get("type") == "stop_search":
                return True

        # Trigger file
        import os
        trigger_path = "/tmp/sar_trigger.txt"
        try:
            if os.path.exists(trigger_path):
                with open(trigger_path, "r") as f:
                    content = f.read().strip().upper()
                os.remove(trigger_path)
                if content == "ABORT_SEARCH":
                    return True
        except Exception:
            pass

        return False
