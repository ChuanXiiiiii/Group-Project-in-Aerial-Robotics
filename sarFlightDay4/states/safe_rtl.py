"""
states/safe_rtl.py — SAFE_RTL state.
=====================================
General-purpose safe return-to-home that flies the SSSI-safe corridor.

Instead of using ArduPilot RTL (which flies a straight line and may
cross the SSSI exclusion zone), this state:
    1. Uploads the SSSI_NAV_TO_HOME waypoints as a new mission
    2. Enters GUIDED (gate), then switches to AUTO
    3. Monitors progress until the drone reaches the final waypoint
    4. Requests LAND at home, waits for disarm
    5. Returns to IDLE

Entry:
    Any state can transition here.  The drone should be airborne
    in GUIDED mode (or the pilot can set GUIDED when ready).

Mode discipline:
    GUIDED (gate) → AUTO (corridor flight) → LAND (at home).
    Never requests LOITER or RTL.

Transitions:
    → IDLE  on completion (disarmed at home), cancel, or error
"""

import time
from states.base import BaseState
from config import SEARCH_ALT_M, RTL_SPEED_MPS

# Monitor tick speed
_TICK_S = 0.5

# Mode change timeout (seconds)
_MODE_TIMEOUT_S = 10

# Maximum time for the entire corridor flight (seconds)
_CORRIDOR_TIMEOUT_S = 600

# Land timeout (seconds)
_LAND_TIMEOUT_S = 60


class SafeRtlState(BaseState):

    name = "SAFE_RTL"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enter(self):
        if "safe_rtl_phase" not in self.shared:
            self.shared["safe_rtl_phase"] = 1
        phase = self.shared["safe_rtl_phase"]
        self.log(f"Entered SAFE_RTL phase {phase}")

    def execute(self):
        phase = self.shared.get("safe_rtl_phase", 1)
        if   phase == 1: return self._phase_upload_corridor()
        elif phase == 2: return self._phase_fly_corridor()
        elif phase == 3: return self._phase_land_at_home()
        else:
            self.log(f"ERROR: unknown safe_rtl_phase {phase}")
            self._cleanup()
            return "IDLE"

    def exit(self):
        if self.drone.status:
            self.drone.status.set_extra("safe_rtl_status", None)

    # ------------------------------------------------------------------
    # Phase 1 — Upload SSSI corridor mission
    # ------------------------------------------------------------------

    def _phase_upload_corridor(self):
        from config import SSSI_NAV_TO_HOME
        nav_home = getattr(self, '_nav_home', None) or SSSI_NAV_TO_HOME

        if not nav_home:
            self.log("ERROR: no SSSI_NAV_TO_HOME waypoints configured")
            self.log("Cannot return safely — operator must fly manually")
            self._push_status("NO CORRIDOR WAYPOINTS")
            self._cleanup()
            return "IDLE"

        alt = float(SEARCH_ALT_M)
        waypoints = [
            {"lat": float(lat), "lon": float(lon), "alt": alt}
            for lat, lon in nav_home
        ]

        self.log(f"Uploading SSSI-safe return corridor: {len(waypoints)} waypoints")
        self._push_status("UPLOADING CORRIDOR")

        success = self.drone.upload_mission(waypoints)
        if not success:
            self.log("ERROR: corridor mission upload FAILED")
            self._push_status("UPLOAD FAILED")
            self._cleanup()
            return "IDLE"

        self.log("Corridor mission uploaded — requesting AUTO")
        self.shared["safe_rtl_total_wps"] = len(waypoints)
        self.shared["safe_rtl_phase"] = 2

        # Need to get into AUTO — verify GUIDED first
        self.drone.drain_buffer()
        mode_name, _ = self.drone.get_mode()
        if mode_name != "GUIDED":
            self.log(f"Mode is {mode_name}, need GUIDED before AUTO")
            self._push_status("WAITING FOR GUIDED")
            if not self._wait_for_guided():
                self._cleanup()
                return "IDLE"

        # Request AUTO via set_mode() — this correctly updates _autopilot_hb
        if self.drone.set_mode("AUTO"):
            self.log("AUTO confirmed — flying corridor")
            return "SAFE_RTL"

        self.log("WARNING: AUTO mode timeout — retrying via self-transition")
        return "SAFE_RTL"

    # ------------------------------------------------------------------
    # Phase 2 — Fly the corridor in AUTO
    # ------------------------------------------------------------------

    def _phase_fly_corridor(self):
        self.log("Monitoring SSSI-safe corridor flight ...")
        self._push_status("RETURNING HOME")

        # ── Flush stale MISSION_ITEM_REACHED from pymavlink cache ──
        # pymavlink stores the last message of each type in conn.messages.
        # If FOCUS/SEARCH reached waypoint 12 (last WP of that mission),
        # the cached MISSION_ITEM_REACHED.seq=12 is still present.
        # Our corridor has only ~4 waypoints, so 12 >= 4 would instantly
        # satisfy the completion gate.  Delete the cache entry so we
        # only react to REACHES from the NEW corridor mission.
        self.drone.conn.messages.pop("MISSION_ITEM_REACHED", None)
        self.log("Cleared stale MISSION_ITEM_REACHED cache")

        # ── Set RTL speed now that AUTO is confirmed ──────────────
        # DO_CHANGE_SPEED only works in AUTO mode.  deliver.py sent a
        # WPNAV_SPEED fire-and-forget before handing off (useful for
        # GUIDED), but DO_CHANGE_SPEED must be sent here after AUTO is
        # active.  Both commands together ensure the correct speed
        # applies in AUTO (and if we ever fall back to GUIDED).
        try:
            self.drone.conn.mav.command_long_send(
                self.drone.conn.target_system,
                self.drone.conn.target_component,
                178,               # MAV_CMD_DO_CHANGE_SPEED
                0,
                1,                 # ground speed
                float(RTL_SPEED_MPS),
                -1,                # throttle: no change
                0, 0, 0, 0,
            )
        except Exception as e:
            self.log(f"WARNING: DO_CHANGE_SPEED failed: {e}")
        try:
            self.drone.set_parameter_nowait("WPNAV_SPEED", RTL_SPEED_MPS * 100)
            self.log(f"RTL speed set: {RTL_SPEED_MPS} m/s")
        except Exception as e:
            self.log(f"WARNING: WPNAV_SPEED failed: {e}")

        total_wps = self.shared.get("safe_rtl_total_wps", 1)
        deadline = time.time() + _CORRIDOR_TIMEOUT_S
        last_push = 0.0
        start_time = time.time()
        last_reached_wp = -1   # tracks MISSION_ITEM_REACHED, same pattern as SEARCH/FOCUS

        while time.time() < deadline:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()
            telem = self.drone.get_telemetry()
            mode_name, _ = self.drone.get_mode()

            # MISSION_CURRENT.seq = waypoint the drone is currently heading TO.
            # MISSION_ITEM_REACHED.seq = waypoint that has just been ARRIVED AT.
            # We use MISSION_CURRENT for display and MISSION_ITEM_REACHED for
            # completion — firing on CURRENT fires one waypoint early because the
            # check triggers as soon as the drone departs the penultimate waypoint.
            mc = self.drone.conn.messages.get("MISSION_CURRENT")
            current_wp = mc.seq if mc else 0

            mir = self.drone.conn.messages.get("MISSION_ITEM_REACHED")
            if mir is not None and mir.seq > last_reached_wp:
                last_reached_wp = mir.seq
                self.log(f"Corridor waypoint {last_reached_wp} REACHED")

            alt = telem.get("alt") or 0.0
            lat = telem.get("lat") or 0.0
            lon = telem.get("lon") or 0.0
            battery_pct = telem.get("battery_pct")
            if battery_pct is None:
                battery_pct = -1

            elapsed_s = int(time.time() - start_time)
            elapsed_str = f"{elapsed_s // 60}:{elapsed_s % 60:02d}"

            now = time.time()
            if now - last_push >= 1.0:
                # Drive progress from MISSION_CURRENT (real-time) rather than
                # MISSION_ITEM_REACHED (stale cache).  If the drone races through
                # several waypoints within a single 0.5 s tick, MISSION_CURRENT
                # already reflects where it is; using it avoids the appearance of
                # jumping from 0 % to 75 % on the first tick.
                # "reached" = number of waypoints already departed, i.e. current-1.
                # MISSION_ITEM_REACHED is still used for the completion gate below.
                reached_display = max(0, current_wp - 1) if current_wp > 0 else 0
                pct = int(reached_display / total_wps * 100) if total_wps > 0 else 0
                self._push_status("RETURNING HOME", extra={
                    "current_wp": current_wp,
                    "reached_wp": reached_display,
                    "total_wps": total_wps,
                    "progress_pct": pct,
                    "alt": round(alt, 1),
                    "lat": round(lat, 6),
                    "lon": round(lon, 6),
                    "battery_pct": battery_pct,
                    "elapsed": elapsed_str,
                    "mode": mode_name,
                })
                last_push = now

            # RC override — pilot took control
            if mode_name not in ("AUTO", "GUIDED", "UNKNOWN"):
                self.log(f"Pilot override (mode={mode_name}) — waiting for GUIDED")
                self._push_status("PAUSED (PILOT)")
                if not self._wait_for_guided():
                    self._cleanup()
                    return "IDLE"
                # Re-request AUTO via set_mode() — updates _autopilot_hb
                self._push_status("RESUMING CORRIDOR")
                if self.drone.set_mode("AUTO"):
                    self.log("AUTO confirmed — corridor flight resuming")
                else:
                    self.log("WARNING: AUTO not confirmed after override handback — continuing anyway")
                self._push_status("RETURNING HOME")

            # Mission complete: all corridor waypoints have been ARRIVED AT.
            # Use MISSION_ITEM_REACHED (not MISSION_CURRENT) so we only proceed
            # to LAND once the drone is physically at the final waypoint, not
            # the moment it departs the penultimate one.
            if last_reached_wp >= total_wps and total_wps > 0:
                self.log(f"All corridor waypoints reached ({last_reached_wp}/{total_wps}) — proceeding to LAND")
                self.shared["safe_rtl_phase"] = 3
                return "SAFE_RTL"

            # Check cancel
            cmd = self._consume_cmd()
            if cmd and cmd.get("type") == "cancel_deliver":
                self.log("SAFE_RTL cancelled by operator")
                self._cleanup()
                return "IDLE"

        self.log("WARNING: corridor flight timeout")
        self._push_status("CORRIDOR TIMEOUT")
        # Proceed to land anyway — we're hopefully near home
        self.shared["safe_rtl_phase"] = 3
        return "SAFE_RTL"

    # ------------------------------------------------------------------
    # Phase 3 — Land at home
    # ------------------------------------------------------------------

    def _phase_land_at_home(self):
        self.log("Requesting LAND mode at home ...")
        self._push_status("LANDING AT HOME")

        # Request LAND via set_mode() — updates _autopilot_hb so the
        # first loop tick sees LAND immediately (no false override).
        if self.drone.set_mode("LAND"):
            self.log("LAND mode confirmed")
        else:
            self.log("WARNING: LAND mode not confirmed within timeout — proceeding anyway")

        deadline = time.time() + _LAND_TIMEOUT_S
        while time.time() < deadline:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()
            mode_name, _ = self.drone.get_mode()
            armed = self.drone.get_armed()
            telem = self.drone.get_telemetry()
            alt = telem.get("alt") or 0.0

            # ── Pilot LOITER override during landing ─────────────────────
            # If the pilot takes over (LOITER, GUIDED, etc.) we pause and
            # wait for GUIDED handback, then re-request LAND to resume.
            if mode_name not in ("LAND", "UNKNOWN"):
                self.log(f"Pilot override during LAND (mode={mode_name}) — pausing")
                self._push_status("PAUSED (PILOT)")
                if not self._wait_for_guided():
                    self._cleanup()
                    return "IDLE"
                if not self.drone.set_mode("LAND"):
                    self.log("WARNING: LAND not confirmed after handback — continuing anyway")
                self.log("LAND re-requested after pilot handback — resuming descent")
                self._push_status("LANDING AT HOME")
                # Reset deadline so the full timeout applies from this point
                deadline = time.time() + _LAND_TIMEOUT_S

            self._push_status(f"LANDING {alt:.1f}m")

            if armed is False:
                self.log("Disarmed at home — MISSION COMPLETE")
                self._push_status("MISSION COMPLETE")
                # Mark mission as complete so operator dashboard shows
                # terminal state, and clear PLB flags so the banner
                # doesn't persist after a successful mission.
                self.shared["mission_complete"] = True
                self.shared.pop("plb_triggered", None)
                if self.drone.status:
                    self.drone.status.set_extra("plb_button_pressed", False)
                    self.drone.status.set_extra("mission_complete", True)
                self._cleanup()
                return "IDLE"

        self.log("WARNING: land timeout at home")
        self._push_status("LAND TIMEOUT")
        self._cleanup()
        return "IDLE"

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _cleanup(self):
        for k in ("safe_rtl_phase", "safe_rtl_total_wps"):
            self.shared.pop(k, None)

    def _consume_cmd(self):
        if self.drone.status:
            return self.drone.status.consume_command()
        return None

    def _push_status(self, status, extra=None):
        if self.drone.status:
            data = {"status": status}
            if extra:
                data.update(extra)
            # Always include current telemetry so spectator can track
            # the drone through all SAFE_RTL phases (same pattern as
            # deliver.py's _push_status).
            telem = self.drone.get_telemetry()
            data.setdefault("lat", round(telem.get("lat") or 0.0, 6))
            data.setdefault("lon", round(telem.get("lon") or 0.0, 6))
            data.setdefault("alt", round(telem.get("alt") or 0.0, 1))
            data.setdefault("battery_pct",
                            telem.get("battery_pct") if telem.get("battery_pct") is not None
                            and telem.get("battery_pct") >= 0 else -1)
            self.drone.status.set_extra("safe_rtl_status", data)

    def _wait_for_guided(self, timeout=120):
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()
            mode_name, _ = self.drone.get_mode()
            if mode_name == "GUIDED":
                self.log("GUIDED confirmed")
                return True
        self.log("Timeout waiting for GUIDED")
        return False
