"""
states/guided_takeoff.py — GUIDED_TAKEOFF state.
==================================================
Optional state: arms the drone, commands a GUIDED mode takeoff to
SEARCH_ALT_M, confirms the target altitude is reached, then returns
to IDLE.  The drone remains armed in GUIDED mode, ready for the
operator to open the Pre-AUTO checklist.

This state is intended for SITL testing and any scenario where the
safety pilot is not manually flying the drone to hover height before
handing over to the state machine.  In a live mission the drone would
normally already be airborne in LOITER when IDLE is entered.

Pre-conditions (checked on entry, returns to IDLE immediately if failed):
    • Not already armed
    • GUIDED mode accepted by autopilot
    • GPS fix available (fix_type >= MIN_GPS_FIX_TYPE)

Sequence:
    1. Request GUIDED mode
    2. Arm throttle via MAV_CMD_COMPONENT_ARM_DISARM
    3. Send MAV_CMD_NAV_TAKEOFF to SEARCH_ALT_M
    4. Poll altitude until within ALTITUDE_TOLERANCE_M of target
       OR timeout (TAKEOFF_TIMEOUT_S)
    5. Set throttle RC override to 1500 (mid-stick) to hold altitude
    6. Return to IDLE

Transitions:
    → IDLE  always (success or failure — operator decides next step)

shared keys written:
    "guided_takeoff_success"  True/False
    "guided_takeoff_alt"      altitude reached (metres)
"""

import time
import mission_log
from pymavlink import mavutil
from states.base import BaseState
from config import (
    SEARCH_ALT_M,
    ALTITUDE_TOLERANCE_M,
    MIN_GPS_FIX_TYPE,
)

# Seconds to wait for altitude to be reached before giving up
TAKEOFF_TIMEOUT_S = 60

# Seconds to wait for arming confirmation
ARM_TIMEOUT_S = 10

# Seconds to wait for mode change confirmation
MODE_TIMEOUT_S = 10


class GuidedTakeoffState(BaseState):

    name = "GUIDED_TAKEOFF"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enter(self):
        self.log(f"GUIDED_TAKEOFF — target altitude {SEARCH_ALT_M}m AGL")

    def execute(self):
        self.drone.drain_buffer()

        # ── Pre-condition: not already armed ──
        armed = self.drone.get_armed()
        if armed:
            self.log("WARNING: drone is already armed — skipping takeoff sequence")
            self.shared["guided_takeoff_success"] = False
            return "IDLE"

        # ── Pre-condition: GPS fix ──
        telem = self.drone.get_telemetry()
        gps_fix = telem.get("gps_fix")
        if gps_fix is None or gps_fix < MIN_GPS_FIX_TYPE:
            self.log(f"ERROR: GPS fix insufficient (fix_type={gps_fix}) — aborting takeoff")
            self.shared["guided_takeoff_success"] = False
            return "IDLE"

        # ── Step 1: Switch to GUIDED ──
        self.log("Step 1: Requesting GUIDED mode ...")
        if not self._set_mode_with_timeout("GUIDED", MODE_TIMEOUT_S):
            self.log("ERROR: Failed to enter GUIDED mode — aborting")
            self.shared["guided_takeoff_success"] = False
            return "IDLE"
        self.log("GUIDED mode confirmed")

        # ── Step 2: Arm ──
        self.log("Step 2: Arming ...")
        if not self._arm(ARM_TIMEOUT_S):
            self.log("ERROR: Arming failed — aborting")
            self.shared["guided_takeoff_success"] = False
            return "IDLE"
        self.log("Armed")

        # ── Step 3: Takeoff command ──
        self.log(f"Step 3: Commanding takeoff to {SEARCH_ALT_M}m ...")
        self.drone.conn.mav.command_long_send(
            self.drone.conn.target_system,
            self.drone.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,          # confirmation
            0,          # param1: min pitch (ignored for copter)
            0,          # param2: empty
            0,          # param3: empty
            0,          # param4: yaw angle (NaN = current)
            0,          # param5: lat (0 = current)
            0,          # param6: lon (0 = current)
            float(SEARCH_ALT_M),  # param7: altitude (metres AGL)
        )

        # ── Step 4: Wait for altitude ──
        self.log(f"Step 4: Climbing — waiting for {SEARCH_ALT_M}m "
                 f"(±{ALTITUDE_TOLERANCE_M}m) ...")
        reached = self._wait_for_altitude(SEARCH_ALT_M, ALTITUDE_TOLERANCE_M,
                                          TAKEOFF_TIMEOUT_S)
        if not reached:
            telem = self.drone.get_telemetry()
            alt_now = telem.get("alt") or 0.0
            self.log(f"WARNING: Takeoff timeout — reached {alt_now:.1f}m of {SEARCH_ALT_M}m")
            # Don't abort — partial altitude is still useful; let operator decide
            self.shared["guided_takeoff_success"] = False
            self.shared["guided_takeoff_alt"] = round(alt_now, 1)
            self._push_status("TIMEOUT")
            return "IDLE"

        # ── Step 5: Confirm hover ──
        telem = self.drone.get_telemetry()
        alt_now = telem.get("alt") or 0.0
        self.log(f"Target altitude reached: {alt_now:.1f}m — hovering in GUIDED")
        mission_log.takeoff(alt_now)
        self.shared["guided_takeoff_success"] = True
        self.shared["guided_takeoff_alt"] = round(alt_now, 1)
        self._push_status("COMPLETE")

        return "IDLE"

    def exit(self):
        if self.drone.status:
            self.drone.status.set_extra("guided_takeoff_status", None)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _set_mode_with_timeout(self, mode_name, timeout_s):
        """Send mode change and wait for confirmation."""
        from config import FLIGHT_MODES, MODE_NAMES
        target_id = FLIGHT_MODES.get(mode_name)
        if target_id is None:
            self.log(f"ERROR: unknown mode '{mode_name}'")
            return False

        self.drone.conn.mav.set_mode_send(
            self.drone.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            target_id,
        )
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.drone.drain_buffer()
            hb = self.drone.conn.messages.get("HEARTBEAT")
            if hb is not None and hb.custom_mode == target_id:
                return True
            time.sleep(0.2)
        return False

    def _arm(self, timeout_s):
        """Send arm command and wait for armed confirmation."""
        self.drone.conn.mav.command_long_send(
            self.drone.conn.target_system,
            self.drone.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,      # confirmation
            1,      # param1: 1 = arm
            0, 0, 0, 0, 0, 0,
        )
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.drone.drain_buffer()
            armed = self.drone.get_armed()
            if armed is True:
                return True
            time.sleep(0.2)
        return False

    def _wait_for_altitude(self, target_m, tolerance_m, timeout_s):
        """
        Poll altitude until within ±tolerance_m of target, or timeout.
        Polls at 4 Hz (0.25s) so monitoring stays tight during the climb.
        Logs progress every ~5 seconds.
        """
        deadline  = time.time() + timeout_s
        last_log  = 0.0
        POLL_RATE = 0.25   # seconds between reads

        while time.time() < deadline:
            self.drone.drain_buffer()
            telem = self.drone.get_telemetry()
            alt   = telem.get("alt")

            if alt is not None:
                if abs(alt - target_m) <= tolerance_m:
                    return True
                now = time.time()
                if now - last_log >= 5.0:
                    self.log(f"  Climbing: {alt:.1f}m → {target_m}m")
                    self._push_status(f"CLIMBING {alt:.1f}m")
                    last_log = now

            time.sleep(POLL_RATE)
        return False

    def _push_status(self, status_str):
        if self.drone.status:
            telem = self.drone.get_telemetry()
            self.drone.status.set_extra("guided_takeoff_status", {
                "status": status_str,
                "target_alt": SEARCH_ALT_M,
                "current_alt": round(telem.get("alt") or 0.0, 1),
            })
