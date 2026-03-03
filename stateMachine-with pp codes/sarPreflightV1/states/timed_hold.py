"""
states/timed_hold.py — TIMED_HOLD state.
=========================================
Switches to a target mode, holds for N seconds while printing
telemetry, then switches back to the original mode.

Uses three phases tracked via shared["timed_hold_phase"]:

    Phase 1: Record original mode → request target → CHANGE_MODE
    Phase 2: Hold for N seconds (print telemetry) → request original → CHANGE_MODE
    Phase 3: Cleanup → WAIT

Trigger command (from file or GUI):
    "TIMED_HOLD LOITER 10"    (mode, seconds)
    "TIMED_HOLD GUIDED"       (mode, default 10s)

Full state chain:
    WAIT → TIMED_HOLD(1) → CHANGE_MODE → TIMED_HOLD(2) → CHANGE_MODE → TIMED_HOLD(3) → WAIT
"""

import time
from states.base import BaseState


DEFAULT_HOLD_DURATION = 10  # seconds


class TimedHoldState(BaseState):

    name = "TIMED_HOLD"

    def enter(self):
        phase = self.shared.get("timed_hold_phase", 1)
        self.log(f"Phase {phase} of 3")

    def execute(self):
        phase = self.shared.get("timed_hold_phase", 1)

        if phase == 1:
            return self._phase_1_request_target()
        elif phase == 2:
            return self._phase_2_hold_and_read()
        elif phase == 3:
            return self._phase_3_cleanup()
        else:
            self.log(f"ERROR: Unknown phase {phase}")
            return "WAIT"

    # ── Phase 1: Record original mode, request switch to target ──

    def _phase_1_request_target(self):
        target = self.shared.get("timed_hold_target")
        duration = self.shared.get("timed_hold_duration", DEFAULT_HOLD_DURATION)

        if target is None:
            self.log("ERROR: No target mode set")
            return "WAIT"

        # Record the current mode so we can switch back later.
        # Use extra retries since this value is critical.
        original_mode, _ = self.drone.get_mode(retries=5)

        if original_mode == "UNKNOWN":
            self.log("ERROR: Cannot determine current mode — aborting")
            self.shared["timed_hold_phase"] = 3
            return "TIMED_HOLD"    # goes to phase 3 cleanup

        self.shared["timed_hold_original"] = original_mode

        self.log(f"Original mode: {original_mode}")
        self.log(f"Will switch to {target} for {duration}s")

        # Ask CHANGE_MODE to switch, then come back here
        self.shared["requested_mode"] = target
        self.shared["return_to"] = "TIMED_HOLD"
        self.shared["timed_hold_phase"] = 2

        return "CHANGE_MODE"

    # ── Phase 2: Hold for N seconds, print telemetry, then switch back ──

    def _phase_2_hold_and_read(self):
        duration = self.shared.get("timed_hold_duration", DEFAULT_HOLD_DURATION)
        target = self.shared.get("timed_hold_target", "???")

        self.log(f"Holding {target} for {duration}s — reading telemetry ...")
        self.log(f"{'─' * 50}")

        start = time.time()
        tick = 0

        while time.time() - start < duration:
            tick += 1
            elapsed = time.time() - start
            remaining = duration - elapsed

            # Read mode FIRST (consumes heartbeat cleanly)
            mode_name, _ = self.drone.get_mode()

            # Then read telemetry from cache (no buffer interference)
            telem = self.drone.get_telemetry()

            # Format telemetry line
            alt_str = f"{telem['alt']:.1f}m" if telem['alt'] is not None else "N/A"
            sat_str = f"{telem['satellites']}" if telem['satellites'] is not None else "N/A"
            fix_str = f"{telem['gps_fix']}" if telem['gps_fix'] is not None else "N/A"
            bat_str = f"{telem['battery_v']:.1f}V" if telem['battery_v'] is not None else "N/A"
            pct_str = f"{telem['battery_pct']}%" if telem['battery_pct'] is not None else "N/A"

            lat_str = f"{telem['lat']:.7f}" if telem['lat'] is not None else "N/A"
            lon_str = f"{telem['lon']:.7f}" if telem['lon'] is not None else "N/A"

            self.log(f"[{tick}] mode={mode_name}  alt={alt_str}  "
                     f"sats={sat_str}  fix={fix_str}  "
                     f"bat={bat_str}({pct_str})  "
                     f"pos={lat_str},{lon_str}  "
                     f"remaining={remaining:.0f}s")

            time.sleep(2)

        self.log(f"{'─' * 50}")

        original = self.shared.get("timed_hold_original")
        if original is None:
            self.log("ERROR: Lost original mode — falling back to current mode")
            current, _ = self.drone.get_mode()
            self.log(f"Currently in {current} — skipping switch-back")
            self.shared["timed_hold_phase"] = 3
            return "TIMED_HOLD"

        self.log(f"Hold complete. Switching back to {original}")

        # Ask CHANGE_MODE to switch back to original
        self.shared["requested_mode"] = original
        self.shared["return_to"] = "TIMED_HOLD"
        self.shared["timed_hold_phase"] = 3

        return "CHANGE_MODE"

    # ── Phase 3: Cleanup and return to WAIT ──

    def _phase_3_cleanup(self):
        self.log("Sequence complete — cleaning up")

        # Remove all timed_hold keys from shared
        for key in list(self.shared.keys()):
            if key.startswith("timed_hold_"):
                del self.shared[key]

        self.log("Returning to WAIT")
        return "WAIT"
