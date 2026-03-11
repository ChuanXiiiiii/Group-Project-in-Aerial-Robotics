"""
states/pre_auto_check.py — PRE_AUTO_CHECK state.
==================================================
Monitors 10 pre-autonomous-flight conditions and gates the START
SEARCH button until all are green.

This is the renamed and extracted version of the preflight checklist
that previously lived inside the WAIT state.  Moving it to a
dedicated state makes the transition sequence explicit and keeps IDLE
uncluttered.

Conditions (8 automated, 2 manual):
    1.  Heartbeat            — consecutive miss count < HEARTBEAT_MISS_LIMIT
    2.  GPS fix quality      — fix ≥ MIN_GPS_FIX_TYPE
    3.  GPS satellites       — sats ≥ MIN_GPS_SATELLITES
    4.  Battery              — battery_pct ≥ MIN_BATTERY_PERCENT
    5.  Mode                 — current mode is LOITER
    6.  Armed                — drone is armed
    7.  Altitude in range    — rel alt within ±ALTITUDE_TOLERANCE_M of SEARCH_ALT_M
    8.  Mission uploaded     — autopilot reports > 1 mission items
    9.  RC / waypoints check — operator checkbox
    10. Safety pilot ready   — operator checkbox

Transitions:
    → CHANGE_MODE  (AUTO, return_to=SEARCH) when operator presses START SEARCH
    → IDLE                                  when operator presses Cancel
"""

import time
from states.base import BaseState
from config import (
    HEARTBEAT_INTERVAL,
    MIN_GPS_SATELLITES, MIN_GPS_FIX_TYPE,
    MIN_BATTERY_PERCENT, ALTITUDE_TOLERANCE_M,
    HEARTBEAT_MISS_LIMIT, SEARCH_ALT_M,
)

# Re-query autopilot mission count this often (seconds)
_MISSION_QUERY_INTERVAL = 20


class PreAutoCheckState(BaseState):

    name = "PRE_AUTO_CHECK"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enter(self):
        self.log("Entered PRE_AUTO_CHECK — monitoring 10 conditions")
        self._hb_miss = 0
        self._last_query_time = 0
        # Seed manual flags from shared (persisted by IDLE)
        if "pre_auto_wp_verified" not in self.shared:
            self.shared["pre_auto_wp_verified"] = False
        if "pre_auto_pilot_ready" not in self.shared:
            self.shared["pre_auto_pilot_ready"] = False

    def execute(self):
        while True:
            # ── Heartbeat pump ─────────────────────────────────────────
            mode_name, _ = self.drone.get_mode()
            hb_ok = (mode_name != "UNKNOWN")
            self._hb_miss = 0 if hb_ok else self._hb_miss + 1

            # ── Evaluate all conditions ────────────────────────────────
            checks = self._evaluate_checks(mode_name, hb_ok)

            # ── Push to GUI ────────────────────────────────────────────
            if self.drone.status:
                self.drone.status.set_extra("pre_auto_checks", checks)

            # ── Check GUI command ──────────────────────────────────────
            if self.drone.status:
                cmd = self.drone.status.consume_command()
                if cmd is not None:
                    next_state = self._handle_command(cmd, checks)
                    if next_state is not None:
                        return next_state

            time.sleep(HEARTBEAT_INTERVAL)

    def exit(self):
        # Clear checklist data from GUI when leaving
        if self.drone.status:
            self.drone.status.set_extra("pre_auto_checks", None)

    # ------------------------------------------------------------------
    # Command handler
    # ------------------------------------------------------------------

    def _handle_command(self, cmd, checks):
        cmd_type = cmd.get("type")

        if cmd_type == "start_search":
            if checks.get("all_ready"):
                self.log("START SEARCH approved — all conditions green")
                # SEARCH will handle the CHANGE_MODE transition internally
                # via its phase 1, so we go straight to SEARCH
                self.shared["search_phase"] = 1
                return "SEARCH"
            else:
                self.log("START SEARCH rejected — not all conditions green")
                return None

        elif cmd_type == "cancel_checklist":
            self.log("Checklist cancelled — returning to IDLE")
            return "IDLE"

        elif cmd_type == "pre_auto_toggle":
            field = cmd.get("field")
            if field == "wp_verified":
                self.shared["pre_auto_wp_verified"] = not self.shared.get(
                    "pre_auto_wp_verified", False)
                self.log(f"Waypoints/RC verified = {self.shared['pre_auto_wp_verified']}")
            elif field == "pilot_ready":
                self.shared["pre_auto_pilot_ready"] = not self.shared.get(
                    "pre_auto_pilot_ready", False)
                self.log(f"Safety pilot ready = {self.shared['pre_auto_pilot_ready']}")
            return None

        self.log(f"Unknown command '{cmd_type}' in PRE_AUTO_CHECK")
        return None

    # ------------------------------------------------------------------
    # Condition evaluation
    # ------------------------------------------------------------------

    def _evaluate_checks(self, mode_name, hb_ok):
        telem = self.drone.get_telemetry()

        # ── Periodically re-query autopilot mission/fence counts ──
        now = time.time()
        if now - self._last_query_time > _MISSION_QUERY_INTERVAL:
            try:
                mc = self.drone.query_mission_count(timeout=3)
                self.shared["_cached_mission_count"] = mc
            except Exception:
                pass
            try:
                fc = self.drone.query_fence_count(timeout=3)
                self.shared["_cached_fence_count"] = fc
            except Exception:
                pass
            self._last_query_time = now

        mission_count = self.shared.get("_cached_mission_count", 0)
        fence_count   = self.shared.get("_cached_fence_count",   0)

        # ── 1. Heartbeat ──
        hb_pass = self._hb_miss < HEARTBEAT_MISS_LIMIT

        # ── 2 & 3. GPS ──
        gps_fix  = telem.get("gps_fix")
        sats     = telem.get("satellites")
        gps_pass = (gps_fix is not None and gps_fix >= MIN_GPS_FIX_TYPE
                    and sats is not None and sats >= MIN_GPS_SATELLITES)

        # ── 4. Battery ──
        bat_pct  = telem.get("battery_pct")
        bat_pass = (bat_pct is not None and bat_pct >= 0
                    and bat_pct >= MIN_BATTERY_PERCENT)

        # ── 5. Mode = LOITER or GUIDED ──
        # LOITER is standard for a safety-pilot-flown approach.
        # GUIDED is accepted for SITL testing and GUIDED_TAKEOFF arrivals.
        mode_pass = (mode_name in ("LOITER", "GUIDED"))

        # ── 6. Armed ──
        armed     = self.drone.get_armed()
        arm_pass  = (armed is True)

        # ── 7. Altitude in range ──
        alt       = telem.get("alt")
        alt_pass  = (alt is not None
                     and abs(alt - SEARCH_ALT_M) <= ALTITUDE_TOLERANCE_M)

        # ── 8. Mission on autopilot (>1 items: home + at least one nav WP) ──
        mission_pass = (mission_count > 1)

        # ── 9 & 10. Manual checkboxes ──
        wp_verified  = bool(self.shared.get("pre_auto_wp_verified", False))
        pilot_ready  = bool(self.shared.get("pre_auto_pilot_ready", False))

        all_ready = all([
            hb_pass, gps_pass, bat_pass, mode_pass, arm_pass,
            alt_pass, mission_pass, wp_verified, pilot_ready,
        ])

        return {
            "heartbeat":   {"pass": hb_pass,
                            "detail": f"miss {self._hb_miss}/{HEARTBEAT_MISS_LIMIT}"},
            "gps":         {"pass": gps_pass,
                            "detail": f"fix={gps_fix} sats={sats}"},
            "battery":     {"pass": bat_pass,
                            "detail": f"{bat_pct}%" if bat_pct is not None and bat_pct >= 0
                                      else "no data"},
            "mode":        {"pass": mode_pass,
                            "detail": f"{mode_name} (LOITER or GUIDED)"},
            "armed":       {"pass": arm_pass,
                            "detail": "armed" if armed else "disarmed"},
            "altitude":    {"pass": alt_pass,
                            "detail": f"{alt:.1f}m" if alt is not None else "no data"},
            "mission":     {"pass": mission_pass,
                            "detail": f"{mission_count} items"},
            "fences":      {"pass": fence_count >= 2,
                            "detail": f"{fence_count} items"},
            "wp_verified": {"pass": wp_verified,  "detail": "operator checkbox"},
            "pilot_ready": {"pass": pilot_ready,  "detail": "operator checkbox"},
            "all_ready":   all_ready,
        }
