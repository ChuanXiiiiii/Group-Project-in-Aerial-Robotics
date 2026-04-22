"""
states/deliver.py — DELIVER state.
====================================
Interactive casualty confirmation, landing point selection, payload
delivery, and return-to-home via stored GUIDED coordinates.

Entry:
    FOCUS detects a casualty image → switches to GUIDED → transitions here.
    Detection image path is in shared["detection_file"].
    Geotag coordinates (detection_lat/lon) are NOT yet available at entry —
    they are acquired in phase 2 from a second stationary-hover image.
    Focus polygon is in shared["focus_polygon"] (preserved from REPLAN).

Phase structure (shared["deliver_phase"]):
    Phase 1 — Detection confirmation: show detection image on GUI,
              operator confirms "is this the dummy?" or rejects.
              Reject → delete detection file, return to FOCUS.
    Phase 2 — Geotag acquisition: drone hovers in GUIDED with a
              stationary downward camera.  CV pipeline drops a second
              image with an accurate geotag (filename: NUM_LAT_LON.png).
              Coordinates are extracted and stored for landing selection.
    Phase 3 — Landing point selection: GUI shows interactive canvas
              with focus polygon, dummy location, and 7.5m red circle.
              Operator clicks within circle to select landing point,
              then confirms.  Coordinates sent via confirm_landing cmd.
    Phase 4 — Approach + land via AUTO mission: upload a short mission
              (DO_CHANGE_SPEED + NAV_WAYPOINT + NAV_LAND) and switch to
              AUTO.  ArduPilot flies to the landing point at approach
              altitude and then performs NAV_LAND automatically.  More
              robust than GUIDED + SET_POSITION_TARGET because the
              autopilot owns the sequence (no single packet to lose).
    Phase 5 — Monitor descent: just wait for disarm — NAV_LAND from
              phase 4 handles the actual landing.
    Phase 6 — Deploy confirmation: GUI "Confirm Deploy" button.
    Phase 7 — Relaunch & hand off to SAFE_RTL: arm in GUIDED,
              takeoff, then transition to SAFE_RTL which flies the
              SSSI-safe corridor home in AUTO mode.

Mode discipline:
    Phase 1–3: GUIDED (hover while operator decides / geotag acquired)
    Phase 4:   GUIDED (gate) → AUTO (fly landing mission, NAV_LAND included)
    Phase 5:   AUTO / LAND (monitor descent until disarm)
    Phase 7:   GUIDED (takeoff) → SAFE_RTL (corridor home)
    The pilot can override at any time.

Transitions:
    → FOCUS      on detection rejection (resume focus search)
    → SAFE_RTL   on phase 7 relaunch (SSSI-safe return home)
    → IDLE       on cancel or error
"""

import os
import glob
import time
import math
from pymavlink import mavutil
from states.base import BaseState
from config import (
    SEARCH_ALT_M,
    ALTITUDE_TOLERANCE_M,
    DETECTION_DIR,
    APPROACH_SPEED_MPS as _APPROACH_SPEED_MPS_CFG,
    RTL_SPEED_MPS,
    LANDING_STANDOFF_M,
    FOCUS_ALT_M,
)
from cv_comm import write_cv_mode

# How close to target before approach is done (metres)
_APPROACH_TOLERANCE_M = 3.0

# Approach speed (m/s) — slow for safety near casualty (from config)
_APPROACH_SPEED_MPS = _APPROACH_SPEED_MPS_CFG

# How far from casualty landing is allowed (metres)  R07: 5–10m
_STANDOFF_RADIUS_M = LANDING_STANDOFF_M

# Timeout for GUIDED approach (seconds)
_APPROACH_TIMEOUT_S = 120

# Timeout for landing (seconds)
_LAND_TIMEOUT_S = 60

# Relaunch altitude (metres AGL)
_RELAUNCH_ALT_M = 15.0

# Takeoff timeout (seconds)
_TAKEOFF_TIMEOUT_S = 60

# Arm timeout (seconds)
_ARM_TIMEOUT_S = 10

# Mode change timeout (seconds)
_MODE_TIMEOUT_S = 10

# Monitor tick speed
_TICK_S = 0.5


class DeliverState(BaseState):

    name = "DELIVER"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def enter(self):
        if "deliver_phase" not in self.shared:
            self.shared["deliver_phase"] = 1
        phase = self.shared["deliver_phase"]
        self.log(f"Entered DELIVER phase {phase}")

    def execute(self):
        phase = self.shared.get("deliver_phase", 1)
        if   phase == 1: return self._phase_confirm_detection()
        elif phase == 2: return self._phase_wait_geotag()
        elif phase == 3: return self._phase_select_landing()
        elif phase == 4: return self._phase_approach()
        elif phase == 5: return self._phase_land()
        elif phase == 6: return self._phase_deploy()
        elif phase == 7: return self._phase_relaunch_home()
        else:
            self.log(f"ERROR: unknown deliver phase {phase}")
            self._cleanup()
            return "IDLE"

    def exit(self):
        # Only clear GUI status — do NOT clean deliver_phase on self-transitions
        if self.drone.status:
            self.drone.status.set_extra("deliver_status", None)

    # ------------------------------------------------------------------
    # Phase 1 — Detection confirmation
    # ------------------------------------------------------------------

    def _phase_confirm_detection(self):
        """
        Hover in GUIDED.  Push detection image path to GUI.
        Wait for operator to confirm or reject the detection.
        Geotag coordinates are NOT yet available — they come from
        the second (stationary-hover) image in phase 2.
        """
        det_file = self.shared.get("detection_file")

        if not det_file:
            self.log("ERROR: no detection file — returning to IDLE")
            self._cleanup()
            return "IDLE"

        self.log(f"Detection image: {det_file}")
        self.log("Waiting for operator to confirm detection ...")

        self._push_status("CONFIRM DETECTION", extra={
            "phase": 1,
            "detection_file": det_file,
        })

        while True:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()
            self.drone.get_telemetry()  # keep telemetry fresh

            cmd = self._consume_cmd()
            if cmd is not None:
                cmd_type = cmd.get("type")

                if cmd_type == "confirm_detection":
                    self.log("Detection CONFIRMED by operator")
                    write_cv_mode("GEOTAG")
                    self.log("CV mode → GEOTAG (hovering for stationary image)")
                    self.shared["deliver_phase"] = 2
                    return "DELIVER"

                elif cmd_type == "reject_detection":
                    self.log("Detection REJECTED by operator — returning to FOCUS")
                    write_cv_mode("DETECTING")
                    # Delete the detection file so FOCUS doesn't re-trigger
                    if det_file:
                        try:
                            filepath = os.path.join(DETECTION_DIR, det_file)
                            if os.path.exists(filepath):
                                os.remove(filepath)
                                self.log(f"Removed detection file: {det_file}")
                        except Exception as e:
                            self.log(f"WARNING: could not remove detection file: {e}")
                    # Clean all deliver + detection keys
                    self._cleanup()
                    # Return to FOCUS — need AUTO mode
                    self.shared["requested_mode"] = "AUTO"
                    self.shared["return_to"] = "FOCUS"
                    return "CHANGE_MODE"

                elif cmd_type == "cancel_deliver":
                    self.log("DELIVER cancelled during detection confirmation")
                    write_cv_mode("PASSIVE")
                    self._cleanup()
                    return "IDLE"

    # ------------------------------------------------------------------
    # Phase 2 — Wait for geotag image (stationary hover)
    # ------------------------------------------------------------------

    def _phase_wait_geotag(self):
        """
        Drone hovers in GUIDED with a stationary downward camera.
        The CV pipeline drops a second image whose filename encodes the
        accurate geotag (NUM_LAT_LON.png).  We scan DETECTION_DIR for
        a new file that is different from the first detection image.
        """
        first_file = self.shared.get("detection_file")
        self.log("Hovering — waiting for geotag image from CV ...")
        self._push_status("WAITING FOR GEOTAG", extra={"phase": 2})

        while True:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()
            self.drone.get_telemetry()

            # Scan for a NEW geotagged image (different from the first)
            geotag = self._scan_for_geotag(exclude=first_file)
            if geotag is not None:
                geo_lat, geo_lon, geo_file = geotag
                self.log(f"GEOTAG IMAGE: {geo_lat:.6f}, {geo_lon:.6f} "
                         f"(file: {geo_file})")
                write_cv_mode("PASSIVE")
                self.log("CV mode → PASSIVE (geotag acquired)")
                self.shared["detection_lat"] = geo_lat
                self.shared["detection_lon"] = geo_lon
                self.shared["geotag_file"] = geo_file
                self.shared["deliver_phase"] = 3
                return "DELIVER"

            cmd = self._consume_cmd()
            if cmd is not None:
                if cmd.get("type") == "cancel_deliver":
                    self.log("DELIVER cancelled while waiting for geotag")
                    write_cv_mode("PASSIVE")
                    self._cleanup()
                    return "IDLE"

    # ------------------------------------------------------------------
    # Phase 3 — Interactive landing point selection
    # ------------------------------------------------------------------

    def _phase_select_landing(self):
        """
        Compute 4 NESW landing options at standoff distance from the
        detected casualty.  Push these plus satellite background config
        to the GUI.  Operator selects N, E, S, or W and the confirm
        command sends the corresponding {lat, lon} back.
        """
        det_lat = self.shared.get("detection_lat")
        det_lon = self.shared.get("detection_lon")
        focus_poly = self.shared.get("focus_polygon")

        nesw = self._compute_nesw_points(det_lat, det_lon)
        self.log(f"NESW landing options computed at {_STANDOFF_RADIUS_M}m standoff:")
        for direction, pt in nesw.items():
            self.log(f"  {direction}: {pt['lat']:.6f}, {pt['lon']:.6f}")

        self.log("Waiting for operator to select landing direction ...")

        self._push_status("SELECT LANDING POINT", extra={
            "phase": 3,
            "detection_lat": det_lat,
            "detection_lon": det_lon,
            "focus_polygon": focus_poly,
            "standoff_radius_m": _STANDOFF_RADIUS_M,
            "nesw_points": nesw,
        })

        while True:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()
            self.drone.get_telemetry()

            cmd = self._consume_cmd()
            if cmd is not None:
                cmd_type = cmd.get("type")

                if cmd_type == "confirm_landing":
                    land_lat = cmd.get("lat")
                    land_lon = cmd.get("lon")
                    if land_lat is None or land_lon is None:
                        self.log("ERROR: confirm_landing missing lat/lon")
                        continue

                    # Validate distance from casualty
                    dist = self._haversine(det_lat, det_lon, land_lat, land_lon)
                    self.log(f"Landing point selected: {land_lat:.6f}, {land_lon:.6f} "
                             f"({dist:.1f}m from casualty)")

                    if dist > _STANDOFF_RADIUS_M + 2.0:
                        self.log(f"WARNING: landing point {dist:.1f}m from casualty "
                                 f"(max {_STANDOFF_RADIUS_M}m) — proceeding anyway")

                    self.shared["deliver_target_lat"] = land_lat
                    self.shared["deliver_target_lon"] = land_lon
                    self.shared["deliver_phase"] = 4
                    return "DELIVER"

                elif cmd_type == "cancel_deliver":
                    self.log("DELIVER cancelled during landing selection")
                    self._cleanup()
                    return "IDLE"

    # ------------------------------------------------------------------
    # Phase 4 — Slow approach to selected landing point
    # ------------------------------------------------------------------

    def _phase_approach(self):
        """
        Upload a short AUTO mission (DO_CHANGE_SPEED + NAV_WAYPOINT +
        NAV_LAND) and switch to AUTO.  ArduPilot owns the sequence —
        so no single packet can derail the approach as happened with
        the previous SET_POSITION_TARGET_GLOBAL_INT implementation.

        Mission structure:
            Seq 0: HOME (placeholder; ArduPilot overwrites on arm)
            Seq 1: DO_CHANGE_SPEED (APPROACH_SPEED_MPS)
            Seq 2: NAV_WAYPOINT     (target @ FOCUS_ALT_M)
            Seq 3: NAV_LAND         (target @ 0)

        Advances to phase 5 once NAV_WAYPOINT (seq=2) is REACHED.
        Phase 5 then monitors descent from NAV_LAND.
        """
        target_lat = self.shared.get("deliver_target_lat")
        target_lon = self.shared.get("deliver_target_lon")
        if target_lat is None or target_lon is None:
            self.log("ERROR: no landing point selected — returning to IDLE")
            self._cleanup()
            return "IDLE"

        # ── Verify GUIDED mode before uploading / switching to AUTO ──
        self.drone.drain_buffer()
        mode_name, _ = self.drone.get_mode()
        if mode_name != "GUIDED":
            self.log(f"Mode is {mode_name}, need GUIDED — waiting")
            self._push_status("WAITING FOR GUIDED", extra={"phase": 4})
            if not self._wait_for_guided():
                self._cleanup()
                return "IDLE"

        # ── Upload the landing mission ───────────────────────────────
        approach_alt = float(FOCUS_ALT_M)   # deterministic, not telemetry
        self.log(f"Uploading landing mission — target {target_lat:.6f},"
                 f"{target_lon:.6f} @ {approach_alt}m, "
                 f"speed {_APPROACH_SPEED_MPS} m/s")
        self._push_status("UPLOADING LANDING MISSION", extra={"phase": 4})

        success = self.drone.upload_landing_mission(
            target_lat, target_lon, approach_alt, _APPROACH_SPEED_MPS,
        )
        if not success:
            self.log("ERROR: landing mission upload FAILED")
            self._push_status("UPLOAD FAILED", extra={"phase": 4})
            self._cleanup()
            return "IDLE"

        # ── Clear stale MISSION_ITEM_REACHED cache ───────────────────
        # pymavlink keeps the last message of each type in conn.messages.
        # Previous missions (SEARCH/FOCUS) will have left a cached
        # MISSION_ITEM_REACHED with a high seq.  We only want to react
        # to REACHES from THIS mission, so wipe it.
        self.drone.conn.messages.pop("MISSION_ITEM_REACHED", None)
        self.log("Cleared stale MISSION_ITEM_REACHED cache")

        # ── Also belt-and-braces the WPNAV_SPEED for GUIDED fallback ─
        # DO_CHANGE_SPEED (seq 1) is authoritative in AUTO.  If the pilot
        # ever handbacks to GUIDED mid-approach, WPNAV_SPEED governs,
        # so pre-set it too.
        self._set_speed(_APPROACH_SPEED_MPS)

        # ── Switch to AUTO ──────────────────────────────────────────
        if not self.drone.set_mode("AUTO"):
            self.log("ERROR: failed to enter AUTO for landing mission")
            self._push_status("AUTO FAILED", extra={"phase": 4})
            self._cleanup()
            return "IDLE"

        self.log(f"AUTO confirmed — flying to {target_lat:.6f}, "
                 f"{target_lon:.6f} @ {approach_alt}m")
        self._push_status("APPROACHING", extra={"phase": 4})

        deadline = time.time() + _APPROACH_TIMEOUT_S
        last_push = 0.0
        last_reached_wp = -1

        while time.time() < deadline:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()
            mode_name, _ = self.drone.get_mode()
            telem = self.drone.get_telemetry()
            armed = self.drone.get_armed()

            mir = self.drone.conn.messages.get("MISSION_ITEM_REACHED")
            if mir is not None and mir.seq > last_reached_wp:
                last_reached_wp = mir.seq
                self.log(f"Landing mission waypoint {last_reached_wp} REACHED")

            # ── Pilot override ──────────────────────────────────────
            # AUTO is the valid mode here.  If the pilot takes over,
            # wait for GUIDED, then re-request AUTO so the mission
            # resumes from wherever ArduPilot paused it.
            if mode_name not in ("AUTO", "UNKNOWN"):
                self.log(f"Pilot override during approach (mode={mode_name})")
                self._push_status("PAUSED (PILOT)", extra={"phase": 4})
                if not self._wait_for_guided():
                    self._cleanup()
                    return "IDLE"
                self._push_status("RESUMING APPROACH", extra={"phase": 4})
                if self.drone.set_mode("AUTO"):
                    self.log("AUTO confirmed — approach resuming")
                else:
                    self.log("WARNING: AUTO not confirmed after handback — continuing")
                self._push_status("APPROACHING", extra={"phase": 4})
                deadline = time.time() + _APPROACH_TIMEOUT_S   # reset

            # ── Cancel ──────────────────────────────────────────────
            if self._check_cancel():
                self.log("Cancel during approach")
                self._cleanup()
                self.shared["requested_mode"] = "GUIDED"
                self.shared["return_to"] = "IDLE"
                return "CHANGE_MODE"

            # ── Telemetry / progress push ───────────────────────────
            cur_lat = telem.get("lat") or 0.0
            cur_lon = telem.get("lon") or 0.0
            dist = self._haversine(cur_lat, cur_lon, target_lat, target_lon)

            now = time.time()
            if now - last_push >= 1.0:
                self._push_status(f"APPROACHING {dist:.0f}m", extra={
                    "phase": 4,
                    "dist": round(dist, 1),
                    "lat": round(cur_lat, 6),
                    "lon": round(cur_lon, 6),
                    "alt": round(telem.get("alt") or 0.0, 1),
                })
                last_push = now

            # ── Early disarm (user/ArduPilot already landed somehow) ─
            if armed is False:
                self.log("Disarmed during approach — skipping to deploy")
                self.shared["deliver_phase"] = 6
                return "DELIVER"

            # ── Completion: NAV_WAYPOINT (seq 2) reached → descent ──
            # NAV_LAND (seq 3) is then active — phase 5 monitors it.
            if last_reached_wp >= 2:
                self.log(f"Arrived at landing point ({dist:.1f}m from target) "
                         f"— NAV_LAND active")
                self.shared["deliver_phase"] = 5
                return "DELIVER"

        self.log("WARNING: Approach timeout — proceeding to land monitor anyway")
        self.shared["deliver_phase"] = 5
        return "DELIVER"

    # ------------------------------------------------------------------
    # Phase 5 — Monitor NAV_LAND descent until disarm
    # ------------------------------------------------------------------

    def _phase_land(self):
        """
        NAV_LAND (seq 3 of the mission uploaded in phase 4) handles the
        descent automatically.  This phase just watches for disarm,
        with pilot-override recovery: if the pilot pauses in LOITER /
        GUIDED, wait for GUIDED and re-request AUTO to resume NAV_LAND.
        """
        self.log("Monitoring NAV_LAND descent ...")
        self._push_status("LANDING", extra={"phase": 5})

        deadline = time.time() + _LAND_TIMEOUT_S
        while time.time() < deadline:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()
            mode_name, _ = self.drone.get_mode()
            telem = self.drone.get_telemetry()
            armed = self.drone.get_armed()
            alt = telem.get("alt") or 0.0

            # ── Pilot override during descent ────────────────────────
            # Valid modes: AUTO (still running NAV_LAND) or LAND
            # (ArduPilot auto-transitions NAV_LAND into LAND near ground
            # on some firmwares).  Anything else = pilot took over.
            if mode_name not in ("AUTO", "LAND", "UNKNOWN"):
                self.log(f"Pilot override during descent (mode={mode_name}) — pausing")
                self._push_status("PAUSED (PILOT)", extra={"phase": 5})
                if not self._wait_for_guided():
                    self._cleanup()
                    return "IDLE"
                # Re-request AUTO to resume NAV_LAND where it paused.
                if self.drone.set_mode("AUTO"):
                    self.log("AUTO re-confirmed — descent resuming")
                else:
                    self.log("WARNING: AUTO not confirmed — trying LAND as fallback")
                    self.drone.set_mode("LAND")
                deadline = time.time() + _LAND_TIMEOUT_S   # reset

            self._push_status(f"LANDING {alt:.1f}m", extra={
                "phase": 5,
                "alt": round(alt, 1),
            })

            if armed is False:
                self.log("Disarmed — landed successfully")
                self._push_status("LANDED", extra={"phase": 5})
                self.shared["deliver_phase"] = 6
                return "DELIVER"

        self.log("WARNING: Land timeout — checking armed status")
        armed = self.drone.get_armed()
        if armed is False:
            self.shared["deliver_phase"] = 6
            return "DELIVER"

        self.log("ERROR: land timeout and still armed")
        self._push_status("LAND TIMEOUT", extra={"phase": 5})
        self._cleanup()
        return "IDLE"

    # ------------------------------------------------------------------
    # Phase 6 — Deploy payload (GUI confirmation)
    # ------------------------------------------------------------------

    def _phase_deploy(self):
        self.log("Waiting for operator to deploy payload ...")
        self.log("Press 'Confirm Deploy' on the dashboard when payload is released")
        self._push_status("DEPLOY — CONFIRM ON GUI", extra={"phase": 6})

        while True:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()

            cmd = self._consume_cmd()
            if cmd is not None:
                cmd_type = cmd.get("type")

                if cmd_type == "confirm_deploy":
                    self.log("PAYLOAD DEPLOYED — confirmed by operator")
                    self.shared["payload_deployed"] = True
                    # Set a persistent flag on SharedStatus that survives
                    # state transitions (deliver_status gets cleared by
                    # exit(), but this extra persists for the spectator).
                    if self.drone.status:
                        self.drone.status.set_extra("payload_deployed", True)
                    self._push_status("DEPLOYED — PRESS COMPLETE MISSION",
                                      extra={"phase": 6})
                    self.shared["deliver_phase"] = 7
                    return "DELIVER"

                elif cmd_type == "cancel_deliver":
                    self.log("DELIVER cancelled by operator")
                    self._cleanup()
                    return "IDLE"

    # ------------------------------------------------------------------
    # Phase 7 — Relaunch and hand off to SAFE_RTL
    # ------------------------------------------------------------------

    def _phase_relaunch_home(self):
        """
        Wait for operator to press 'Complete Mission', then arm, takeoff
        in GUIDED, and transition to SAFE_RTL which flies the SSSI-safe
        corridor home.
        """
        self.log("Waiting for operator to press 'Complete Mission' ...")
        self._push_status("PRESS COMPLETE MISSION", extra={"phase": 7})

        # Wait for relaunch command
        while True:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()

            cmd = self._consume_cmd()
            if cmd is not None:
                cmd_type = cmd.get("type")

                if cmd_type == "complete_mission":
                    self.log("COMPLETE MISSION received — relaunching")
                    break

                elif cmd_type == "cancel_deliver":
                    self.log("Cancelled — staying on ground")
                    self._cleanup()
                    return "IDLE"

        # ── Arm and takeoff in GUIDED ──
        self._push_status("ARMING", extra={"phase": 7})

        if not self._set_mode_with_timeout("GUIDED", _MODE_TIMEOUT_S):
            self.log("ERROR: failed to enter GUIDED for relaunch")
            self._cleanup()
            return "IDLE"

        if not self._arm(_ARM_TIMEOUT_S):
            self.log("ERROR: arming failed for relaunch")
            self._cleanup()
            return "IDLE"

        self.log(f"Armed — commanding takeoff to {_RELAUNCH_ALT_M}m")
        self._push_status(f"CLIMBING TO {_RELAUNCH_ALT_M}m", extra={"phase": 7})

        self.drone.conn.mav.command_long_send(
            self.drone.conn.target_system,
            self.drone.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0,
            float(_RELAUNCH_ALT_M),
        )

        if not self._wait_for_altitude(_RELAUNCH_ALT_M, ALTITUDE_TOLERANCE_M,
                                       _TAKEOFF_TIMEOUT_S):
            self.log("WARNING: takeoff timeout — attempting SAFE_RTL anyway")

        # ── Pre-set RTL speed before handing off ──
        # WPNAV_SPEED was set to approach speed (1 m/s) during delivery.
        # Fire-and-forget WPNAV_SPEED to RTL_SPEED so the parameter is
        # ready when SAFE_RTL enters AUTO.  SAFE_RTL will also re-send
        # DO_CHANGE_SPEED once AUTO is confirmed (the authoritative set).
        self.log(f"Pre-setting RTL speed: {RTL_SPEED_MPS} m/s")
        self._set_speed(RTL_SPEED_MPS)

        # ── Hand off to SAFE_RTL ──
        self.log("Airborne — handing off to SAFE_RTL for SSSI-safe return")
        self._cleanup()
        return "SAFE_RTL"

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _scan_for_geotag(self, exclude=None):
        """
        Scan DETECTION_DIR for a geotagged image, skipping *exclude*.
        Filename format: NUM_LAT_LON.png  (e.g. "2_51.4241_-2.6714.png").
        Returns (lat, lon, filename) or None.
        """
        try:
            for ext in ("*.png", "*.jpg", "*.jpeg"):
                files = sorted(glob.glob(os.path.join(DETECTION_DIR, ext)))
                for filepath in files:
                    filename = os.path.basename(filepath)
                    if filename == exclude:
                        continue
                    name_part = os.path.splitext(filename)[0]
                    parts = name_part.split("_")
                    if len(parts) >= 2:
                        try:
                            det_lat = float(parts[-2])
                            det_lon = float(parts[-1])
                            if -90 <= det_lat <= 90 and -180 <= det_lon <= 180:
                                if len(parts) >= 3:
                                    try:
                                        self.shared["detection_index"] = int(parts[0])
                                    except ValueError:
                                        pass
                                return (det_lat, det_lon, filename)
                        except ValueError:
                            continue
        except Exception as e:
            self.log(f"WARNING: geotag scan error: {e}")
        return None

    def _cleanup(self):
        """Remove deliver working keys from shared."""
        for k in ("deliver_phase", "deliver_target_lat", "deliver_target_lon",
                  "payload_deployed", "detection_lat", "detection_lon",
                  "detection_file", "detection_index", "geotag_file"):
            self.shared.pop(k, None)

    def _consume_cmd(self):
        if self.drone.status:
            return self.drone.status.consume_command()
        return None

    def _check_cancel(self):
        cmd = self._consume_cmd()
        if cmd and cmd.get("type") == "cancel_deliver":
            return True
        return False

    def _push_status(self, status, extra=None):
        if self.drone.status:
            data = {"status": status}
            if extra:
                data.update(extra)
            # Always include current drone position so external displays
            # can track the drone through all deliver phases.
            telem = self.drone.get_telemetry()
            data.setdefault("lat", round(telem.get("lat") or 0.0, 6))
            data.setdefault("lon", round(telem.get("lon") or 0.0, 6))
            data.setdefault("alt", round(telem.get("alt") or 0.0, 1))
            data.setdefault("battery_pct",
                            telem.get("battery_pct") if telem.get("battery_pct") is not None
                            and telem.get("battery_pct") >= 0 else -1)
            # Expose the selected landing target so external displays
            # (e.g. spectator GUI) can show it on a map.
            tgt_lat = self.shared.get("deliver_target_lat")
            tgt_lon = self.shared.get("deliver_target_lon")
            if tgt_lat is not None and tgt_lon is not None:
                data.setdefault("deliver_target_lat", tgt_lat)
                data.setdefault("deliver_target_lon", tgt_lon)
            self.drone.status.set_extra("deliver_status", data)

    def _wait_for_guided(self, timeout=120):
        """Wait for pilot to set GUIDED. Returns True if GUIDED seen."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(_TICK_S)
            self.drone.drain_buffer()
            mode_name, _ = self.drone.get_mode()
            if mode_name == "GUIDED":
                self.log("GUIDED confirmed")
                return True
            if self._check_cancel():
                return False
        self.log("Timeout waiting for GUIDED")
        return False

    def _set_mode_with_timeout(self, mode_name, timeout_s):
        """Send mode change and wait for confirmation.
        Uses drone.set_mode() which correctly updates the filtered
        autopilot heartbeat cache (_autopilot_hb), preventing stale
        mode reads on the next get_mode() call.
        """
        success = self.drone.set_mode(mode_name)
        return success

    def _arm(self, timeout_s):
        """Send arm command and wait for confirmation."""
        self.drone.conn.mav.command_long_send(
            self.drone.conn.target_system,
            self.drone.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,          # confirmation
            1,          # param1: 1 = arm
            0, 0, 0, 0, 0, 0,   # param2–7
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
        """Poll altitude until within tolerance, or timeout."""
        deadline = time.time() + timeout_s
        last_push = 0.0
        while time.time() < deadline:
            self.drone.drain_buffer()
            telem = self.drone.get_telemetry()
            alt = telem.get("alt")
            if alt is not None:
                if abs(alt - target_m) <= tolerance_m:
                    return True
                now = time.time()
                if now - last_push >= 1.0:
                    self._push_status(f"CLIMBING {alt:.1f}m", extra={"phase": 7})
                    last_push = now
            time.sleep(0.25)
        return False

    def _set_speed(self, speed_mps):
        """
        Set ground speed for both AUTO and GUIDED modes.

        MAV_CMD_DO_CHANGE_SPEED is honoured in AUTO mode (waypoint missions).
        In GUIDED mode ArduPilot ignores it and uses WPNAV_SPEED instead,
        so we set both to ensure the correct speed applies regardless of mode.
        WPNAV_SPEED is in cm/s.

        For critical speed changes (e.g. approach at 1 m/s) we send
        WPNAV_SPEED three times with short gaps — a single fire-and-forget
        PARAM_SET can be lost or overridden by telemetry traffic, which
        caused the drone to approach at 6 m/s instead of 1 m/s.
        """
        cms = speed_mps * 100

        # AUTO mode: mission speed override
        self.drone.conn.mav.command_long_send(
            self.drone.conn.target_system,
            self.drone.conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,                  # speed type: ground speed
            float(speed_mps),   # speed m/s
            -1,                 # throttle: no change
            0, 0, 0, 0,
        )
        # GUIDED mode: ArduPilot parameter (cm/s).
        # Send three times with drain_buffer() between sends to maximise
        # the chance the autopilot processes at least one PARAM_SET before
        # the next position target command arrives.  The drone is hovering
        # during these calls so the ~0.3s total delay is acceptable.
        for i in range(3):
            self.drone.set_parameter_nowait("WPNAV_SPEED", cms)
            if i < 2:
                time.sleep(0.1)
                self.drone.drain_buffer()
        self.log(f"Speed set: {speed_mps} m/s (WPNAV_SPEED={cms} cm/s, sent 3x)")

    def _send_position_target(self, lat, lon, alt):
        """Send SET_POSITION_TARGET_GLOBAL_INT to fly to a GPS point."""
        self.drone.conn.mav.set_position_target_global_int_send(
            0,                                              # time_boot_ms
            self.drone.conn.target_system,
            self.drone.conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,                             # type_mask: pos only
            int(lat * 1e7),                                 # lat (degE7)
            int(lon * 1e7),                                 # lon (degE7)
            float(alt),                                     # alt (m, relative)
            0, 0, 0,                                        # vx, vy, vz
            0, 0, 0,                                        # afx, afy, afz
            0, 0,                                           # yaw, yaw_rate
        )

    def _compute_nesw_points(self, lat, lon):
        """
        Compute 4 landing candidate points at _STANDOFF_RADIUS_M from
        the detection centroid, one in each cardinal direction.

        Returns dict: {"N": {"lat":…, "lon":…, "label":"N"}, "E":…, …}
        """
        R = 6371000.0
        d = _STANDOFF_RADIUS_M
        dlat = math.degrees(d / R)
        dlon = math.degrees(d / (R * math.cos(math.radians(lat))))
        return {
            "N": {"lat": lat + dlat, "lon": lon,        "label": "N"},
            "E": {"lat": lat,        "lon": lon + dlon,  "label": "E"},
            "S": {"lat": lat - dlat, "lon": lon,        "label": "S"},
            "W": {"lat": lat,        "lon": lon - dlon,  "label": "W"},
        }

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2):
        """Great-circle distance in metres between two GPS points."""
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
