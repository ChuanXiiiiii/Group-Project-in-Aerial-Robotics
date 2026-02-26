"""
states/wait.py — WAIT state.
=============================
Monitors heartbeat, watches for commands from:
    1. The trigger file  (/tmp/sar_trigger.txt)
    2. The web dashboard  (via shared status command queue)

Transitions:
    → CHANGE_MODE    when a valid mode command is received.
    → TIMED_HOLD     when a timed hold command is received.
    → UPLOAD_MISSION when waypoint(s) are submitted.
"""

import os
import time
from states.base import BaseState
from config import (
    TRIGGER_FILE, HEARTBEAT_INTERVAL, FLIGHT_MODES, KML_FILE,
    SEARCH_ALT_M, MIN_GPS_SATELLITES, MIN_GPS_FIX_TYPE,
    MIN_BATTERY_PERCENT, ALTITUDE_TOLERANCE_M, HEARTBEAT_MISS_LIMIT,
)


class WaitState(BaseState):

    name = "WAIT"

    def enter(self):
        self.log("Listening for commands ...")
        # ── Preflight monitor state ──
        self._hb_miss_count = 0
        self._last_query_time = 0
        # Manual checkboxes (persisted in shared dict across re-entries)
        if "preflight_wp_verified" not in self.shared:
            self.shared["preflight_wp_verified"] = False
        if "preflight_pilot_ready" not in self.shared:
            self.shared["preflight_pilot_ready"] = False

    def execute(self):
        while True:
            # ── Heartbeat ──
            mode_name, mode_num = self.drone.get_mode()
            hb_ok = (mode_name != "UNKNOWN")

            if hb_ok:
                self._hb_miss_count = 0
            else:
                self._hb_miss_count += 1

            if hb_ok:
                self.log(f"mode={mode_name}")

            # ── Update preflight checks ──
            self._update_preflight_checks(mode_name, mode_num, hb_ok)

            # ── Check GUI command (via shared status) ──
            if self.drone.status:
                cmd = self.drone.status.consume_command()
                if cmd is not None:
                    next_state = self._handle_gui_command(cmd)
                    if next_state is not None:
                        return next_state

            # ── Check trigger file ──
            if os.path.exists(TRIGGER_FILE):
                next_state = self._read_trigger()
                if next_state is not None:
                    return next_state

            time.sleep(HEARTBEAT_INTERVAL)

    def _handle_gui_command(self, cmd):
        """
        Process a command dict from the GUI.
        Returns next state name, or None to stay in WAIT.
        """
        cmd_type = cmd.get("type")

        if cmd_type == "mode_change":
            mode = cmd.get("mode", "").upper()
            if mode in FLIGHT_MODES:
                self.shared["requested_mode"] = mode
                self.log(f"GUI COMMAND: switch to {mode}")
                return "CHANGE_MODE"

        elif cmd_type == "timed_hold":
            mode = cmd.get("mode", "").upper()
            duration = cmd.get("duration", 10)
            if mode in FLIGHT_MODES:
                self.shared["timed_hold_target"] = mode
                self.shared["timed_hold_duration"] = duration
                self.shared["timed_hold_phase"] = 1
                self.log(f"GUI COMMAND: TIMED_HOLD {mode} for {duration}s")
                return "TIMED_HOLD"

        elif cmd_type == "upload_mission":
            return self._prepare_mission_upload(
                cmd.get("waypoints", []),
                source="GUI",
            )

        elif cmd_type == "upload_fence":
            return self._prepare_fence_upload(
                kml_path=cmd.get("kml_path", KML_FILE),
                source="GUI",
            )

        elif cmd_type == "generate_pattern":
            self.log("GUI COMMAND: generate search pattern")
            return "GENERATE_PATTERN"

        elif cmd_type == "confirm_pattern":
            return self._confirm_pattern(source="GUI")

        elif cmd_type == "cancel_pattern":
            self.shared.pop("pending_pattern", None)
            self.shared["generate_pattern_status"] = "cancelled"
            self.log("GUI COMMAND: pattern cancelled by operator")
            if self.drone.status:
                self.drone.status.set_extra("pattern_status", "cancelled")
                self.drone.status.set_extra("pending_pattern_info", None)
            return None

        elif cmd_type == "preflight_toggle":
            field = cmd.get("field")
            if field == "wp_verified":
                self.shared["preflight_wp_verified"] = not self.shared.get(
                    "preflight_wp_verified", False)
                self.log(f"GUI: Waypoints verified = "
                         f"{self.shared['preflight_wp_verified']}")
            elif field == "pilot_ready":
                self.shared["preflight_pilot_ready"] = not self.shared.get(
                    "preflight_pilot_ready", False)
                self.log(f"GUI: Safety pilot ready = "
                         f"{self.shared['preflight_pilot_ready']}")
            return None

        elif cmd_type == "start_search":
            # Only allow if all preflight checks pass
            checks = self.drone.status.get_status().get("preflight_checks") \
                     if self.drone.status else None
            if checks and checks.get("mission_ready"):
                self.log("GUI COMMAND: START SEARCH — all checks passed")
                return "SEARCH"
            else:
                self.log("GUI COMMAND: START SEARCH rejected — "
                         "preflight checks not satisfied")
                return None

        self.log(f"GUI COMMAND: unknown — {cmd}")
        return None

    def _validate_waypoint(self, lat, lon, alt, index, source):
        """
        Validate a single waypoint's coordinates.
        Returns (lat, lon, alt) floats on success, None on error.
        """
        try:
            lat = float(lat)
            lon = float(lon)
            alt = float(alt)
        except (TypeError, ValueError) as e:
            self.log(f"{source}: WP {index} invalid coordinates — {e}")
            return None

        if not (-90 <= lat <= 90):
            self.log(f"{source}: WP {index} latitude {lat} out of range [-90, 90]")
            return None
        if not (-180 <= lon <= 180):
            self.log(f"{source}: WP {index} longitude {lon} out of range [-180, 180]")
            return None
        if not (0 <= alt <= 50):
            self.log(f"{source}: WP {index} altitude {alt}m outside allowed range [0, 50] (R04)")
            return None

        return lat, lon, alt

    def _prepare_mission_upload(self, waypoints, source="TRIGGER"):
        """
        Validate a list of waypoint dicts and prepare for upload.
        Each waypoint must have "lat", "lon", "alt" keys.
        Returns next state name or None on error.
        """
        if not waypoints:
            self.log(f"{source}: no waypoints provided — ignoring")
            return None

        validated = []
        for i, wp in enumerate(waypoints):
            result = self._validate_waypoint(
                wp.get("lat"), wp.get("lon"), wp.get("alt"),
                index=i + 1, source=source,
            )
            if result is None:
                return None  # abort entire upload on any bad waypoint
            lat, lon, alt = result
            validated.append({"lat": lat, "lon": lon, "alt": alt})

        self.shared["waypoints"] = validated
        self.shared["return_to"] = "WAIT"
        self.log(f"{source} COMMAND: MISSION with {len(validated)} waypoint(s)")
        return "UPLOAD_MISSION"

    def _prepare_fence_upload(self, kml_path=None, source="TRIGGER"):
        """
        Parse the KML file and prepare fence polygons for upload.
        Uses kml_parser to extract Flight Area (inclusion) and
        SSSI (exclusion) polygons.
        """
        if kml_path is None:
            kml_path = KML_FILE

        try:
            from kml_parser import get_fence_polygons
        except ImportError:
            self.log(f"{source}: ERROR: kml_parser module not found")
            return None

        if not os.path.exists(kml_path):
            self.log(f"{source}: ERROR: KML file not found at '{kml_path}'")
            return None

        try:
            flight_area, sssi = get_fence_polygons(kml_path)
        except (KeyError, Exception) as e:
            self.log(f"{source}: ERROR parsing KML — {e}")
            return None

        if len(flight_area) < 3:
            self.log(f"{source}: ERROR: Flight Area has {len(flight_area)} "
                     f"points (need ≥3)")
            return None

        if len(sssi) < 3:
            self.log(f"{source}: ERROR: SSSI has {len(sssi)} points (need ≥3)")
            return None

        polygons = [
            {
                "type": "inclusion",
                "points": flight_area,
                "label": "Flight Area",
            },
            {
                "type": "exclusion",
                "points": sssi,
                "label": "SSSI",
            },
        ]

        self.shared["fence_polygons"] = polygons
        self.shared["return_to"] = "WAIT"
        self.log(f"{source} COMMAND: FENCE upload — "
                 f"Flight Area ({len(flight_area)} pts, inclusion), "
                 f"SSSI ({len(sssi)} pts, exclusion)")
        return "UPLOAD_FENCE"

    def _update_preflight_checks(self, mode_name, mode_num, hb_ok):
        """
        Evaluate all preflight conditions and push status to GUI.
        Runs every poll cycle in the WAIT loop.
        """
        telem = self.drone.get_telemetry()

        # ── 1. Heartbeat (rolling counter, 5 misses to fail) ──
        heartbeat_pass = self._hb_miss_count < HEARTBEAT_MISS_LIMIT

        # ── 2. GPS fix quality ──
        gps_fix = telem.get("gps_fix")
        sats = telem.get("satellites")
        gps_pass = (gps_fix is not None
                    and gps_fix >= MIN_GPS_FIX_TYPE
                    and sats is not None
                    and sats >= MIN_GPS_SATELLITES)

        # ── 3. Armed ──
        armed = self.drone.get_armed()
        armed_pass = (armed is True)

        # ── 4. Mode is GUIDED or LOITER ──
        mode_pass = mode_name in ("GUIDED", "LOITER")

        # ── 5. Altitude in range ──
        alt = telem.get("alt")
        alt_pass = (alt is not None
                    and abs(alt - SEARCH_ALT_M) <= ALTITUDE_TOLERANCE_M)

        # ── 6. Mission uploaded (query every ~10 cycles to avoid spam) ──
        # Use a cached value, refresh periodically
        now = time.time()
        if now - self._last_query_time > 20:    # query every ~20 seconds
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
        mission_pass = (mission_count > 1)   # >1 because seq 0 is home

        # ── 7. Fences uploaded ──
        fence_count = self.shared.get("_cached_fence_count", 0)
        fence_pass = (fence_count >= 2)

        # ── 8. Battery sufficient ──
        bat_pct = telem.get("battery_pct")
        battery_pass = (bat_pct is not None
                        and bat_pct >= 0
                        and bat_pct >= MIN_BATTERY_PERCENT)

        # ── 9. Waypoints verified (manual checkbox) ──
        wp_verified = self.shared.get("preflight_wp_verified", False)

        # ── 10. Safety pilot ready (manual checkbox) ──
        pilot_ready = self.shared.get("preflight_pilot_ready", False)

        # ── Compute overall readiness ──
        all_checks = [
            heartbeat_pass, gps_pass, armed_pass, mode_pass,
            alt_pass, mission_pass, fence_pass, battery_pass,
            wp_verified, pilot_ready,
        ]
        mission_ready = all(all_checks)

        # ── Build status dict for GUI ──
        checks = {
            "heartbeat":    {"pass": heartbeat_pass,
                             "detail": f"miss {self._hb_miss_count}/{HEARTBEAT_MISS_LIMIT}"},
            "gps":          {"pass": gps_pass,
                             "detail": f"fix={gps_fix} sats={sats}"},
            "armed":        {"pass": armed_pass,
                             "detail": "armed" if armed else "disarmed"},
            "mode":         {"pass": mode_pass,
                             "detail": mode_name},
            "altitude":     {"pass": alt_pass,
                             "detail": f"{alt:.1f}m" if alt is not None
                                       else "no data"},
            "mission":      {"pass": mission_pass,
                             "detail": f"{mission_count} items"},
            "fences":       {"pass": fence_pass,
                             "detail": f"{fence_count} items"},
            "battery":      {"pass": battery_pass,
                             "detail": f"{bat_pct}%" if bat_pct is not None
                                       and bat_pct >= 0 else "no data"},
            "wp_verified":  {"pass": wp_verified,
                             "detail": "operator checkbox"},
            "pilot_ready":  {"pass": pilot_ready,
                             "detail": "operator checkbox"},
            "mission_ready": mission_ready,
        }

        if self.drone.status:
            self.drone.status.set_extra("preflight_checks", checks)

    def _confirm_pattern(self, source="GUI"):
        """
        Confirm the staged pattern — move pending waypoints
        to shared["waypoints"] and transition to UPLOAD_MISSION.
        """
        pending = self.shared.get("pending_pattern")
        if not pending:
            self.log(f"{source}: no pending pattern to confirm")
            return None

        waypoints = pending["waypoints"]
        if not waypoints:
            self.log(f"{source}: pending pattern has no waypoints")
            self.shared.pop("pending_pattern", None)
            return None

        # Validate all waypoints through the existing validator
        validated = []
        for i, wp in enumerate(waypoints):
            result = self._validate_waypoint(
                wp.get("lat"), wp.get("lon"), wp.get("alt"),
                index=i + 1, source=f"{source} PATTERN",
            )
            if result is None:
                return None
            lat, lon, alt = result
            validated.append({"lat": lat, "lon": lon, "alt": alt})

        # Move to mission upload
        self.shared["waypoints"] = validated
        self.shared["return_to"] = "WAIT"
        self.shared.pop("pending_pattern", None)
        self.shared["generate_pattern_status"] = "confirmed"
        if self.drone.status:
            self.drone.status.set_extra("pattern_status", "confirmed")
            self.drone.status.set_extra("pending_pattern_info", None)
        self.log(f"{source}: pattern CONFIRMED — uploading "
                 f"{len(validated)} waypoints")
        if self.drone.status:
            self.drone.status.set_extra("pattern_status", "confirmed")
            self.drone.status.set_extra("pending_pattern_info", None)
        return "UPLOAD_MISSION"

    def _read_trigger(self):
        try:
            with open(TRIGGER_FILE, "r") as f:
                raw = f.read().strip()
            os.remove(TRIGGER_FILE)

            lines = [l.strip() for l in raw.splitlines() if l.strip()]
            if not lines:
                return None

            parts = lines[0].split()
            command = parts[0].upper()

            # ── Simple mode change: "LOITER" ──
            if command in FLIGHT_MODES:
                self.shared["requested_mode"] = command
                self.log(f"FILE TRIGGER: switch to {command}")
                return "CHANGE_MODE"

            # ── Timed hold: "TIMED_HOLD LOITER 10" ──
            if command == "TIMED_HOLD" and len(parts) >= 2:
                target = parts[1].upper()
                duration = int(parts[2]) if len(parts) >= 3 else 10

                if target not in FLIGHT_MODES:
                    self.log(f"TRIGGER: unknown mode '{target}' — ignoring")
                    return None

                self.shared["timed_hold_target"] = target
                self.shared["timed_hold_duration"] = duration
                self.shared["timed_hold_phase"] = 1
                self.log(f"FILE TRIGGER: TIMED_HOLD {target} for {duration}s")
                return "TIMED_HOLD"

            # ── Mission: multi-line format ──
            # Line 1: "MISSION <default_alt>"
            # Lines 2+: "lat,lon" or "lat,lon,alt"
            #
            # Example:
            #   MISSION 30
            #   51.4230,-2.6710
            #   51.4235,-2.6700
            #   51.4240,-2.6710,25
            if command == "MISSION" and len(lines) >= 2:
                default_alt = float(parts[1]) if len(parts) >= 2 else 30.0
                waypoints = []

                for line in lines[1:]:
                    coords = line.split(",")
                    if len(coords) < 2:
                        self.log(f"FILE TRIGGER: bad waypoint line '{line}' — ignoring")
                        return None
                    lat = coords[0].strip()
                    lon = coords[1].strip()
                    alt = coords[2].strip() if len(coords) >= 3 else str(default_alt)
                    waypoints.append({"lat": lat, "lon": lon, "alt": alt})

                return self._prepare_mission_upload(waypoints, source="FILE TRIGGER")

            # ── Single waypoint shorthand: "WAYPOINT lat lon alt" ──
            if command == "WAYPOINT" and len(parts) >= 4:
                waypoints = [{"lat": parts[1], "lon": parts[2], "alt": parts[3]}]
                return self._prepare_mission_upload(waypoints, source="FILE TRIGGER")

            # ── Fence upload from KML: "FENCE" or "FENCE /path/to/file.kml" ──
            if command == "FENCE":
                kml_path = parts[1] if len(parts) >= 2 else KML_FILE
                return self._prepare_fence_upload(
                    kml_path=kml_path,
                    source="FILE TRIGGER",
                )

            # ── Pattern generation: "PATTERN" ──
            if command == "PATTERN":
                self.log("FILE TRIGGER: generate search pattern")
                return "GENERATE_PATTERN"

            # ── Start search: "SEARCH" ──
            if command == "SEARCH":
                self.log("FILE TRIGGER: start search")
                return "SEARCH"

            self.log(f"TRIGGER: unknown command '{lines[0]}' — ignoring")
            return None

        except Exception as e:
            self.log(f"TRIGGER ERROR: {e}")
            return None