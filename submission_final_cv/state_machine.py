# state machine handlers (mixin). lives on VisualFlightMission.

import time, math
import os
import json
from datetime import datetime

from states import State
import config
from navigation import NavigationController
from stream_server import add_detection_event


class StateHandlersMixin:
    """Per-state handlers. Mixed into VisualFlightMission."""

    # -- State transition --

    def _set_state(self, new_state):
        old_state = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self._arming_timeout_warned = False
        self._takeoff_timeout_warned = False
        self._centering_timeout_warned = False
        self._descending_timeout_warned = False
        self._land_cmd_sent = False
        self._verify_last_warn = -1
        self._verify_remaining = None
        elapsed = time.time() - getattr(self, '_mission_start_time', time.time())
        mins, secs = int(elapsed // 60), int(elapsed % 60)
        print(f"\n{'='*60}")
        print(f"  STATE: {old_state} --> {new_state}  [T+{mins:02d}:{secs:02d}]")
        print(f"{'='*60}")

    # -- Helpers --

    def _current_search_alt(self):
        # rescan passes drop the altitude each time
        if self.rescan_pass == 0:
            return config.TARGET_ALT
        return getattr(self, '_rescan_alt', config.TARGET_ALT)

    def get_dist_to_target(self):
        return self.get_dist_to_point(self.target_lat, self.target_lon)

    def get_dist_to_point(self, t_lat, t_lon):
        s = 111132.0
        return math.sqrt(((self.lat - t_lat) * s) ** 2 +
                         ((self.lon - t_lon) * s * math.cos(math.radians(self.lat))) ** 2)

    @staticmethod
    def _gps_dist(lat1, lon1, lat2, lon2):
        s = 111132.0
        return math.sqrt(((lat1 - lat2) * s) ** 2 +
                         ((lon1 - lon2) * s * math.cos(math.radians(lat1))) ** 2)

    # -- Detection image saving --

    def _save_detection_image(self, decision, lat, lon, conf=0.0):
        # dumps current frame + small JSON next to it. silent on failure.
        import cv2
        frame = getattr(self, 'last_frame', None)
        if frame is None:
            return
        det_dir = "mission_detections"
        os.makedirs(det_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"{decision}_{ts}_{lat:.6f}_{lon:.6f}"
        img_path = os.path.join(det_dir, base + ".jpg")
        json_path = os.path.join(det_dir, base + ".json")
        cv2.imwrite(img_path, frame)
        meta = {
            "decision": decision,
            "target_lat": lat,
            "target_lon": lon,
            "confidence": round(conf, 4),
            "altitude": round(self.alt, 1),
            "timestamp": ts,
            "drone_lat": round(self.lat, 7),
            "drone_lon": round(self.lon, 7),
        }
        try:
            with open(json_path, 'w') as f:
                json.dump(meta, f, indent=2)
        except Exception as e:
            print(f"[WARN] Failed to save detection json: {e}")

    # -- Detection dedup --

    def _is_near_known(self, lat, lon):
        # True if this point is within r of any rejected / IOI / queued target
        r = config.REJECTED_TARGET_RADIUS_M
        for rej_lat, rej_lon in self.rejected_targets:
            if self._gps_dist(lat, lon, rej_lat, rej_lon) < r:
                return True
        for item in getattr(self, 'items_of_interest', []):
            if self._gps_dist(lat, lon, item['lat'], item['lon']) < r:
                return True
        # TODO: the queue check duplicates logic, ran out of time before demo to merge into one pass
        for q_item in getattr(self, '_detect_queue', []):
            if self._gps_dist(lat, lon, q_item[0], q_item[1]) < r:
                return True
        return False

    def _is_inside_nfz(self, lat, lon):
        if hasattr(self, 'geofence') and self.geofence:
            _, inside = self.geofence.distance_to_boundary(lat, lon)
            return inside
        return False

    def _is_outside_search_area(self, lat, lon):
        if hasattr(self, 'planner') and self.planner and hasattr(self.planner, 'search_polygon'):
            poly = self.planner.search_polygon
            if poly and len(poly) >= 3:
                import cv2
                import numpy as np
                pt = self.planner.geo.gps_to_pixels(lat, lon)
                contour = np.array(poly, dtype=np.float32).reshape(-1, 1, 2)
                result = cv2.pointPolygonTest(contour, (float(pt[0]), float(pt[1])), False)
                return result < 0
        return False

    def _enqueue_detection(self, lat, lon, conf):
        # push onto queue, drop oldest if full
        if not hasattr(self, '_detect_queue'):
            self._detect_queue = []
        self._detect_queue.append((lat, lon, conf))
        max_q = getattr(config, 'MAX_DETECT_QUEUE', 20)
        while len(self._detect_queue) > max_q:
            dropped = self._detect_queue.pop(0)
            print(f"[QUEUE] Dropped oldest detection ({dropped[0]:.6f}, {dropped[1]:.6f}) — queue full ({max_q})")

    def _pop_valid_target(self):
        while getattr(self, '_detect_queue', None) and len(self._detect_queue) > 0:
            q_lat, q_lon, _qc = self._detect_queue.pop(0)
            if self._is_inside_nfz(q_lat, q_lon) or \
               self._is_outside_search_area(q_lat, q_lon) or \
               self._is_near_known(q_lat, q_lon):
                print(f"Skipping invalid queued target ({q_lat:.6f}, {q_lon:.6f})")
                continue
            return q_lat, q_lon, _qc
        return None

    # -- Per-state handlers --

    def _handle_init(self, target_found, px_u, px_v, key):
        from pymavlink import mavutil
        if time.time() - self.last_req > 1.0:
            if not getattr(self, '_init_connect_printed', False):
                print(f"Connecting to {config.CONNECTION_STR}...")
                self._init_connect_printed = True
            try:
                self.master = mavutil.mavlink_connection(config.CONNECTION_STR)
                self.nav = NavigationController(
                    self.master, no_turn=True, get_yaw=lambda: self.yaw)
                self.connect_start_time = time.time()
                self._set_state(State.CONNECTING)
            except Exception as e:
                print(f"[ERROR] Connection failed: {e}")
                print(f"  ACTION: Check mavproxy is running. Verify port {config.CONNECTION_STR}.")
            self.last_req = time.time()

    def _handle_connecting(self, target_found, px_u, px_v, key):
        from pymavlink import mavutil
        if self.connect_start_time > 0 and self.last_heartbeat == 0:
            elapsed = time.time() - self.connect_start_time
            if elapsed > 15 and int(elapsed) % 15 == 0 and time.time() - self.last_req > 5:
                print(f"[WARN] No heartbeat in {int(elapsed)}s.")
                print(f"  ACTION: Check mavproxy is running and Cube is powered.")
                self.last_req = time.time()
        if self.last_heartbeat > 0:
            print("Heartbeat. Requesting Data Stream...")
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
            if config.MODE == "SIMULATION":
                self.master.mav.param_set_send(
                    self.master.target_system, self.master.target_component,
                    b'SIM_SPEEDUP', config.SIM_SPEED, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                print(f"  SITL speedup set to {config.SIM_SPEED}x")
            self._set_state(State.ARMING)

    def _handle_arming(self, target_found, px_u, px_v, key):
        from pymavlink import mavutil
        arming_elapsed = time.time() - self.state_start_time
        if arming_elapsed > 120 and not self._arming_timeout_warned:
            print("[WARN] ARMING TIMEOUT (120s) — cannot arm.")
            print("  ACTION: Check GPS fix, safety switch, RC failsafe.")
            print("  Open Mission Planner Messages tab for pre-arm failure reason.")
            self._arming_timeout_warned = True

        if not self.gps_fix_ok:
            gps_msg = self.master.recv_match(type='GPS_RAW_INT', blocking=False)
            if gps_msg:
                fix_type, sats = gps_msg.fix_type, gps_msg.satellites_visible
                if fix_type >= 3 and sats >= 6:
                    self.gps_fix_ok = True
                    self.lat = gps_msg.lat / 1e7
                    self.lon = gps_msg.lon / 1e7
                    self.home_lat, self.home_lon = self.lat, self.lon
                    print(f"GPS FIX OK — fix={fix_type}, sats={sats}, ({self.lat:.6f}, {self.lon:.6f})")
                    # Check distance from TOL (R03)
                    if hasattr(config, 'TAKEOFF_GPS') and config.TAKEOFF_GPS[0] != 0:
                        tol_lat, tol_lon = config.TAKEOFF_GPS
                        tol_dist = math.sqrt(((self.lat - tol_lat) * 111320) ** 2 +
                                             ((self.lon - tol_lon) * 111320 * math.cos(math.radians(self.lat))) ** 2)
                        if tol_dist > 5.0:
                            print(f"WARNING: {tol_dist:.1f}m from TOL (limit 5m) — check drone position")
                        else:
                            print(f"  TOL check: {tol_dist:.1f}m from defined takeoff point (OK)")
                elif time.time() - self._last_gps_status_print > 5.0:
                    print(f"Waiting for GPS fix... (fix={fix_type}, sats={sats})")
                    self._last_gps_status_print = time.time()
            elif time.time() - self._last_gps_status_print > 5.0:
                print("Waiting for GPS fix... (no GPS_RAW_INT yet)")
                self._last_gps_status_print = time.time()
        elif self.master.motors_armed():
            print("Armed! Taking Off...")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, config.TARGET_ALT)
            self._set_state(State.TAKEOFF)
        elif time.time() - self.last_req > 3.0:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 0, 0, 0, 0, 0)
            mode_ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if mode_ack:
                if mode_ack.result != 0:
                    print(f"SET_MODE REJECTED: result={mode_ack.result}")
                else:
                    print("SET_MODE (GUIDED) accepted")
            time.sleep(0.5)
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            arm_ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if arm_ack:
                if arm_ack.result != 0:
                    print(f"ARM REJECTED: result={arm_ack.result}")
                else:
                    print("ARM command accepted")
            self.last_req = time.time()

    def _handle_takeoff(self, target_found, px_u, px_v, key):
        if self.master and not self.master.motors_armed():
            if config.MODE == "SIMULATION":
                print("Drone disarmed during takeoff — retrying arm sequence...")
                self._set_state(State.ARMING)
            else:
                print("DRONE DISARMED — safety stop. Re-arm manually via RC.")
                self._set_state(State.DONE)
            return

        if time.time() - self.state_start_time > 60 and not self._takeoff_timeout_warned:
            print(f"[WARN] TAKEOFF TIMEOUT (60s) — alt {self.alt:.1f}m / {config.TARGET_ALT}m target.")
            print("  ACTION: Check propellers spinning, GPS lock, no obstructions.")
            self._takeoff_timeout_warned = True

        if self.alt >= config.TARGET_ALT * 0.90:
            print("Target Altitude Reached.")
            if config.MODE == "SIMULATION":
                canvas_w, canvas_h = self.sim.map_w, self.sim.map_h
            else:
                canvas_w, canvas_h = config.REAL_CANVAS_SIZE, config.REAL_CANVAS_SIZE
            start_gps = self.pre_waypoints[-1] if self.pre_waypoints else (self.lat, self.lon)
            self.waypoints = self.planner.generate_search_pattern(canvas_w, canvas_h, start_gps)
            if self.pre_waypoints:
                print(f"Flying {len(self.pre_waypoints)} transit waypoints first.")
                self.pre_wp_index = 0
                self._set_state(State.PRE_WAYPOINTS)
                self.nav.last_speed_req = 0
            elif self.waypoints:
                print(f"Path generated. Transiting to start: {self.waypoints[0]}")
                self._set_state(State.TRANSIT_TO_SEARCH)
                self.nav.last_speed_req = 0
            else:
                print("No Waypoints generated.")
                self._set_state(State.HOVER)

    def _handle_pre_waypoints(self, target_found, px_u, px_v, key):
        self.nav.set_speed(config.TRANSIT_SPEED_MPS)
        if self.pre_wp_index < len(self.pre_waypoints):
            wp = self.pre_waypoints[self.pre_wp_index]
            if time.time() - self.last_req > 2.0:
                self.nav.send_global_target(wp[0], wp[1], config.TARGET_ALT)
                self.last_req = time.time()
            if self.get_dist_to_point(wp[0], wp[1]) < 2.0:
                self.pre_wp_index += 1
                print(f"Pre-waypoint {self.pre_wp_index}/{len(self.pre_waypoints)} reached.")
        else:
            print("All pre-waypoints complete. Starting search pattern.")
            if self.waypoints:
                self._set_state(State.TRANSIT_TO_SEARCH)
            else:
                print("No search waypoints generated.")
                self._set_state(State.HOVER)

    def _handle_return_from_manual(self, target_found, px_u, px_v, key):
        # Safety checks before resuming autonomous flight
        if not getattr(self, '_resume_validated', False):
            issues = []
            if self.gps_fix_type < 3 or self.gps_satellites < 4:
                issues.append(f"GPS poor (fix={self.gps_fix_type}, sats={self.gps_satellites})")
            if hasattr(self, 'geofence') and self.geofence:
                _, nfz_inside = self.geofence.distance_to_boundary(self.lat, self.lon)
                if nfz_inside:
                    issues.append("INSIDE NFZ — fly out before resuming")
            if issues:
                if time.time() - getattr(self, '_resume_warn_time', 0) > 5.0:
                    for iss in issues:
                        print(f"[RESUME BLOCKED] {iss}")
                    self._resume_warn_time = time.time()
                return  # Stay in RETURN_FROM_MANUAL until safe
            self._resume_validated = True
            print(f"[RESUME] Safety checks passed. Returning to departure point at {self.manual_departure_alt:.0f}m.")

        self.nav.set_speed(config.TRANSIT_SPEED_MPS)
        # Always return at the departure altitude (safe altitude)
        if time.time() - self.last_req > 2.0:
            self.nav.send_global_target(self.manual_departure_lat,
                                        self.manual_departure_lon,
                                        self.manual_departure_alt)
            self.last_req = time.time()
        if self.get_dist_to_point(self.manual_departure_lat, self.manual_departure_lon) < 3.0:
            print(f"Back at manual departure point. Resuming {self.previous_state}.")
            self.last_req = 0
            self._resume_validated = False
            self._set_state(self.previous_state)

    def _handle_transit_to_search(self, target_found, px_u, px_v, key):
        self.nav.set_speed(config.TRANSIT_SPEED_MPS)
        target = self.waypoints[0]
        if time.time() - self.last_req > 2.0:
            self.nav.send_global_target(target[0], target[1], config.TARGET_ALT)
            self.last_req = time.time()
        if self.get_dist_to_point(target[0], target[1]) < 2.0:
            print("Reached Search Start Point. Beginning Pattern.")
            self._set_state(State.SEARCH)
            self.wp_index = 0

    def _handle_return_to_search(self, target_found, px_u, px_v, key):
        search_alt = self._current_search_alt()
        self.nav.set_speed(config.TRANSIT_SPEED_MPS)
        if time.time() - self.last_req > 2.0:
            self.nav.send_global_target(self.departure_lat, self.departure_lon, search_alt)
            self.last_req = time.time()
        if self.get_dist_to_point(self.departure_lat, self.departure_lon) < 3.0 and self.alt > search_alt * 0.85:
            print("Back at departure point. Resuming search pattern.")
            self.departure_lat = self.departure_lon = 0
            self._set_state(State.SEARCH)

    # -- Search sub-methods --

    def _orient_search_yaw(self):
        """Align yaw to scan direction. Returns True while still rotating."""
        if getattr(self, '_search_yaw_done', False):
            return False
        if len(self.waypoints) < 2 and not hasattr(self.planner, 'last_scan_angle'):
            return False

        from pymavlink import mavutil

        if not getattr(self, '_search_yaw_sent', False):
            if len(self.waypoints) >= 2:
                wp0, wp1 = self.waypoints[0], self.waypoints[1]
                dlat, dlon = wp1[0] - wp0[0], wp1[1] - wp0[1]
                yaw_deg = math.degrees(math.atan2(dlon * math.cos(math.radians(wp0[0])), dlat)) % 360
            elif hasattr(self.planner, 'last_scan_angle'):
                yaw_deg = self.planner.last_scan_angle
            else:
                self._search_yaw_done = True
                return False
            if config.DIAGONAL_YAW_OFFSET_DEG is not None:
                diag_offset = config.DIAGONAL_YAW_OFFSET_DEG
            else:
                diag_offset = math.degrees(math.atan2(config.IMAGE_W, config.IMAGE_H))
            yaw_deg = (yaw_deg + diag_offset) % 360
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                yaw_deg, 45, 1, 0, 0, 0, 0)
            self._search_yaw_sent = True
            self._search_yaw_target = yaw_deg
            self._search_yaw_time = time.time()
            print(f"[YAW] Orienting to {yaw_deg:.0f} deg (diagonal alignment)...")
            return True

        yaw_error = abs(math.degrees(self.yaw) - self._search_yaw_target) % 360
        if yaw_error > 180:
            yaw_error = 360 - yaw_error
        if yaw_error < 10 or time.time() - self._search_yaw_time > 5.0:
            self._search_yaw_done = True
            print("[YAW] Aligned. Starting search pattern.")
            return False
        return True

    def _process_detection(self, target_found, px_u, px_v):
        """Filter, confirm, and queue valid detections."""
        # bit hacky, revisit: pulls SMART_DETECT off __main__ rather than
        # threading a flag through the constructor. works for now.
        import __main__ as _main
        smart_detect = getattr(_main, 'SMART_DETECT', False)

        if not hasattr(self, '_detect_queue'):
            self._detect_queue = []
            self._consecutive_detect_count = 0

        if target_found:
            self.calculate_target_gps(px_u, px_v)
            if self._is_inside_nfz(self.target_lat, self.target_lon) or \
               self._is_outside_search_area(self.target_lat, self.target_lon):
                self._consecutive_detect_count = 0
                return
            if self._is_near_known(self.target_lat, self.target_lon):
                self._consecutive_detect_count = 0
                return
            conf = getattr(self, 'current_conf', 0.5)
            if smart_detect:
                self._consecutive_detect_count += 1
                if self._consecutive_detect_count >= config.DETECT_CONFIRM_FRAMES:
                    print(f"[SMART] Confirmed ({self._consecutive_detect_count} frames) at ({self.target_lat:.6f}, {self.target_lon:.6f}) conf={conf:.2f}")
                    self._enqueue_detection(self.target_lat, self.target_lon, conf)
                    self._save_detection_image("SEARCH", self.target_lat, self.target_lon, conf)
                    self._consecutive_detect_count = 0
            else:
                if not self._is_near_known(self.target_lat, self.target_lon):
                    print(f"[DETECT] Target at ({self.target_lat:.6f}, {self.target_lon:.6f}) conf={conf:.2f} alt={self.alt:.0f}m")
                    self._enqueue_detection(self.target_lat, self.target_lon, conf)
                    self._save_detection_image("SEARCH", self.target_lat, self.target_lon, conf)
        else:
            self._consecutive_detect_count = 0

    def _advance_waypoint(self):
        """Fly toward next waypoint; start rescan or finish when exhausted."""
        if self.wp_index < len(self.waypoints):
            target = self.waypoints[self.wp_index]
            if time.time() - self.last_req > 2.0:
                # Maintain search yaw if --lock-yaw flag is set
                search_yaw = getattr(self, '_search_yaw_target', None)
                yaw_arg = math.radians(search_yaw) if config.LOCK_YAW and search_yaw is not None else None
                self.nav.send_global_target(target[0], target[1], self._current_search_alt(), yaw=yaw_arg)
                self.last_req = time.time()
            if self.get_dist_to_point(target[0], target[1]) < 2.0:
                self.wp_index += 1
        elif self.rescan_pass < self.max_rescan_passes:
            current_alt = self._current_search_alt()
            new_alt = max(config.RESCAN_ALT_FLOOR_M, current_alt * config.RESCAN_ALT_FACTOR)
            if new_alt <= config.RESCAN_ALT_FLOOR_M:
                print(f"[WARN] Rescan floor reached ({config.RESCAN_ALT_FLOOR_M}m). No confirmed target.")
                print("  ACTION: Lower RESCAN_ALT_FLOOR_M in config.py or improve model.")
                self._set_state(State.DONE)
                return
            self.rescan_pass += 1
            self._rescan_alt = new_alt
            print(f"[RESCAN] Pass {self.rescan_pass + 1}: altitude {current_alt:.0f}m -> {new_alt:.0f}m")
            if config.MODE == "SIMULATION":
                canvas_w, canvas_h = self.sim.map_w, self.sim.map_h
            else:
                canvas_w, canvas_h = config.REAL_CANVAS_SIZE, config.REAL_CANVAS_SIZE
            self.waypoints = self.planner.generate_search_pattern(
                canvas_w, canvas_h, (self.lat, self.lon), alt_override=new_alt)
            self.wp_index = 0
            self._search_yaw_done = False
            self._search_yaw_sent = False
            self._set_state(State.SEARCH)
        else:
            self._set_state(State.DONE)

    # -- Search orchestrator --

    def _handle_search(self, target_found, px_u, px_v, key):
        if self._orient_search_yaw():
            return

        # Speed: slower at low altitude or in focus area
        if getattr(self, '_beacon_triggered', False):
            search_speed = min(config.FOCUS_SEARCH_SPEED_MPS, config.speed_for_altitude(self.alt))
        else:
            search_speed = config.speed_for_altitude(self.alt)
        self.nav.set_speed(search_speed)

        # Auto-trigger PLB beacon after delay
        beacon_delay = config.BEACON_DELAY
        if beacon_delay > 0 and not getattr(self, '_beacon_triggered', False):
            if not hasattr(self, '_search_first_start'):
                self._search_first_start = time.time()
            if time.time() - self._search_first_start >= beacon_delay:
                self._trigger_beacon_redirect()

        self._process_detection(target_found, px_u, px_v)

        # Pop queued target -> CENTERING, or advance waypoint
        if self.state == State.SEARCH:
            result = self._pop_valid_target()
            if result:
                q_lat, q_lon, q_conf = result
                print(f"Investigating target at ({q_lat:.6f}, {q_lon:.6f}) conf={q_conf:.2f} — {len(self._detect_queue)} remaining")
                self.target_lat, self.target_lon = q_lat, q_lon
                self.departure_lat, self.departure_lon = self.lat, self.lon
                self._locked_target = (q_lat, q_lon)
                self._set_state(State.CENTERING)
        if self.state == State.SEARCH:
            self._advance_waypoint()

    def _handle_centering(self, target_found, px_u, px_v, key):
        import __main__ as _main
        center_verify = getattr(_main, 'CENTER_VERIFY', False)
        sim_tilt = getattr(_main, 'SIM_TILT', False) or getattr(_main, 'COMPENSATE_TILT', False)

        if time.time() - self.state_start_time > 30 and not self._centering_timeout_warned:
            print(f"[WARN] CENTERING TIMEOUT (30s) — dist {self.get_dist_to_target():.1f}m. Resuming search.")
            self._centering_timeout_warned = True
            self._set_state(State.SEARCH)
            return

        # Two-step centering when tilt compensation is active:
        # Step 1: Fly to rough tilt-compensated estimate
        # Step 2: Hover 3s to eliminate tilt, re-detect with nadir math, update estimate
        hover_relock = getattr(self, '_hover_relock_done', False)

        if target_found:
            self.calculate_target_gps(px_u, px_v)
            locked = getattr(self, '_locked_target', None)
            if locked:
                dist_to_locked = self._gps_dist(self.target_lat, self.target_lon, locked[0], locked[1])
                if dist_to_locked <= config.DETECT_LOCK_RADIUS_M:
                    self._locked_target = (self.target_lat, self.target_lon)
                else:
                    # New target outside lock radius — queue if valid
                    if not self._is_inside_nfz(self.target_lat, self.target_lon) and \
                       not self._is_outside_search_area(self.target_lat, self.target_lon) and \
                       not self._is_near_known(self.target_lat, self.target_lon):
                        c = getattr(self, 'current_conf', 0.5)
                        self._enqueue_detection(self.target_lat, self.target_lon, c)
                        print(f"New target during CENTERING queued at ({self.target_lat:.6f}, {self.target_lon:.6f})")
                    self.target_lat, self.target_lon = locked

        if time.time() - self.last_req > 0.2:
            self.nav.send_global_target(self.target_lat, self.target_lon, self.alt)
            self.last_req = time.time()

        # Step 1 complete: arrived at rough estimate
        if self.get_dist_to_target() < 3.0 and sim_tilt and not hover_relock:
            # Hover for 3 seconds to eliminate tilt, then re-detect
            if not hasattr(self, '_hover_relock_start'):
                self._hover_relock_start = time.time()
                print(f"[CENTERING] Step 1 complete — hovering 3s to eliminate tilt for re-detection...")
                # Send hold-position command
                self.nav.send_global_target(self.lat, self.lon, self.alt)
                return
            elif time.time() - self._hover_relock_start < 3.0:
                # Still hovering -- hold position, keep detecting
                self.nav.send_global_target(self.lat, self.lon, self.alt)
                return
            else:
                # 3s elapsed -- pitch/roll should be ~0 now
                self._hover_relock_done = True
                if target_found:
                    print(f"[CENTERING] Step 2 — nadir re-detection at ({self.target_lat:.6f}, {self.target_lon:.6f})")
                    self._locked_target = (self.target_lat, self.target_lon)
                else:
                    print(f"[CENTERING] Step 2 — no detection during hover (false positive?)")
                del self._hover_relock_start
                # Continue to fly toward updated/original estimate

        if self.get_dist_to_target() < 1.0:
            if center_verify:
                self._gps_avg_start = time.time()
                self._gps_avg_samples = []
                self._trig_estimate = (self.target_lat, self.target_lon)
                self._confirmed_y = False
                print(f"[CENTERING] Centered — GPS averaging started, trig=({self.target_lat:.6f}, {self.target_lon:.6f})")
            else:
                self._gps_avg_start = None
                self._gps_avg_samples = None
                self._confirmed_y = False
            self._set_state(State.VERIFY)
            add_detection_event({"time": datetime.now().strftime("%H:%M:%S"),
                "lat": self.target_lat, "lon": self.target_lon,
                "conf": self.current_conf, "decision": "pending"})
            print(f"  Target: ({self.target_lat:.6f}, {self.target_lon:.6f}) at {self.alt:.0f}m")
            print(f"  ACTION: Y=Confirm  N=Reject  I=Interest (120s timeout)")

    def _handle_descending(self, target_found, px_u, px_v, key):
        # Descend to verify altitude (config value or halfway to 3m, whichever is higher)
        search_alt = self._current_search_alt()
        verify_alt = max(config.VERIFY_ALT, (search_alt + 3.0) / 2.0)
        verify_alt = min(verify_alt, search_alt - 2.0)  # at least 2m below search alt
        alt_error = self.alt - verify_alt
        if alt_error > 0.5:
            vz = min(0.5 * alt_error, 1.5)
        elif alt_error < -0.5:
            vz = max(0.5 * alt_error, -1.5)
        else:
            vz = 0
        if time.time() - self.last_req > 0.3:
            self.nav.send_global_target(self.target_lat, self.target_lon, verify_alt, vz=vz)
            self.last_req = time.time()
        # Transition when at verify altitude
        if abs(alt_error) < 1.0:
            print(f"  At {self.alt:.0f}m — ready for operator verification")
            self._set_state(State.VERIFY)

    def _handle_verify(self, target_found, px_u, px_v, key):
        self.waiting_for_confirmation = True
        elapsed_v = time.time() - self.state_start_time
        remaining = 120 - elapsed_v
        self._verify_remaining = remaining  # exposed for HUD countdown

        if remaining <= 0:
            print("[WARN] VERIFY TIMEOUT (120s) — no operator response. Auto-rejecting target.")
            self.rejected_targets.append((self.target_lat, self.target_lon))
            self.waiting_for_confirmation = False
            self.selecting_landing_side = False   # FIX 3: reset stale flags
            self._confirmed_y = False
            # FIX 3: check detection queue before falling back to SEARCH
            next_target = self._pop_valid_target()
            if next_target:
                q_lat, q_lon, _qc = next_target
                print(f"Next queued target at ({q_lat:.6f}, {q_lon:.6f}) — {len(self._detect_queue)} remaining")
                self.target_lat, self.target_lon = q_lat, q_lon
                self._locked_target = (q_lat, q_lon)
                self._set_state(State.CENTERING)
            elif hasattr(self, 'departure_lat') and self.departure_lat:
                self._set_state(State.RETURN_TO_SEARCH)
            else:
                self._set_state(State.SEARCH)
            return

        # Periodic terminal warnings so operator knows time is running out
        _lw = getattr(self, '_verify_last_warn', -1)
        if elapsed_v >= 100 and int(elapsed_v / 10) != int(_lw / 10):
            print(f"  VERIFY: {remaining:.0f}s remaining — press Y/N/I!")
            self._verify_last_warn = elapsed_v
        elif elapsed_v >= 90 and _lw < 90:
            print(f"  VERIFY: {remaining:.0f}s remaining — press Y/N/I!")
            self._verify_last_warn = elapsed_v
        elif elapsed_v >= 60 and _lw < 60:
            print(f"  VERIFY: {remaining:.0f}s remaining — press Y/N/I!")
            self._verify_last_warn = elapsed_v

        self.nav.send_global_target(self.target_lat, self.target_lon, self.alt)
        if getattr(self, '_gps_avg_samples', None) is not None:
            self._gps_avg_samples.append((self.lat, self.lon))
        # Y pressed — wait for 10s averaging then finalize
        if getattr(self, '_confirmed_y', False) and hasattr(self, '_gps_avg_start'):
            elapsed = time.time() - self._gps_avg_start
            if elapsed >= 10.0 and len(self._gps_avg_samples) > 0:
                avg_lat = sum(s[0] for s in self._gps_avg_samples) / len(self._gps_avg_samples)
                avg_lon = sum(s[1] for s in self._gps_avg_samples) / len(self._gps_avg_samples)
                lat_m = 111320.0
                lon_m = 111320.0 * math.cos(math.radians(avg_lat))
                err = math.sqrt(((avg_lat - self.target_lat) * lat_m)**2 +
                                ((avg_lon - self.target_lon) * lon_m)**2)
                print(f"[GPS AVG] {len(self._gps_avg_samples)} samples / {elapsed:.1f}s — diff {err:.1f}m")
                self.target_lat, self.target_lon = avg_lat, avg_lon
                self._confirmed_y = False
                self._gps_avg_samples = None
                print("USER CONFIRMED TARGET. SELECT LANDING SIDE: N/E/S/W")
                self.selecting_landing_side = True

    def _handle_hover(self, target_found, px_u, px_v, key):
        if time.time() - self.state_start_time > 60.0:
            print("[WARN] HOVER TIMEOUT (60s) — no waypoints.")
            print("  ACTION: Check SEARCH_AREA_GPS in config.py or search_area.json.")
            self._set_state(State.DONE)

    def _handle_approach(self, target_found, px_u, px_v, key):
        # Fly to landing spot and descend for payload deploy
        deploy_alt = 3.0
        # Reset speed — SEARCH leaves a low speed (6 m/s) that starves the
        # vertical velocity budget.  TRANSIT_SPEED gives full 3D envelope.
        self.nav.set_speed(config.TRANSIT_SPEED_MPS)
        if time.time() - self.last_req > 0.5:
            # Compute yaw toward landing spot so drone flies FORWARD (not sideways)
            # — ArduCopter is much faster/more capable flying nose-first.
            dy = (self.landing_lat - self.lat) * 111320
            dx = (self.landing_lon - self.lon) * 111320 * math.cos(math.radians(self.lat))
            approach_yaw = math.atan2(dx, dy)  # radians, 0=north
            self.nav.send_global_target(self.landing_lat, self.landing_lon, deploy_alt,
                                        yaw=approach_yaw)
            self.last_req = time.time()
            if self.alt > deploy_alt + 1.0:
                print(f"  [DESCENT] {self.alt:.1f}m → {deploy_alt}m")
        if self.get_dist_to_point(self.landing_lat, self.landing_lon) < 2.0 and self.alt < deploy_alt + 2.0:
            print(f"  At {self.alt:.1f}m above target — payload deploy sequence (15s)")
            self._set_state(State.HOVER_TARGET)

    def _handle_hover_target(self, target_found, px_u, px_v, key):
        from pymavlink import mavutil
        # Hold at deploy altitude — let ArduCopter position controller handle
        self.nav.send_global_target(self.landing_lat, self.landing_lon, 3.0)
        elapsed = time.time() - self.state_start_time
        self._hover_elapsed = elapsed

        SERVO_CH = config.SERVO_CHANNEL
        SERVO_CLOSED = config.SERVO_CLOSE_PWM
        SERVO_S1 = config.SERVO_PARTIAL_PWM
        SERVO_S2 = config.SERVO_FULL_PWM

        if elapsed >= 3.0 and not hasattr(self, '_servo_stage1_done'):
            self._servo_stage1_done = True
            print(f"  SERVO STAGE 1 — partial release at {self.alt:.1f}m (ch{SERVO_CH}, PWM {SERVO_S1})")
            if self.master:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, SERVO_CH, SERVO_S1, 0, 0, 0, 0, 0)

        if elapsed >= 6.0 and not self._servo_released:
            self._servo_released = True
            print(f"  SERVO STAGE 2 — full release at {self.alt:.1f}m (ch{SERVO_CH}, PWM {SERVO_S2})")
            if self.master:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, SERVO_CH, SERVO_S2, 0, 0, 0, 0, 0)

        if elapsed > 15.0:
            if self.master:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, SERVO_CH, SERVO_CLOSED, 0, 0, 0, 0, 0)
            self._servo_released = False
            if hasattr(self, '_servo_stage1_done'):
                del self._servo_stage1_done
            print(f"Hover complete ({elapsed:.0f}s). Climbing and returning home.")
            if self.pre_waypoints:
                self.return_wp_index = len(self.pre_waypoints) - 1
                self._set_state(State.RETURN_TRANSIT)
            else:
                self._set_state(State.RETURN_HOME)

    def _handle_return_transit(self, target_found, px_u, px_v, key):
        return_alt = self._current_search_alt()
        self.nav.set_speed(config.TRANSIT_SPEED_MPS)
        if self.return_wp_index >= 0:
            wp = self.pre_waypoints[self.return_wp_index]
            if time.time() - self.last_req > 0.5:
                vz = -2.5 if self.alt < return_alt - 2.0 else 0
                if config.LOCK_YAW:
                    # Let _enforce_search_yaw handle heading
                    self.nav.send_global_target(wp[0], wp[1], return_alt, vz=vz)
                else:
                    # Yaw toward waypoint — don't fly sideways/backwards
                    dy = (wp[0] - self.lat) * 111320
                    dx = (wp[1] - self.lon) * 111320 * math.cos(math.radians(self.lat))
                    wp_yaw = math.atan2(dx, dy)
                    self.nav.send_global_target(wp[0], wp[1], return_alt, yaw=wp_yaw, vz=vz)
                self.last_req = time.time()
                if self.alt < return_alt - 5.0:
                    print(f"  [CLIMB] {self.alt:.1f}m → {return_alt:.0f}m  vz={vz}")
            if self.get_dist_to_point(wp[0], wp[1]) < 2.0:
                print(f"Return transit WP {len(self.pre_waypoints) - self.return_wp_index}/{len(self.pre_waypoints)} reached.")
                self.return_wp_index -= 1
        else:
            print("Transit path retraced. Returning to launch point.")
            self._set_state(State.RETURN_HOME)

    def _handle_return_home(self, target_found, px_u, px_v, key):
        # Guard: if GPS never fixed, home_lat/lon may be config defaults rather
        # than the actual takeoff position.  Land in place instead of flying to
        # a potentially wrong location.
        if not getattr(self, 'gps_fix_ok', False):
            print("WARNING: GPS never fixed — home position unknown. Landing in place.")
            self._set_state(State.LANDING)
            return
        self.nav.set_speed(config.TRANSIT_SPEED_MPS)
        if time.time() - self.last_req > 0.5:
            vz = -2.5 if self.alt < config.TARGET_ALT - 2.0 else 0
            if config.LOCK_YAW:
                self.nav.send_global_target(self.home_lat, self.home_lon, config.TARGET_ALT, vz=vz)
            else:
                # Yaw toward home — don't fly sideways/backwards
                dy = (self.home_lat - self.lat) * 111320
                dx = (self.home_lon - self.lon) * 111320 * math.cos(math.radians(self.lat))
                home_yaw = math.atan2(dx, dy)
                self.nav.send_global_target(self.home_lat, self.home_lon, config.TARGET_ALT,
                                            yaw=home_yaw, vz=vz)
            self.last_req = time.time()
            if self.alt < config.TARGET_ALT - 5.0:
                print(f"  [CLIMB] {self.alt:.1f}m → {config.TARGET_ALT:.0f}m  vz={vz}")
        if self.get_dist_to_point(self.home_lat, self.home_lon) < 2.0:
            print("Home reached. Landing.")
            self._set_state(State.LANDING)

    def _handle_landing(self, target_found, px_u, px_v, key):
        from pymavlink import mavutil

        TOUCHDOWN_ALT = 0.5          # m, baro drifts ~0.2 so 0.3 was too tight
        TOUCHDOWN_TICKS = 5
        LANDING_TIMEOUT_S = 90.0     # FIXME: maybe expose in config

        # ArduPilot auto-disarms on touchdown (accelerometer-based).
        # if motors are already disarmed, we're on the ground.
        if self._land_cmd_sent and not self.master.motors_armed():
            self._finish_landing("ArduPilot auto-disarmed — touchdown confirmed.")
            return

        if self.alt < TOUCHDOWN_ALT:
            self._touchdown_count = getattr(self, '_touchdown_count', 0) + 1
        else:
            self._touchdown_count = 0

        if self._touchdown_count >= TOUCHDOWN_TICKS:
            print("Touchdown (alt stable below threshold). Disarming.")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
            self._finish_landing("Touchdown detected via altimeter.")
            return

        # send LAND once
        if not getattr(self, '_land_cmd_sent', False):
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0,
                self.home_lat, self.home_lon, 0)
            self._land_cmd_sent = True
            self._land_cmd_time = time.time()
            self._land_retries = 0
            print("  MAV_CMD_NAV_LAND sent")
            return

        # absolute timeout — covers baro-drift deadlock
        total_elapsed = time.time() - self._land_cmd_time
        if total_elapsed > LANDING_TIMEOUT_S:
            print(f"WARNING: Landing timeout ({LANDING_TIMEOUT_S}s). Force disarming.")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
            self._finish_landing(f"Forced disarm after {LANDING_TIMEOUT_S}s timeout.")
            return

        retry_elapsed = time.time() - getattr(self, '_last_land_retry', self._land_cmd_time)
        if retry_elapsed > 5.0 and self.alt > 1.0:
            self._land_retries = getattr(self, '_land_retries', 0) + 1
            if self._land_retries >= 5:
                print("WARNING: LAND failed after 5 retries. Force disarming.")
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
                self._finish_landing("Forced disarm after 5 LAND retries.")
                return
            print(f"  LAND not descending — retrying ({self._land_retries}/5)")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0,
                self.home_lat, self.home_lon, 0)
            self._last_land_retry = time.time()

    def _finish_landing(self, reason):
        # compute landing distances, log, go to DONE
        s = 111132.0
        self.final_dist = math.sqrt(((self.lat - self.home_lat) * s) ** 2 +
                                    ((self.lon - self.home_lon) * s * math.cos(math.radians(self.lat))) ** 2)
        elapsed = time.time() - getattr(self, '_mission_start_time', time.time())
        mins, secs = int(elapsed // 60), int(elapsed % 60)
        print(f"\n{'='*60}")
        print(f"  MISSION COMPLETE  [T+{mins:02d}:{secs:02d}]")
        print(f"  {reason}")
        print(f"  Landing error: {self.final_dist:.2f}m from home")
        if self.target_lat != 0 and self.target_lon != 0:
            target_dist = math.sqrt(((self.lat - self.target_lat) * s) ** 2 +
                                    ((self.lon - self.target_lon) * s * math.cos(math.radians(self.lat))) ** 2)
            print(f"  Distance from casualty: {target_dist:.2f}m (R07 target: 5-10m)")
            self.casualty_dist = target_dist
        else:
            self.casualty_dist = -1
        print(f"{'='*60}")
        self._set_state(State.DONE)

    # -- PLB beacon redirect --

    def _trigger_beacon_redirect(self):
        # fakes a PLB hit and redirects search to the Focus Area polygon
        if getattr(self, '_beacon_triggered', False):
            return
        import json, os
        fa_path = "flight_plans/focus_area.json"
        if os.path.exists(fa_path):
            try:
                with open(fa_path, encoding='utf-8') as f:
                    data = json.load(f)
                if not isinstance(data, list):
                    raise ValueError(f"Expected a JSON list, got {type(data).__name__}")
                loaded = []
                for i, wp in enumerate(data):
                    if isinstance(wp, dict):
                        lat, lon = float(wp["lat"]), float(wp["lon"])
                    else:
                        lat, lon = float(wp[0]), float(wp[1])
                    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                        raise ValueError(f"Point {i} out of range: ({lat}, {lon})")
                    loaded.append((lat, lon))
                if len(loaded) < 3:
                    raise ValueError(f"Need >= 3 points, got {len(loaded)}")
                config.FOCUS_AREA_GPS = loaded
                print(f"[PLB] Loaded focus_area.json: {len(loaded)} points")
            except Exception as e:
                print(f"[PLB] Failed to read {fa_path}: {e} — using default Focus Area")

        if not config.FOCUS_AREA_GPS or len(config.FOCUS_AREA_GPS) < 3:
            print("[PLB] No Focus Area defined — ignoring beacon")
            return

        self._beacon_triggered = True
        self._search_yaw_done = False
        self._search_yaw_sent = False
        print(f"\n[PLB] BEACON SIGNAL RECEIVED! Redirecting to Focus Area ({len(config.FOCUS_AREA_GPS)} pts)\n")

        focus_poly_px = []
        for lat, lon in config.FOCUS_AREA_GPS:
            focus_poly_px.append(self.geo.gps_to_pixels(lat, lon))
        self.planner.search_polygon = focus_poly_px
        self.planner._focus_area = True
        self.search_poly = focus_poly_px

        if config.MODE == "SIMULATION":
            canvas_w, canvas_h = self.sim.map_w, self.sim.map_h
        else:
            canvas_w, canvas_h = config.REAL_CANVAS_SIZE, config.REAL_CANVAS_SIZE

        self.waypoints = self.planner.generate_search_pattern(canvas_w, canvas_h, (self.lat, self.lon))
        self.wp_index = 0
        self.rescan_pass = 0
        print(f"  New search pattern: {len(self.waypoints)} waypoints at current altitude")

    # -- Key input handling --

    def _handle_manual_toggle(self, target_found, px_u, px_v):
        # M key: flip between MANUAL and previous auto state
        if self.state != State.MANUAL:
            print("!!! MANUAL CONTROL OVERRIDE !!! WASD=move R/F=up/down Q/E=yaw M=resume")
            if self.state != State.RETURN_FROM_MANUAL:
                self.previous_state = self.state
                self.manual_departure_lat = self.lat
                self.manual_departure_lon = self.lon
                self.manual_departure_alt = self.alt
            self._set_state(State.MANUAL)
            if self.nav:
                self.nav.send_velocity(0, 0, 0)
        else:
            if target_found:
                self.calculate_target_gps(px_u, px_v)
                if not self._is_inside_nfz(self.target_lat, self.target_lon) and \
                   not self._is_outside_search_area(self.target_lat, self.target_lon) and \
                   not self._is_near_known(self.target_lat, self.target_lon):
                    print("Target detected during manual flight — investigating!")
                    self._locked_target = (self.target_lat, self.target_lon)
                    self._set_state(State.CENTERING)
                    return
                else:
                    self.target_lat = self.target_lon = 0
                    target_found = False
            if not target_found:
                result = self._pop_valid_target()
                if result:
                    q_lat, q_lon, _qc = result
                    print(f"Investigating queued detection at ({q_lat:.6f}, {q_lon:.6f}) — {len(self._detect_queue)} remaining")
                    self.target_lat, self.target_lon = q_lat, q_lon
                    self._locked_target = (q_lat, q_lon)
                    self._set_state(State.CENTERING)
                else:
                    dist = self.get_dist_to_point(self.manual_departure_lat, self.manual_departure_lon)
                    if dist > 5.0:
                        print(f"Returning to manual departure point ({dist:.0f}m away)...")
                        self._set_state(State.RETURN_FROM_MANUAL)
                    else:
                        print("Resuming Automation...")
                        self.last_req = 0
                        self._set_state(self.previous_state)

    def _handle_manual_flight(self, key):
        """WASD/RF velocity + QE yaw in MANUAL. NFZ viscous field clamps the
        component pointing into the NFZ only (tangential motion still allowed)."""
        import math
        spd = config.MANUAL_FLY_SPEED_MPS
        climb  =  config.MANUAL_CLIMB_RATE_MPS
        yaw_rate = config.MANUAL_YAW_RATE_DEGS
        k = chr(key).lower() if key else ''
        vf, vr, vd = 0, 0, 0  # forward, right, down
        if k == 'w':   vf = spd
        elif k == 's': vf = -spd
        elif k == 'a': vr = -spd
        elif k == 'd': vr = spd
        elif k == 'r': vd = -climb
        elif k == 'f': vd = climb
        elif k in ('q', 'e'):
            from pymavlink import mavutil
            direction = -1 if k == 'q' else 1
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                config.MANUAL_YAW_STEP_DEG, yaw_rate, direction, 1, 0, 0, 0)
            return
        # Apply NFZ viscous field to horizontal velocity
        nfz_toward = getattr(self, '_nfz_toward_vec', None)  # (toward_n, toward_e)
        nfz_max = getattr(self, '_nfz_manual_max_speed', None)
        if nfz_toward is not None and nfz_max is not None and (vf != 0 or vr != 0):
            toward_n, toward_e = nfz_toward
            # Convert body-frame velocity to NED using yaw
            yaw = getattr(self, 'yaw', 0)  # already in radians from MAVLink ATTITUDE
            vn = vf * math.cos(yaw) - vr * math.sin(yaw)
            ve = vf * math.sin(yaw) + vr * math.cos(yaw)
            # Approach component toward NFZ
            v_toward = vn * toward_n + ve * toward_e
            if v_toward > nfz_max:
                # Clamp approach component, keep tangential
                reduction = v_toward - nfz_max
                vn -= toward_n * reduction
                ve -= toward_e * reduction
            # Send clamped velocity directly in NED (bypass body rotation)
            self.nav.send_velocity(vn, ve, vd, current_yaw=0)
            return
        # No NFZ limit — send normally in body frame
        if vf != 0 or vr != 0 or vd != 0:
            self.nav.send_velocity(vf, vr, vd)
        elif k in ('q', 'e'):
            from pymavlink import mavutil
            direction = -1 if k == 'q' else 1
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                config.MANUAL_YAW_STEP_DEG, yaw_rate, direction, 1, 0, 0, 0)

    def _handle_verify_input(self, key):
        # Y/N/I/X and landing-side (N/E/S/W) in VERIFY
        if self.selecting_landing_side:
            if key in [ord('n'), ord('e'), ord('w'), ord('s'), ord('N'), ord('E'), ord('W'), ord('S')]:
                self.calculate_landing_spot(chr(key).lower())
                self.selecting_landing_side = False
                self.waiting_for_confirmation = False
                self._set_state(State.APPROACH)
            return

        if key == ord('y') or key == ord('Y'):
            self._save_detection_image("Y", self.target_lat, self.target_lon, self.current_conf)
            add_detection_event({"time": datetime.now().strftime("%H:%M:%S"),
                "lat": self.target_lat, "lon": self.target_lon,
                "conf": self.current_conf, "decision": "Y"})
            if getattr(self, '_gps_avg_start', None):
                elapsed = time.time() - self._gps_avg_start
                remaining = max(0, 10.0 - elapsed)
                self._confirmed_y = True
                if remaining > 0:
                    print(f"  Y confirmed — averaging GPS for {remaining:.0f}s more...")
            else:
                print("USER CONFIRMED TARGET. SELECT LANDING SIDE: N/E/S/W")
                self.selecting_landing_side = True
        elif key == ord('i') or key == ord('I'):
            self._save_detection_image("I", self.target_lat, self.target_lon, self.current_conf)
            add_detection_event({"time": datetime.now().strftime("%H:%M:%S"),
                "lat": self.target_lat, "lon": self.target_lon,
                "conf": self.current_conf, "decision": "I"})
            if not hasattr(self, 'items_of_interest'):
                self.items_of_interest = []
            self.items_of_interest.append({
                'lat': self.target_lat, 'lon': self.target_lon,
                'alt': self.alt, 'time': time.time(),
            })
            print(f"  ITEM OF INTEREST #{len(self.items_of_interest)} at ({self.target_lat:.6f}, {self.target_lon:.6f})")
            self.waiting_for_confirmation = False
            self.target_lat = self.target_lon = 0
            self.last_req = 0
            result = self._pop_valid_target()
            if result:
                q_lat, q_lon, _qc = result
                print(f"Next queued target at ({q_lat:.6f}, {q_lon:.6f}) — {len(self._detect_queue)} remaining")
                self.target_lat, self.target_lon = q_lat, q_lon
                self._locked_target = (q_lat, q_lon)
                self._set_state(State.CENTERING)
            elif self.departure_lat != 0:
                self._set_state(State.RETURN_TO_SEARCH)
            else:
                self._set_state(State.SEARCH)
        elif key == ord('n') or key == ord('N'):
            self._save_detection_image("N", self.target_lat, self.target_lon, self.current_conf)
            add_detection_event({"time": datetime.now().strftime("%H:%M:%S"),
                "lat": self.target_lat, "lon": self.target_lon,
                "conf": self.current_conf, "decision": "N"})
            self.rejected_targets.append((self.target_lat, self.target_lon))
            print(f"USER REJECTED TARGET at ({self.target_lat:.6f}, {self.target_lon:.6f}). RESUMING.")
            self.waiting_for_confirmation = False
            self.target_lat = self.target_lon = 0
            self.last_req = 0
            result = self._pop_valid_target()
            if result:
                q_lat, q_lon, _qc = result
                print(f"Next queued target at ({q_lat:.6f}, {q_lon:.6f}) — {len(self._detect_queue)} remaining")
                self.target_lat, self.target_lon = q_lat, q_lon
                self._locked_target = (q_lat, q_lon)
                self._set_state(State.CENTERING)
            elif self.manual_departure_lat != 0 and self.previous_state == State.MANUAL:
                print("  Returning to manual departure point")
                self._set_state(State.RETURN_FROM_MANUAL)
            elif self.departure_lat != 0:
                print("  Returning to search departure point")
                self._set_state(State.RETURN_TO_SEARCH)
            else:
                self._set_state(State.SEARCH)
        elif key == ord('x') or key == ord('X'):
            self._save_detection_image("X", self.target_lat, self.target_lon, self.current_conf)
            add_detection_event({"time": datetime.now().strftime("%H:%M:%S"),
                "lat": self.target_lat, "lon": self.target_lon,
                "conf": self.current_conf, "decision": "X"})
            print(f"[VERIFY] FALSE POSITIVE — target rejected as false positive")
            self.rejected_targets.append((self.target_lat, self.target_lon))
            self.waiting_for_confirmation = False
            self.target_lat = self.target_lon = 0
            self.last_req = 0
            result = self._pop_valid_target()
            if result:
                q_lat, q_lon, _qc = result
                print(f"Next queued target at ({q_lat:.6f}, {q_lon:.6f}) — {len(self._detect_queue)} remaining")
                self.target_lat, self.target_lon = q_lat, q_lon
                self._locked_target = (q_lat, q_lon)
                self._set_state(State.CENTERING)
            elif self.manual_departure_lat != 0 and self.previous_state == State.MANUAL:
                print("  Returning to manual departure point")
                self._set_state(State.RETURN_FROM_MANUAL)
            elif self.departure_lat != 0:
                print("  Returning to search departure point")
                self._set_state(State.RETURN_TO_SEARCH)
            else:
                self._set_state(State.SEARCH)

    def _handle_keys(self, key, target_found, px_u, px_v):
        # dispatch keyboard / browser button input
        if key == ord('m') or key == ord('M'):
            self._handle_manual_toggle(target_found, px_u, px_v)
        if key == ord('l') or key == ord('L'):
            print("[OPERATOR] RTL requested")
            from pymavlink import mavutil
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, 0, 0, 0, 0, 0)  # 6 = RTL
            self._set_state(State.LANDING)
        if key == ord('o') or key == ord('O'):
            # emergency kill
            print("[OPERATOR] KILL/DISARM requested")
            try:
                if self.master:
                    self.master.arducopter_disarm()
                    print("[KILL] Disarm command sent")
            except Exception as e:
                print(f"[KILL] Disarm failed: {e}")
        if key == ord('k') or key == ord('K'):
            # K = clear rejected targets + items of interest (original behavior)
            n_rej = len(self.rejected_targets)
            n_ioi = len(getattr(self, 'items_of_interest', []))
            self.rejected_targets.clear()
            if hasattr(self, 'items_of_interest'):
                self.items_of_interest.clear()
            print(f"[RESET] Cleared {n_rej} rejected + {n_ioi} IOI. All areas re-enabled.")
        if (key == ord('b') or key == ord('B')) and self.state == State.SEARCH:
            self._trigger_beacon_redirect()
        # Queue detections during MANUAL mode
        if self.state == State.MANUAL and target_found:
            self.calculate_target_gps(px_u, px_v)
            if not self._is_inside_nfz(self.target_lat, self.target_lon) and \
               not self._is_outside_search_area(self.target_lat, self.target_lon) and \
               not self._is_near_known(self.target_lat, self.target_lon):
                conf = getattr(self, 'current_conf', 0.5)
                if not self._is_near_known(self.target_lat, self.target_lon):
                    self._enqueue_detection(self.target_lat, self.target_lon, conf)
                    print(f"[MANUAL] Detection queued at ({self.target_lat:.6f}, {self.target_lon:.6f}) conf={conf:.2f}")
            self.target_lat = self.target_lon = 0
        if self.state == State.MANUAL and self.master:
            self._handle_manual_flight(key)
        if self.state == State.VERIFY:
            self._handle_verify_input(key)
