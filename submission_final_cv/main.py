# main.py — mission orchestrator. state machine + telemetry + HUD.
# see states.py for the state enum and state_machine.py for the handlers.
import time, math, cv2, csv, sys, os, threading, random
from pymavlink import mavutil
import numpy as np
from datetime import datetime

try:
    import config
    if not hasattr(config, 'MODE'):
        print("[ERROR] Wrong 'config' module! Rename local config.py to avoid conflict.")
        sys.exit(1)
except ImportError:
    print("[ERROR] Could not import 'config.py'.")
    sys.exit(1)

from states import State
from utils import GeoTransformer
from planning import PathPlanner
from vision import VisionSystem
from state_machine import StateHandlersMixin
from navigation import NavigationController
from stream_server import (start_stream_server, set_stream_frame,
                           get_stream_frame, set_telemetry,
                           add_detection_event,
                           cmd_queue as stream_cmd_queue)
from gps_utils import calculate_target_from_pixels, landing_offset_7_5m

if config.MODE == "SIMULATION":
    from simulator.simulation import SimulationEnvironment

# ── CLI flags & config ────────────────────────────────────────────────
DRY_RUN = "--dry-run" in sys.argv
MODEL_PATH = "best.tflite"
TRANSIT_FILE = "flight_plans/transit.json"
_TRANSIT_EXPLICIT = False
STREAM_ENABLED = "--no-stream" not in sys.argv
STREAM_PORT = 8090
STREAM_W, STREAM_H = 640, 480          # default stream resolution (not full frame)
STREAM_FPS = 10
STREAM_QUALITY = 50                    # WiFi-friendly default
CENTER_VERIFY = "--center-verify" in sys.argv
SMART_DETECT = "--smart-detect" in sys.argv
NO_NFZ = "--no-nfz" in sys.argv
NFZ_DIRECTIONAL = "--nfz-total-speed" not in sys.argv  # directional is default
USE_SPIRAL = "--spiral" in sys.argv  # Zian's perimeter spiral instead of lawnmower
config.LOCK_YAW = "--lock-yaw" in sys.argv  # Maintain search yaw throughout sweep
CLEAN_DETECTIONS = "--clean" in sys.argv  # Wipe mission_detections/ at start
VERBOSE_GPS = "--verbose-gps" in sys.argv  # Print detailed GPS estimation math for every detection
SHAKE_PX=0  # --shake <pixels>: random pixel offset per frame (simulates vibration)
SIM_PITCH = "--sim-pitch" in sys.argv  # simulate camera pitch offset during forward flight
SIM_ROLL_DEG = 0.0  # --sim-roll <deg>: random roll oscillation (±degrees)
SIM_TILT = "--sim-tilt" in sys.argv  # combined: camera tilts in flight direction (replaces separate pitch/roll)
COMPENSATE_TILT = "--compensate-tilt" in sys.argv  # correct GPS estimation for camera tilt (use in REAL mode)
BLUR_FACTOR = 0.0  # --blur <factor>: motion blur proportional to speed (1.0=realistic, 2.0=stress)
NOISE_LEVEL = 0.0  # --noise <level>: GPS/attitude noise (1.0=realistic ~2-3m CEP, 2.0=stress test)

for _i, _arg in enumerate(sys.argv):
    if _arg == "--model" and _i + 1 < len(sys.argv):        MODEL_PATH = sys.argv[_i + 1]
    elif _arg == "--transit" and _i + 1 < len(sys.argv):     TRANSIT_FILE = sys.argv[_i + 1]; _TRANSIT_EXPLICIT = True
    elif _arg == "--speed" and _i + 1 < len(sys.argv):       config.SIM_SPEED = float(sys.argv[_i + 1])
    elif _arg == "--alt" and _i + 1 < len(sys.argv):         config.TARGET_ALT = float(sys.argv[_i + 1])
    elif _arg == "--beacon-delay" and _i + 1 < len(sys.argv): config.BEACON_DELAY = float(sys.argv[_i + 1])
    elif _arg == "--conf" and _i + 1 < len(sys.argv):         config.CONFIDENCE_THRESHOLD = float(sys.argv[_i + 1])
    elif _arg == "--shake" and _i + 1 < len(sys.argv):        SHAKE_PX = int(sys.argv[_i + 1])
    elif _arg == "--sim-roll" and _i + 1 < len(sys.argv):    SIM_ROLL_DEG = float(sys.argv[_i + 1])
    elif _arg == "--blur" and _i + 1 < len(sys.argv):         BLUR_FACTOR = float(sys.argv[_i + 1])
    elif _arg == "--noise" and _i + 1 < len(sys.argv):        NOISE_LEVEL = float(sys.argv[_i + 1])
    elif _arg == "--stream-scale" and _i + 1 < len(sys.argv):
        _sc = float(sys.argv[_i + 1])
        STREAM_W = int(config.IMAGE_W * _sc)
        STREAM_H = int(config.IMAGE_H * _sc)
    elif _arg == "--stream-quality" and _i + 1 < len(sys.argv):
        STREAM_QUALITY = int(sys.argv[_i + 1])
    elif _arg == "--stream-fps" and _i + 1 < len(sys.argv):
        STREAM_FPS = int(sys.argv[_i + 1])

if DRY_RUN:
    print("=" * 60)
    print("  DRY-RUN MODE — No arming, no flying, no GPS needed")
    print("=" * 60)

if USE_SPIRAL:
    print("[PATTERN] Spiral mode (Zian's perimeter planner)")


def _generate_spiral_waypoints(altitude_m, search_polygon_gps, drone_gps=None):
    # spiral waypoints for a GPS polygon. same spacing as lawnmower (footprint h, 1/3 edge margin).
    # tries Zian's planner first, falls back to our own if it blows up.
    import importlib.util as _ilu
    footprint_w = (config.SENSOR_WIDTH_MM * altitude_m) / config.FOCAL_LENGTH_MM
    footprint_h = footprint_w * config.IMAGE_H / config.IMAGE_W
    strip_spacing = footprint_h
    edge_margin = strip_spacing / 3.0

    # Try Zian's actual planner first (patches polygon via search_area module)
    try:
        _zian_dir = os.path.join(os.path.dirname(__file__), "Zian", "path_planner")
        spec_sa = _ilu.spec_from_file_location(
            "search_area", os.path.join(_zian_dir, "search_area.py"))
        _sa = _ilu.module_from_spec(spec_sa)
        spec_sa.loader.exec_module(_sa)

        spec_pp = _ilu.spec_from_file_location(
            "perimeter_planner", os.path.join(_zian_dir, "perimeter_planner.py"))
        _pp = _ilu.module_from_spec(spec_pp)
        spec_pp.loader.exec_module(_pp)

        # Patch polygon — override the imported search_area values
        import math as _m
        n = len(search_polygon_gps)
        c_lat = sum(p[0] for p in search_polygon_gps) / n
        c_lon = sum(p[1] for p in search_polygon_gps) / n
        _pp.CENTER_LAT = c_lat
        _pp.CENTER_LON = c_lon
        cos_lat = _m.cos(_m.radians(c_lat))
        _dlon_m = 111320.0 * cos_lat
        _dlat_m = 111320.0

        # Rebuild _POLY_XY from the new polygon
        def _ll2xy(lat, lon):
            return (lon - c_lon) * _dlon_m, (lat - c_lat) * _dlat_m
        _pp._ll2xy = _ll2xy
        _pp._xy2ll = lambda x, y: (c_lat + y / _dlat_m, c_lon + x / _dlon_m)
        _pp._POLY_XY = [_ll2xy(lat, lon) for lat, lon in search_polygon_gps]
        _pp.CORNERS = list(search_polygon_gps)
        _pp.N_VERTS = n
        _pp._dlon_m = _dlon_m
        _pp._dlat_m = _dlat_m

        # Patch spacing
        _pp.HALF_SWATH = edge_margin
        _pp.SWATH = strip_spacing
        if drone_gps:
            _pp.TAKEOFF_LAT = drone_gps[0]
            _pp.TAKEOFF_LON = drone_gps[1]
        _pp.ENTER_OFFSET_M = edge_margin

        wps = _pp.plan()
        result = [(wp["lat"], wp["lon"]) for wp in wps if wp["name"] != "enter"]
        if result:
            return result
    except Exception as e:
        print(f"[SPIRAL] Zian planner failed ({e}), using standalone fallback")

    # Fallback: standalone reimplementation (for edge cases)
    from spiral_planner import generate_spiral
    return generate_spiral(search_polygon_gps, strip_spacing, drone_gps, edge_margin)


HEADLESS = "--headless" in sys.argv
if not HEADLESS:
    try:
        if os.environ.get('DISPLAY', '') == '' and sys.platform != 'win32':
            HEADLESS = True
    except Exception:
        HEADLESS = True


def _terminal_input_thread():
    # read raw keypresses from the terminal (PuTTY/SSH) and dump them into stream_cmd_queue.
    import select
    if sys.platform == 'win32':
        import msvcrt
        while True:
            if msvcrt.kbhit():
                stream_cmd_queue.put(msvcrt.getch()[0])
            time.sleep(0.01)
    else:
        import tty, termios
        try:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
        except OSError:
            # running under nohup / no tty. fine, fall back to the browser buttons.
            print("[INPUT] No terminal — use browser buttons at http://PI_IP:8090")
            return
        try:
            tty.setcbreak(fd)
            while True:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    ch = sys.stdin.read(1)
                    if ch: stream_cmd_queue.put(ord(ch))
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Servo animation helper ───────────────────────────────────────────

def _draw_servo_animation(frame, elapsed):
    # payload drop animation drawn into the bottom-right corner during HOVER_TARGET
    # TODO: replace the magic 15s timeline with config.HOVER_TARGET_SECONDS

    h, w = frame.shape[:2]
    ax, ay, aw, ah = w - 270, h - 200, 250, 180
    ground_y = ay + ah - 25
    drone_x, drone_y = ax + aw // 2, ay + 35

    cv2.rectangle(frame, (ax, ay), (ax+aw, ay+ah), (180, 130, 80), -1)
    cv2.rectangle(frame, (ax, ground_y), (ax+aw, ay+ah), (50, 120, 50), -1)
    cv2.rectangle(frame, (ax, ay), (ax+aw, ay+ah), (255, 255, 255), 1)

    def draw_drone(dx, dy):
        cv2.rectangle(frame, (dx-25, dy-5), (dx+25, dy+5), (200, 200, 200), -1)
        cv2.line(frame, (dx-30, dy-8), (dx-15, dy-8), (180, 180, 180), 2)
        cv2.line(frame, (dx+15, dy-8), (dx+30, dy-8), (180, 180, 180), 2)

    def draw_pkg(px, py):
        cv2.rectangle(frame, (px-10, py), (px+10, py+14), (0, 0, 220), -1)

    if elapsed < 3.0:
        pkg_y = drone_y + 45
        draw_drone(drone_x, drone_y)
        cv2.line(frame, (drone_x, drone_y+5), (drone_x, pkg_y), (150, 150, 150), 2)
        draw_pkg(drone_x, pkg_y)
        label, color = f"HOVERING ({elapsed:.0f}s)", (255, 255, 255)
    elif elapsed < 4.0:
        progress = (elapsed - 3.0)
        pkg_y = int((drone_y + 45) + progress * (ground_y - (drone_y + 45) - 16))
        draw_drone(drone_x, drone_y)
        cv2.line(frame, (drone_x, drone_y+5), (drone_x, pkg_y), (150, 150, 150), 2)
        draw_pkg(drone_x, pkg_y)
        label, color = "STAGE 1 — DROP!", (0, 165, 255)
    elif elapsed < 6.0:
        draw_drone(drone_x, drone_y)
        cv2.line(frame, (drone_x, drone_y+5), (drone_x, ground_y-16), (150, 150, 150), 2)
        draw_pkg(drone_x, ground_y - 16)
        label, color = "STAGE 1 — DEPLOYED", (0, 165, 255)
    elif elapsed < 7.0:
        draw_drone(drone_x, drone_y)
        draw_pkg(drone_x, ground_y - 16)
        cv2.putText(frame, "OK", (drone_x+15, ground_y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 2)
        label, color = "STAGE 2 — DETACHED", (0, 0, 255)
    elif elapsed < 15.0:
        draw_drone(drone_x, drone_y)
        draw_pkg(drone_x, ground_y - 16)
        cv2.putText(frame, "OK", (drone_x+15, ground_y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 2)
        label, color = f"COMPLETE ({15-elapsed:.0f}s)", (0, 255, 0)
    else:
        fp = min(1.0, (elapsed - 15.0) / 2.0)
        draw_drone(int(drone_x + fp * 60), int(drone_y - fp * 30))
        draw_pkg(drone_x, ground_y - 16)
        cv2.putText(frame, "OK", (drone_x+15, ground_y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 2)
        label, color = "DEPARTING", (0, 255, 0)

    cv2.putText(frame, label, (ax+10, ay+18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    # Timeline bar
    bar_y = ay + ah - 10
    cv2.rectangle(frame, (ax+5, bar_y), (ax+aw-5, bar_y+6), (80, 80, 80), -1)
    prog = min(1.0, elapsed / 15.0)
    cv2.rectangle(frame, (ax+5, bar_y), (ax+5+int((aw-10)*prog), bar_y+5), (0, 255, 0), -1)


# ══════════════════════════════════════════════════════════════════════

class VisualFlightMission(StateHandlersMixin):
    def __init__(self):
        print(f"--- INITIALIZING IN {config.MODE} MODE ---")

        if not config.load_kml_zones():
            print("[WARN] KML load failed — using fallback GPS from config.py")

        # Simulation or Real setup
        if config.MODE == "SIMULATION":
            self.sim = SimulationEnvironment(GeoTransformer(map_w_px=100))
            self.geo = GeoTransformer(map_w_px=self.sim.map_w)
            self.sim.geo = self.geo
            preload_gps = config.SEARCH_AREA_GPS
            preload_transit = self._load_transit_from_file()
            self.target_px, self.tgt_type, self.search_poly, transit_px, focus_px = \
                self.sim.setup_on_map(preload_polygon_gps=preload_gps, preload_transit_gps=preload_transit)

            if focus_px and len(focus_px) >= 3:
                config.FOCUS_AREA_GPS = [self.geo.pixels_to_gps(px[0], px[1]) for px in focus_px]
                import json
                os.makedirs("flight_plans", exist_ok=True)
                with open("flight_plans/focus_area.json", "w") as f:
                    json.dump([{"lat": pt[0], "lon": pt[1], "label": f"F{i+1}"}
                               for i, pt in enumerate(config.FOCUS_AREA_GPS)], f, indent=2)
                print(f"  Focus Area: {len(config.FOCUS_AREA_GPS)} points saved")

            self._drawn_transit_gps = (
                [self.geo.pixels_to_gps(px[0], px[1]) for px in transit_px]
                if transit_px and not preload_transit else [])

            self.eyes = VisionSystem(camera_index=None, model_path=MODEL_PATH)
            self.eyes.using_ai = (self.tgt_type == "dummy") and self.eyes.model is not None
        else:
            self.sim = None
            self.search_poly = self._setup_real_search_area()
            cam_idx = None if DRY_RUN else config.REAL_CAMERA_INDEX
            self.eyes = VisionSystem(camera_index=cam_idx, model_path=MODEL_PATH)
            if self.eyes.model is not None:
                self.eyes.using_ai = True
            else:
                print("[WARN] AI model not loaded — detection disabled, mission will fly pattern only")

        # Geofence
        self.geofence = None
        if not NO_NFZ and config.SSSI_GPS and len(config.SSSI_GPS) >= 3:
            from geofence import NFZGeofence
            self.geofence = NFZGeofence(self.geo)
            print(f"[GEOFENCE] SSSI NFZ active — {len(config.SSSI_GPS)} corners, "
                  f"hard={self.geofence.HARD_BOUNDARY}m, soft={self.geofence.SOFT_BOUNDARY}m")
        if hasattr(config, 'FLIGHT_AREA_GPS') and len(config.FLIGHT_AREA_GPS) >= 3:
            print(f"[GEOFENCE] Flight Area geofence: {len(config.FLIGHT_AREA_GPS)} vertices (R01)")

        # Planner
        self.planner = PathPlanner(self.geo, self.search_poly)
        self.planner._no_turn = True
        if USE_SPIRAL:
            # Replace the planner's generate method with spiral waypoint generation.
            # state_machine.py calls self.planner.generate_search_pattern() everywhere —
            # by replacing the method here, ALL search pattern generation uses spiral
            # without any changes to state_machine.py. Same logic, different waypoints.
            _original_generate = self.planner.generate_search_pattern
            _planner_ref = self.planner
            _geo_ref = self.geo
            def _spiral_generate(map_w, map_h, drone_gps=None, alt_override=None,
                                 _orig=_original_generate, _pl=_planner_ref,
                                 _geo=_geo_ref):
                alt = alt_override or config.TARGET_ALT
                # Get current polygon as GPS (works for main area AND beacon area)
                poly_gps = [_geo.pixels_to_gps(pt[0], pt[1])
                            for pt in _pl.search_polygon]
                wps = _generate_spiral_waypoints(alt, poly_gps, drone_gps)
                if not wps:
                    # Fallback to lawnmower if spiral fails (e.g. polygon too small)
                    return _orig(map_w, map_h, drone_gps, alt_override)
                return wps
            self.planner.generate_search_pattern = _spiral_generate

        # Connection
        self.master = None
        self.last_req = 0
        self.last_heartbeat = 0
        self.nav = None

        # State & telemetry
        self.state = State.INIT
        self.previous_state = State.HOVER
        self._mission_start_time = time.time()
        self.state_start_time = time.time()
        self.connect_start_time = 0
        self.gps_fix_ok = False
        self.gps_fix_type = 0
        self.gps_satellites = 0
        self._gps_degraded_time = 0       # when fix first dropped below 3D
        self._gps_warn_printed = 0        # throttle warnings
        self._last_gps_status_print = 0
        self._last_mode_warn = -1
        self._cube_mode = 4  # Assume GUIDED until first heartbeat
        self._arming_timeout_warned = False
        self._takeoff_timeout_warned = False
        self._centering_timeout_warned = False
        self._descending_timeout_warned = False
        self._last_log_time = 0
        self._servo_released = False
        self._camera_none_count = 0
        self._last_camera_warn = 0

        if config.TAKEOFF_GPS:
            self.lat, self.lon = config.TAKEOFF_GPS[0], config.TAKEOFF_GPS[1]
        else:
            self.lat, self.lon = config.REF_LAT, config.REF_LON
        self.home_lat, self.home_lon = self.lat, self.lon
        self.return_wp_index = 0
        self.alt = 0.0
        self.vx = 0; self.vy = 0; self.vz = 0
        self.roll = 0; self.pitch = 0; self.yaw = 0
        self._prev_yaw = None        # for yaw-rate roll calculation (sim-tilt)
        self._prev_tilt_time = None   # timestamp for dt calculation (sim-tilt)
        self._last_tilt_log = 0       # throttle sim-tilt debug prints

        # Mission data
        if not hasattr(self, 'waypoints'): self.waypoints = []
        self.wp_index = 0
        self.target_lat = 0; self.target_lon = 0
        self.landing_lat = 0; self.landing_lon = 0
        self.current_conf = 0.0
        self.final_dist = 0.0
        self.rejected_targets = []
        self._detect_queue = []
        self.departure_lat = 0; self.departure_lon = 0
        self.manual_departure_lat = 0; self.manual_departure_lon = 0; self.manual_departure_alt = 0
        self.max_rescan_passes = config.MAX_RESCAN_PASSES
        self.rescan_pass = 0

        # Pre-planned waypoints (transit before search)
        self.pre_waypoints = []
        self.pre_wp_index = 0
        if TRANSIT_FILE and os.path.exists(TRANSIT_FILE):
            try:
                wps = self._load_waypoints_json(TRANSIT_FILE)
                self.pre_waypoints.extend(wps)
                print(f"  Transit: {len(wps)} points from {TRANSIT_FILE}")
            except Exception as e:
                print(f"WARNING: Failed to load {TRANSIT_FILE}: {e}")
        elif TRANSIT_FILE and _TRANSIT_EXPLICIT:
            print(f"ERROR: --transit file not found: {TRANSIT_FILE}")
            print("  Cannot proceed without explicitly requested transit path.")
            sys.exit(1)
        elif TRANSIT_FILE:
            print(f"  Transit: no file at {TRANSIT_FILE} — flying direct to search area")
        if hasattr(self, '_drawn_transit_gps') and self._drawn_transit_gps:
            self.pre_waypoints.extend(self._drawn_transit_gps)

        # Generate search waypoints
        if self.search_poly and len(self.search_poly) >= 3:
            cw = self.sim.map_w if config.MODE == "SIMULATION" else config.REAL_CANVAS_SIZE
            ch = self.sim.map_h if config.MODE == "SIMULATION" else config.REAL_CANVAS_SIZE
            start_ref = self.pre_waypoints[-1] if self.pre_waypoints else None
            self.waypoints = self.planner.generate_search_pattern(cw, ch, start_ref)
            print(f"  Search: {len(self.waypoints)} waypoints")

        # UI helpers
        self.view_w_px = 100; self.view_h_px = 100; self.zoom_level = 1.0
        self.waiting_for_confirmation = False
        self.selecting_landing_side = False

        # CSV logging
        os.makedirs(os.path.dirname(config.LOG_FILE) or ".", exist_ok=True)
        self.log_file = open(config.LOG_FILE, 'w', newline='')
        self.logger = csv.writer(self.log_file)
        self.logger.writerow(["Timestamp", "State", "Lat", "Lon", "Alt", "Target_Conf"])

    @staticmethod
    def _load_waypoints_json(path):
        # accepts both [{lat,lon},...] and [[lat,lon],...]
        import json
        with open(path) as f:
            data = json.load(f)
        return [(wp["lat"], wp["lon"]) if isinstance(wp, dict) else (wp[0], wp[1])
                for wp in data]

    def _load_transit_from_file(self):
        if not TRANSIT_FILE or not os.path.exists(TRANSIT_FILE):
            return None
        try:
            wps = self._load_waypoints_json(TRANSIT_FILE)
            print(f"  Transit path: {len(wps)} points from {TRANSIT_FILE}")
            return wps
        except Exception as e:
            print(f"WARNING: Failed to load transit from {TRANSIT_FILE}: {e}")
            return None

    def _setup_real_search_area(self):
        """Load the search polygon. Tries flight_plans/search_area.json first,
        then falls back to config.SEARCH_AREA_GPS (populated from the KML)."""
        self.geo = GeoTransformer(map_w_px=config.REAL_CANVAS_SIZE)
        sa_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_plans", "search_area.json")
        if os.path.exists(sa_file):
            import json
            with open(sa_file, "r") as f:
                data = json.load(f)
            if data and len(data) >= 3:
                gps_coords = [(pt["lat"], pt["lon"]) for pt in data]
                poly = [self.geo.gps_to_pixels(lat, lon) for lat, lon in gps_coords]
                print(f"[REAL] Search area: {len(poly)} points from search_area.json")
                return poly
        if hasattr(config, 'SEARCH_AREA_GPS') and len(config.SEARCH_AREA_GPS) >= 3:
            poly = [self.geo.gps_to_pixels(lat, lon) for lat, lon in config.SEARCH_AREA_GPS]
            print(f"[REAL] Search area: {len(poly)} points from config/KML")
            return poly
        print("[REAL] No search area — run draw_search_area.py or set SEARCH_AREA_GPS")
        return []

    # ── Telemetry ─────────────────────────────────────────────────────

    def update_telemetry(self):
        if not self.master: return
        while True:
            try:
                msg = self.master.recv_match(blocking=False)
            except (ConnectionResetError, ConnectionAbortedError,
                    BrokenPipeError, OSError) as e:
                print(f"\n[LINK LOST] recv_match failed: {e}")
                self._emergency_rtl(reason=str(e))
                self.state = State.DONE
                return
            if not msg: break
            mtype = msg.get_type()
            if mtype == 'GLOBAL_POSITION_INT':
                self.lat = msg.lat / 1e7; self.lon = msg.lon / 1e7
                self.alt = msg.relative_alt / 1000.0
                self.vx = msg.vx / 100.0; self.vy = msg.vy / 100.0; self.vz = msg.vz / 100.0
            elif mtype == 'ATTITUDE':
                self.roll = msg.roll; self.pitch = msg.pitch; self.yaw = msg.yaw
            elif mtype == 'HEARTBEAT' and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.last_heartbeat = time.time()
                if self.master.target_system == 0:
                    self.master.target_system = msg.get_srcSystem()
                    self.master.target_component = msg.get_srcComponent()
                    print(f"[LINK] Autopilot found: system {self.master.target_system}")
                self._cube_mode = getattr(msg, 'custom_mode', 4)
                if hasattr(msg, 'custom_mode') and self.state not in (
                        State.MANUAL, State.DONE, State.INIT, State.CONNECTING):
                    if msg.custom_mode != 4 and self._last_mode_warn != msg.custom_mode:
                        modes = {0:'STABILIZE',2:'ALT_HOLD',3:'AUTO',4:'GUIDED',
                                 5:'LOITER',6:'RTL',9:'LAND',16:'POSHOLD'}
                        print(f"WARNING: Cube in {modes.get(msg.custom_mode, f'MODE_{msg.custom_mode}')} — RC OVERRIDE ACTIVE, commands paused")
                        self._last_mode_warn = msg.custom_mode
            elif mtype == 'GPS_RAW_INT':
                self.gps_fix_type = msg.fix_type
                self.gps_satellites = msg.satellites_visible
                self._check_gps_degradation()

        # ── Inject realistic noise in simulation ─────────────────────
        if NOISE_LEVEL > 0 and config.MODE == "SIMULATION":
            # GPS noise: ~2-3m CEP at level 1.0
            self.lat += random.gauss(0, 0.000025 * NOISE_LEVEL)        # ~2.8m std
            self.lon += random.gauss(0, 0.000025 * NOISE_LEVEL / max(math.cos(math.radians(self.lat)), 0.01))
            # Altitude noise: ~0.5m std at level 1.0
            self.alt += random.gauss(0, 0.5 * NOISE_LEVEL)
            # Attitude noise: ~0.5° std at level 1.0
            self.pitch += random.gauss(0, math.radians(0.5 * NOISE_LEVEL))
            self.roll  += random.gauss(0, math.radians(0.5 * NOISE_LEVEL))
            self.yaw   += random.gauss(0, math.radians(1.0 * NOISE_LEVEL))

    GPS_DEGRADE_RTL_SECONDS = 5  # RTL after this many seconds of degraded GPS

    def _check_gps_degradation(self):
        # warn + RTL if GPS fix drops for too long mid-flight.
        # only runs during active flight states — no point nagging on the ground.
        if self.state in (State.INIT, State.CONNECTING, State.ARMING, State.DONE,
                          State.MANUAL, State.LANDING):
            return

        now = time.time()
        fix_ok = self.gps_fix_type >= 3 and self.gps_satellites >= 6

        if fix_ok:
            # fix came back, reset the timer
            if self._gps_degraded_time > 0:
                elapsed = now - self._gps_degraded_time
                print(f"[GPS] Fix recovered (fix={self.gps_fix_type}, sats={self.gps_satellites}) "
                      f"after {elapsed:.1f}s degraded")
                self._gps_degraded_time = 0
            return

        if self._gps_degraded_time == 0:
            self._gps_degraded_time = now

        # throttle the warning to ~1 per 3s, otherwise the console drowns
        if now - self._gps_warn_printed >= 3.0:
            elapsed = now - self._gps_degraded_time
            print(f"WARNING: GPS DEGRADED — fix={self.gps_fix_type}, sats={self.gps_satellites}, "
                  f"degraded for {elapsed:.1f}s (RTL in {max(0, self.GPS_DEGRADE_RTL_SECONDS - elapsed):.0f}s)")
            self._gps_warn_printed = now

        # RTL after sustained degradation
        elapsed = now - self._gps_degraded_time
        if elapsed >= self.GPS_DEGRADE_RTL_SECONDS:
            print(f"[GPS] FIX LOST for {elapsed:.1f}s — EMERGENCY RTL")
            self._emergency_rtl(reason=f"GPS fix lost ({self.gps_fix_type}, {self.gps_satellites} sats) for {elapsed:.1f}s")
            self._set_state(State.LANDING)

    # ── GPS math wrappers ─────────────────────────────────────────────

    def calculate_target_gps(self, u, v):
        # use actual frame size if we have it, otherwise fall back to config
        fw = getattr(self, '_frame_w', config.IMAGE_W)
        fh = getattr(self, '_frame_h', config.IMAGE_H)

        # Tilt compensation: when pitch/roll > ~1deg the nadir is not the
        # frame centre, so we ray-trace the detection pixel using the same
        # rotation matrix as the simulator. Works in both SIM and REAL.
        if (SIM_TILT or COMPENSATE_TILT) and self.alt > 1.0 and (abs(self.pitch) > 0.02 or abs(self.roll) > 0.02):
            focal_px = config.FOCAL_LENGTH_MM / config.SENSOR_WIDTH_MM * fw

            # Build SAME rotation as simulation: R = Rz(yaw) @ Ry(-pitch) @ Rx(-roll)
            def _Rx(a):
                c, s = math.cos(a), math.sin(a)
                return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)
            def _Ry(a):
                c, s = math.cos(a), math.sin(a)
                return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)
            def _Rz(a):
                c, s = math.cos(a), math.sin(a)
                return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)

            # a bit hacky but it works — we tried 8 different rotation
            # conventions before finding one that matched the sim. Kept them
            # all behind the env var so we can flip back fast on flight day
            # if the sim/real match breaks.
            # MUST MATCH simulator/simulation.py::_get_perspective_view R matrix.
            _variant = os.environ.get("SIM_TILT_VARIANT", "1")
            _R_c2b_ccw = np.array([
                [ 0.0, -1.0, 0.0],
                [ 1.0,  0.0, 0.0],
                [ 0.0,  0.0, 1.0],
            ], dtype=np.float64)
            _R_c2b_cw = np.array([
                [ 0.0,  1.0, 0.0],
                [-1.0,  0.0, 0.0],
                [ 0.0,  0.0, 1.0],
            ], dtype=np.float64)
            if _variant == "1":
                R = _Rz(self.yaw) @ _Ry(-self.pitch) @ _Rx(-self.roll) @ _R_c2b_ccw
            elif _variant == "2":
                R = _Rz(self.yaw) @ _Ry(self.pitch) @ _Rx(-self.roll) @ _R_c2b_ccw
            elif _variant == "3":
                R = _Rz(self.yaw) @ _Ry(-self.pitch) @ _Rx(self.roll) @ _R_c2b_ccw
            elif _variant == "4":
                R = _Rz(self.yaw) @ _Ry(self.pitch) @ _Rx(self.roll) @ _R_c2b_ccw
            elif _variant == "5":
                R = _Rz(-self.yaw) @ _Ry(-self.pitch) @ _Rx(-self.roll)
            elif _variant == "6":
                R = _Rz(-self.yaw) @ _Ry(-self.pitch) @ _Rx(-self.roll) @ _R_c2b_ccw
            elif _variant == "7":
                R = _Rz(self.yaw) @ _Ry(-self.pitch) @ _Rx(-self.roll) @ _R_c2b_cw
            else:
                R = _Rz(self.yaw) @ _Ry(-self.pitch) @ _Rx(-self.roll)

            # Ray from detection pixel in camera frame
            ray_cam = np.array([(u - fw / 2) / focal_px,
                                (v - fh / 2) / focal_px,
                                1.0])

            # Rotate to world frame (NED: X=North, Y=East, Z=Down)
            ray_world = R @ ray_cam

            if ray_world[2] > 0.01:
                t = self.alt / ray_world[2]
                north_m = ray_world[0] * t
                east_m = ray_world[1] * t

                # Convert to GPS directly (matches simulation ground-plane intersection)
                self.target_lat = self.lat + north_m / 111132.0
                self.target_lon = self.lon + east_m / (111132.0 * math.cos(math.radians(self.lat)))
                return  # Skip the normal GSD-based calculation

        self.target_lat, self.target_lon = calculate_target_from_pixels(
            u, v, self.alt, self.yaw, self.lat, self.lon,
            fw, fh, config.SENSOR_WIDTH_MM, config.FOCAL_LENGTH_MM,
            verbose=VERBOSE_GPS,
            drone_roll=self.roll, drone_pitch=self.pitch)

    def calculate_landing_spot(self, direction_key):
        self.landing_lat, self.landing_lon = landing_offset_7_5m(
            self.target_lat, self.target_lon, direction_key)

        # Check if landing spot is too close to NFZ — try other directions if so
        if self.geofence:
            dist, inside = self.geofence.distance_to_boundary(
                self.landing_lat, self.landing_lon)
            if inside or dist < 15.0:  # 15m buffer — descent/climb near NFZ is risky
                print(f"  [NFZ] Landing spot {direction_key.upper()} is {dist:.0f}m from NFZ — too close!")
                # Try all 4 directions, pick the one furthest from NFZ
                best_dir, best_dist = direction_key, dist
                for d in ['n', 's', 'e', 'w']:
                    lat, lon = landing_offset_7_5m(self.target_lat, self.target_lon, d)
                    dd, di = self.geofence.distance_to_boundary(lat, lon)
                    if not di and dd > best_dist:
                        best_dist = dd
                        best_dir = d
                        self.landing_lat, self.landing_lon = lat, lon
                if best_dir != direction_key:
                    print(f"  [NFZ] Redirected to {best_dir.upper()} ({best_dist:.0f}m from NFZ)")
                else:
                    print(f"  [NFZ] WARNING: all directions near NFZ — proceeding with caution")
            else:
                print(f"  Landing Spot: {direction_key.upper()} of target ({dist:.0f}m from NFZ — safe)")
        else:
            print(f"Landing Spot: {direction_key.upper()} of target")

    # ── Dashboard / HUD ───────────────────────────────────────────────

    def update_dashboard(self):
        found = False; u = 0; v = 0; conf = 0.0

        # Get frame
        if config.MODE == "SIMULATION":
            px, py = self.geo.gps_to_pixels(self.lat, self.lon)
            spd = math.sqrt(self.vx**2 + self.vy**2)

            # --sim-tilt / --sim-pitch: pass actual SITL pitch/roll to
            # get_drone_view() for ray-traced perspective warp (no black edges).
            _sim_pitch = 0.0
            _sim_roll = 0.0
            if (SIM_TILT or SIM_PITCH) and self.alt > 1.0:
                _sim_pitch = self.pitch
                _sim_roll = self.roll

            frame, self.view_w_px, self.view_h_px = self.sim.get_drone_view(
                int(px), int(py), self.alt, self.yaw,
                pitch=_sim_pitch, roll=_sim_roll)

            # --sim-roll: random roll oscillation (cosmetic frame rotation only)
            if SIM_ROLL_DEG > 0:
                roll_angle = random.uniform(-SIM_ROLL_DEG, SIM_ROLL_DEG)
                h, w = frame.shape[:2]
                M_roll = cv2.getRotationMatrix2D((w // 2, h // 2), roll_angle, 1.0)
                frame = cv2.warpAffine(frame, M_roll, (w, h))
            if SHAKE_PX > 0:
                dx = random.randint(-SHAKE_PX, SHAKE_PX)
                dy = random.randint(-SHAKE_PX, SHAKE_PX)
                M = np.float32([[1, 0, dx], [0, 1, dy]])
                frame = cv2.warpAffine(frame, M, (frame.shape[1], frame.shape[0]))
            # Simulate motion blur from ALL pixel-motion sources
            # Blur = how many pixels a ground point moves during exposure
            if BLUR_FACTOR > 0:
                fw = getattr(self, '_frame_w', config.IMAGE_W)
                exposure_time = 0.005  # 5ms typical exposure
                gsd = (config.SENSOR_WIDTH_MM * max(self.alt, 1.0)) / (config.FOCAL_LENGTH_MM * fw)

                # 1) Translation blur: ground speed → pixel motion
                trans_px = (spd * exposure_time) / max(gsd, 0.001)

                # 2) Rotation blur: yaw rate → pixel motion at frame edge
                rot_px = 0.0
                if hasattr(self, '_prev_yaw_blur'):
                    dt = max(0.02, time.time() - self._prev_yaw_blur_time)
                    dyaw = abs(self.yaw - self._prev_yaw_blur)
                    if dyaw > math.pi:
                        dyaw = 2 * math.pi - dyaw
                    yaw_rate = dyaw / dt  # rad/s
                    focal_px = config.FOCAL_LENGTH_MM / config.SENSOR_WIDTH_MM * fw
                    rot_px = yaw_rate * focal_px * exposure_time
                self._prev_yaw_blur = self.yaw
                self._prev_yaw_blur_time = time.time()

                # Total pixel motion (RSS of translation + rotation)
                pixel_motion = math.sqrt(trans_px**2 + rot_px**2) * BLUR_FACTOR

                if pixel_motion > 1.0:
                    kernel_size = int(pixel_motion) | 1  # must be odd
                    kernel_size = min(kernel_size, 51)    # cap to avoid huge kernels
                    kernel = np.zeros((kernel_size, kernel_size))
                    angle = math.degrees(self.yaw)
                    mid = kernel_size // 2
                    for i in range(kernel_size):
                        x = int(mid + (i - mid) * math.cos(math.radians(angle)))
                        y = int(mid + (i - mid) * math.sin(math.radians(angle)))
                        if 0 <= x < kernel_size and 0 <= y < kernel_size:
                            kernel[y, x] = 1.0
                    kernel /= kernel.sum()
                    frame = cv2.filter2D(frame, -1, kernel)
            # Store actual frame dimensions for GPS estimation
            self._frame_w = frame.shape[1]
            self._frame_h = frame.shape[0]
        else:
            frame = self.eyes.get_frame()
            if frame is None:
                self._camera_none_count += 1
                now_cam = time.time()
                if now_cam - self._last_camera_warn > 5.0:
                    print(f"[WARN] Camera returned None "
                          f"({self._camera_none_count} consecutive frames) "
                          f"— flying blind!")
                    self._last_camera_warn = now_cam
                frame = np.zeros((config.IMAGE_H, config.IMAGE_W, 3),
                                 dtype=np.uint8)
                cv2.putText(frame, "CAMERA LOST",
                            (config.IMAGE_W // 2 - 150,
                             config.IMAGE_H // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5,
                            (0, 0, 255), 3)
            else:
                if self._camera_none_count > 0:
                    print(f"[INFO] Camera recovered after "
                          f"{self._camera_none_count} dropped frames")
                self._camera_none_count = 0
            # Store actual frame dimensions for GPS estimation
            self._frame_w = frame.shape[1]
            self._frame_h = frame.shape[0]

        found, u, v, conf = self.eyes.process_frame_manually(frame)
        self.current_conf = conf
        self.last_frame = frame.copy()

        # HUD overlay
        cx, cy = config.IMAGE_W // 2, config.IMAGE_H // 2
        cv2.line(frame, (cx-20, cy), (cx+20, cy), (0,255,255), 2)
        cv2.line(frame, (cx, cy-20), (cx, cy+20), (0,255,255), 2)
        if found:
            cv2.circle(frame, (u,v), 15, (0,255,0), 2)
            cv2.line(frame, (u,v), (cx,cy), (0,255,0), 2)
            cv2.putText(frame, f"TGT {conf:.2f}", (u+10,v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # Info panel
        F = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"MODE: {config.MODE}", (10,20), F, 0.6, (0,0,255), 2)
        cv2.putText(frame, f"STATE: {self.state}", (10,50), F, 0.6, (255,255,0), 2)
        cv2.putText(frame, f"ALT: {self.alt:.1f}m", (10,80), F, 0.6, (255,255,0), 2)
        spd = math.sqrt(self.vx**2 + self.vy**2)
        cv2.putText(frame, f"POS: {self.lat:.6f}, {self.lon:.6f}  SPD: {spd:.1f}m/s", (10,110), F, 0.6, (255,255,255), 2)
        if self.target_lat != 0:
            cv2.putText(frame, f"TARGET: {self.target_lat:.6f}, {self.target_lon:.6f}", (10,140), F, 0.6, (0,255,0), 2)
        if self.landing_lat != 0:
            cv2.putText(frame, f"LANDING: {self.landing_lat:.6f}, {self.landing_lon:.6f}", (10,170), F, 0.6, (0,255,255), 2)
        mname = os.path.basename(MODEL_PATH)
        cv2.putText(frame, f"MODEL: {mname}", (10,170 if self.landing_lat == 0 else 200), F, 0.6, (0,255,255), 2)

        # State-specific HUD
        if self.state == State.HOVER_TARGET:
            # Show distance from casualty during payload deploy
            if self.target_lat != 0:
                s = 111132.0
                cdist = math.sqrt(((self.lat - self.target_lat) * s) ** 2 +
                                  ((self.lon - self.target_lon) * s * math.cos(math.radians(self.lat))) ** 2)
                dcolor = (0,255,0) if 5 <= cdist <= 10 else (0,165,255)
                cv2.putText(frame, f"DIST FROM CASUALTY: {cdist:.1f}m (R07: 5-10m)", (cx-280, cy-60), F, 0.8, dcolor, 2)
            elapsed = time.time() - self.state_start_time
            rem = max(0, 15.0 - elapsed)
            if elapsed < 3.0:
                cv2.putText(frame, f"HOVERING — waiting ({3.0-elapsed:.0f}s)", (cx-200, cy+60), F, 0.8, (0,255,255), 2)
            elif elapsed < 6.0:
                if int(elapsed*3)%2==0:
                    cv2.putText(frame, "PHASE 1 RELEASED", (cx-180, cy-30), F, 1.2, (0,165,255), 3)
                cv2.putText(frame, f"Phase 2 in {6.0-elapsed:.0f}s", (cx-100, cy+60), F, 0.7, (255,255,255), 2)
            elif elapsed < 15.0:
                if int(elapsed*3)%2==0:
                    cv2.putText(frame, "PHASE 2 RELEASED", (cx-180, cy-30), F, 1.2, (0,0,255), 3)
                cv2.putText(frame, f"Departing in {15.0-elapsed:.0f}s", (cx-120, cy+60), F, 0.7, (0,255,255), 2)

        if self.state == State.MANUAL:
            cv2.putText(frame, "MANUAL OVERRIDE", (cx-150, cy-30), F, 1.0, (0,165,255), 3)
            cv2.putText(frame, "WASD=fly R/F=alt Q/E=yaw M=resume", (cx-220, cy+20), F, 0.6, (0,165,255), 2)

        if self.state == State.DONE:
            cv2.putText(frame, "MISSION COMPLETE", (cx-180, cy-40), F, 1.2, (0,255,0), 3)
            cv2.putText(frame, f"LANDING ERROR FROM HOME: {self.final_dist:.2f} m", (cx-250, cy+10), F, 0.8, (0,0,255), 2)
            casualty_d = getattr(self, 'casualty_dist', -1)
            if casualty_d >= 0:
                color = (0,255,0) if 5 <= casualty_d <= 10 else (0,165,255)
                cv2.putText(frame, f"DISTANCE FROM CASUALTY: {casualty_d:.2f} m (R07: 5-10m)", (cx-300, cy+50), F, 0.8, color, 2)
            elapsed = time.time() - self._mission_start_time
            cv2.putText(frame, f"MISSION TIME: {elapsed/60:.1f} min", (cx-150, cy+90), F, 0.7, (255,255,255), 2)

        if self.waiting_for_confirmation:
            if self.selecting_landing_side:
                cv2.putText(frame, "SELECT LANDING SIDE:", (cx-200, cy+60), F, 0.8, (0,255,255), 2)
                cv2.putText(frame, "N/S/W/E", (cx-60, cy+90), F, 0.8, (0,255,255), 2)
            else:
                cv2.putText(frame, "Y=Confirm  N=Reject  I=Interest", (cx-200, cy+80), F, 0.8, (0,0,255), 2)
                _vr = getattr(self, '_verify_remaining', None)
                if _vr is not None:
                    _vr_int = int(_vr)
                    _vr_color = (0, 0, 255) if _vr < 30 else (0, 165, 255) if _vr < 60 else (0, 255, 255)
                    cv2.putText(frame, f"Timeout: {_vr_int}s", (cx-80, cy+110), F, 0.7, _vr_color, 2)

        if hasattr(self, 'items_of_interest') and self.items_of_interest:
            y_off = frame.shape[0] - 30 * len(self.items_of_interest) - 10
            for idx, item in enumerate(self.items_of_interest):
                cv2.putText(frame, f"I{idx+1}: ({item['lat']:.5f}, {item['lon']:.5f})",
                            (10, y_off + idx*30), F, 0.6, (255,100,0), 2)

        # Servo animation (HOVER_TARGET only)
        if self.state == State.HOVER_TARGET and hasattr(self, '_hover_elapsed'):
            _draw_servo_animation(frame, self._hover_elapsed)

        # Composite view (simulation: god view + camera side by side)
        final_display = frame
        if config.MODE == "SIMULATION":
            god_frame = self.sim.get_god_view(
                px, py, self.yaw, self.view_w_px, self.view_h_px, self.zoom_level,
                self.planner.virtual_polygon, self.search_poly,
                (self.target_lat, self.target_lon), (self.landing_lat, self.landing_lon), self.geo,
                search_wps=self.waypoints, search_wp_index=self.wp_index,
                transit_wps_gps=self.pre_waypoints, transit_wp_index=self.pre_wp_index,
                current_state=self.state, rescan_pass=self.rescan_pass,
                items_of_interest=getattr(self, 'items_of_interest', None),
                rejected_targets=getattr(self, 'rejected_targets', None),
                nfz_buffer_m=config.NFZ_SLOW_ZONE_M if self.geofence else 0,
                nfz_repulsion_vec=getattr(self, '_last_repulsion_vec', None),
            )
            # Take-Off/Landing marker (cyan star) on god view
            if hasattr(config, 'TAKEOFF_GPS') and config.TAKEOFF_GPS[0] != 0:
                _gs = self.sim._god_scale
                tol_full = self.geo.gps_to_pixels(*config.TAKEOFF_GPS)
                tol_pt = (int(tol_full[0] * _gs), int(tol_full[1] * _gs))
                cv2.drawMarker(god_frame, tol_pt, (255, 255, 0), cv2.MARKER_STAR, max(8, int(20*_gs)), 2)
                cv2.putText(god_frame, "TOL", (tol_pt[0]+8, tol_pt[1]-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            # god_frame is already at display scale; just match height to camera frame
            h_scale = frame.shape[0] / god_frame.shape[0]
            god_resized = cv2.resize(god_frame, (int(god_frame.shape[1]*h_scale), frame.shape[0]))
            final_display = np.hstack((god_resized, frame))

        if STREAM_ENABLED:
            set_stream_frame(frame)
            spd_val = math.sqrt(self.vx**2 + self.vy**2)
            _mode_names = {0:'STABILIZE',2:'ALT_HOLD',3:'AUTO',4:'GUIDED',
                           5:'LOITER',6:'RTL',9:'LAND',16:'POSHOLD'}
            _geo_status = "OK"
            if getattr(self, '_fa_rtl_triggered', False):
                _geo_status = "VIOLATION"
            elif getattr(self, '_last_repulsion_vec', None):
                _geo_status = "WARN"
            set_telemetry(
                state=str(self.state).replace("State.", ""),
                alt=round(self.alt, 1),
                lat=self.lat,
                lon=self.lon,
                speed=round(spd_val, 1),
                conf=round(self.current_conf, 3),
                cmd_queue_size=stream_cmd_queue.qsize(),
                wp_index=self.wp_index,
                wp_total=len(self.waypoints) if self.waypoints else 0,
                gps_fix=self.gps_fix_type,
                gps_sats=self.gps_satellites,
                waiting=getattr(self, 'waiting_for_confirmation', False),
                selecting_side=getattr(self, 'selecting_landing_side', False),
                verify_timeout=getattr(self, '_verify_remaining', None),
                heading=round(math.degrees(self.yaw) % 360, 1),
                mode=_mode_names.get(self._cube_mode, str(self._cube_mode)),
                det_count=len(getattr(self, 'rejected_targets', [])) + len(getattr(self, 'items_of_interest', [])) + (1 if self.state in (State.VERIFY, State.CENTERING, State.DESCENDING) else 0),
                mission_elapsed=round(time.time() - self._mission_start_time, 1),
                rejected=len(getattr(self, 'rejected_targets', [])),
                interests=len(getattr(self, 'items_of_interest', [])),
                geofence=_geo_status,
            )
        if not HEADLESS:
            try:
                cv2.imshow("Mission Dashboard", final_display)
            except cv2.error as e:
                print(f"[WARN] cv2.imshow failed: {e}")
        return found, u, v

    # ── Main loop ─────────────────────────────────────────────────────

    def run(self):
        self._mission_start_time = time.time()
        det_dir = "mission_detections"
        if getattr(self, '_clean_detections', False):
            import shutil
            if os.path.exists(det_dir):
                shutil.rmtree(det_dir)
                print(f"[INIT] Cleared previous {det_dir}/")
        os.makedirs(det_dir, exist_ok=True)
        print("\n" + "=" * 60)
        print("  MISSION LOOP STARTED")
        print("  Keys: M=manual  Y/N/I=verify  K=reset  B=beacon  ESC=quit")
        print("=" * 60)
        if STREAM_ENABLED:
            srv = start_stream_server(port=STREAM_PORT, stream_w=STREAM_W, stream_h=STREAM_H,
                                      stream_fps=STREAM_FPS, stream_quality=STREAM_QUALITY)
            if srv is None:
                print("[WARN] Stream server failed to start!")
                print("  ACTION: Kill any other script using port %d and restart." % STREAM_PORT)

        threading.Thread(target=_terminal_input_thread, daemon=True).start()

        if not HEADLESS:
            cv2.namedWindow("Mission Dashboard", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            if config.MODE == "SIMULATION":
                cv2.setMouseCallback("Mission Dashboard", self.on_dashboard_mouse)
        else:
            print(f"[HEADLESS] Browser: http://localhost:{STREAM_PORT}/")

        _dispatch = {
            State.INIT:               self._handle_init,
            State.CONNECTING:         self._handle_connecting,
            State.ARMING:             self._handle_arming,
            State.TAKEOFF:            self._handle_takeoff,
            State.PRE_WAYPOINTS:      self._handle_pre_waypoints,
            State.TRANSIT_TO_SEARCH:  self._handle_transit_to_search,
            State.SEARCH:             self._handle_search,
            State.CENTERING:          self._handle_centering,
            State.DESCENDING:         self._handle_descending,
            State.VERIFY:             self._handle_verify,
            State.HOVER:              self._handle_hover,
            State.APPROACH:           self._handle_approach,
            State.HOVER_TARGET:       self._handle_hover_target,
            State.RETURN_TO_SEARCH:   self._handle_return_to_search,
            State.RETURN_FROM_MANUAL: self._handle_return_from_manual,
            State.RETURN_TRANSIT:     self._handle_return_transit,
            State.RETURN_HOME:        self._handle_return_home,
            State.LANDING:            self._handle_landing,
        }

        import queue as _queue
        key = -1
        while True:
            self.update_telemetry()
            target_found, px_u, px_v = self.update_dashboard()

            # Log ~1 Hz
            now = time.time()
            if now - self._last_log_time > 1.0:
                self.logger.writerow([datetime.now(), self.state, self.lat, self.lon, self.alt, self.current_conf])
                self.log_file.flush()
                self._last_log_time = now

            # RC OVERRIDE GUARD: if pilot switched away from GUIDED,
            # stop ALL commands. This prevents fighting the RC pilot.
            # Modes 4=GUIDED, 9=LAND are ours. Anything else = pilot has control.
            _pilot_override = self._cube_mode not in (4, 9, 6) and self.state not in (
                State.INIT, State.CONNECTING, State.ARMING, State.DONE)
            if _pilot_override:
                if not HEADLESS:
                    cv2.waitKey(1)  # Keep cv2 window responsive during override
                else:
                    time.sleep(0.02)
                time.sleep(0.05)
                continue  # Skip keys + state handler + geofence — pilot is flying

            # Read keys BEFORE handling (no 1-loop delay)
            key = -1
            if not HEADLESS:
                key = cv2.waitKey(1) & 0xFF
            else:
                time.sleep(0.02)
            # Drain ALL queued keys and process each immediately
            _queued_keys = []
            try:
                while True:
                    _queued_keys.append(stream_cmd_queue.get_nowait())
            except _queue.Empty:
                pass
            for _qk in _queued_keys:
                self._handle_keys(_qk, target_found, px_u, px_v)
                handler = _dispatch.get(self.state)
                if handler:
                    handler(target_found, px_u, px_v, _qk)
            if key == 27: break
            # Use cv2 key if no queued key
            if not _queued_keys and key != 255 and key != -1:
                _queued_keys = [key]  # mark as handled below

            self._handle_keys(key, target_found, px_u, px_v)

            handler = _dispatch.get(self.state)
            if handler:
                handler(target_found, px_u, px_v, key)

            # Geofence enforcement
            self._last_repulsion_vec = None
            if (self.geofence and self.master and self.lat != 0
                    and self.state not in (State.INIT, State.CONNECTING, State.ARMING,
                                           State.TAKEOFF, State.LANDING, State.DONE)):
                skip_speed = self.state in (State.HOVER_TARGET, State.APPROACH,
                                            State.RETURN_TRANSIT, State.RETURN_HOME)
                self._enforce_geofence(skip_speed_clamp=skip_speed)

            # Continuous yaw enforcement (--lock-yaw)
            if config.LOCK_YAW and self.master and hasattr(self, '_search_yaw_target'):
                self._enforce_search_yaw()

            if self.state == State.DONE:
                self.update_dashboard()  # Render DONE HUD with landing distances
                (cv2.waitKey(10000) if not HEADLESS else time.sleep(10.0))
                break
            if key == 27: break

    def _enforce_search_yaw(self):
        # keep yaw within ±15 deg of the search heading (only if --lock-yaw is on)
        if self.state not in (State.SEARCH, State.RETURN_TRANSIT, State.RETURN_HOME, State.PRE_WAYPOINTS, State.TRANSIT_TO_SEARCH):
            return
        import math
        from pymavlink import mavutil
        target_deg = self._search_yaw_target
        current_deg = math.degrees(self.yaw) % 360
        error = (target_deg - current_deg + 180) % 360 - 180  # signed, -180..+180
        if abs(error) > 15.0:
            # rate-limit to avoid overshoot
            now = time.time()
            if now - getattr(self, '_last_yaw_correction', 0) < 5.0:
                return
            self._last_yaw_correction = now
            direction = 1 if error > 0 else -1
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                target_deg, 15, direction, 0, 0, 0, 0)  # 15 deg/s, used to be 30

    def _enforce_geofence(self, skip_speed_clamp=False):
        """NFZ speed cap + inner polygon repulsion + flight area containment.
        skip_speed_clamp=True skips the speed clamp (we use that during
        approach/return where the clamp fights the altitude controller)."""
        # R01 Flight Area containment check
        try:
            if hasattr(config, 'FLIGHT_AREA_GPS') and len(config.FLIGHT_AREA_GPS) >= 3:
                fa_poly = np.array([(lon, lat) for lat, lon in config.FLIGHT_AREA_GPS], dtype=np.float32)
                pos = (self.sm.lon if hasattr(self, 'sm') else self.lon,
                       self.sm.lat if hasattr(self, 'sm') else self.lat)
                inside_fa = cv2.pointPolygonTest(fa_poly, pos, False)
                if inside_fa < 0:
                    if not getattr(self, '_fa_rtl_triggered', False):
                        print("[GEOFENCE] OUTSIDE FLIGHT AREA — triggering RTL")
                        self._emergency_rtl(reason="Outside Flight Area boundary")
                        self._fa_rtl_triggered = True
                        self.sm._set_state(State.LANDING)
                    return
                else:
                    self._fa_rtl_triggered = False
        except Exception as e:
            print(f"[GEOFENCE] Flight Area check error: {e}")

        nfz_dist, nfz_inside = self.geofence.distance_to_boundary(self.lat, self.lon)

        # Compute NFZ approach limit + toward vector (used by manual AND navigation)
        if not nfz_inside and nfz_dist < config.NFZ_SLOW_ZONE_M:
            if nfz_dist <= config.NFZ_SCALAR_ZERO_M:
                self._nfz_manual_max_speed = 0.3
            else:
                ratio = (nfz_dist - config.NFZ_SCALAR_ZERO_M) / (config.NFZ_SLOW_ZONE_M - config.NFZ_SCALAR_ZERO_M)
                self._nfz_manual_max_speed = ratio * config.NFZ_ZONE_MAX_SPEED_MPS
            # Compute toward-NFZ unit vector for directional clamping
            best_dist_sq = float('inf')
            nfz_lat, nfz_lon = self.lat, self.lon
            poly = self.geofence.sssi_polygon_gps
            for i in range(len(poly)):
                p1_lat, p1_lon = poly[i]
                p2_lat, p2_lon = poly[(i + 1) % len(poly)]
                e_n = (p2_lat - p1_lat) * 111320
                e_e = (p2_lon - p1_lon) * 111320 * math.cos(math.radians(self.lat))
                d_n = (self.lat - p1_lat) * 111320
                d_e = (self.lon - p1_lon) * 111320 * math.cos(math.radians(self.lat))
                e_len_sq = e_n * e_n + e_e * e_e
                t = max(0.0, min(1.0, (d_n * e_n + d_e * e_e) / e_len_sq)) if e_len_sq > 1e-12 else 0.0
                c_lat = p1_lat + t * (p2_lat - p1_lat)
                c_lon = p1_lon + t * (p2_lon - p1_lon)
                dsq = ((self.lat - c_lat) * 111320) ** 2 + \
                      ((self.lon - c_lon) * 111320 * math.cos(math.radians(self.lat))) ** 2
                if dsq < best_dist_sq:
                    best_dist_sq = dsq
                    nfz_lat, nfz_lon = c_lat, c_lon
            dx = (nfz_lon - self.lon) * 111320 * math.cos(math.radians(self.lat))
            dy = (nfz_lat - self.lat) * 111320
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.1:
                self._nfz_toward_vec = (dy / dist, dx / dist)  # (north, east) toward NFZ
            else:
                self._nfz_toward_vec = None
        else:
            self._nfz_manual_max_speed = None
            self._nfz_toward_vec = None

        if nfz_inside and self.state != State.MANUAL:
            if skip_speed_clamp:
                # During approach/return/hover: do NOT switch to MANUAL.
                # Don't send velocity commands (they override position targets and
                # kill altitude control with vz=0).  The state handler's
                # send_global_target already ran this tick — just let it work.
                # The position target pulls the drone back outside NFZ naturally.
                print(f"[GEOFENCE] Brief NFZ incursion during {self.state.name} — "
                      f"position target active, no MANUAL switch")
                return
            print(f"[GEOFENCE] INSIDE NFZ! Switching to MANUAL")
            if self.state != State.RETURN_FROM_MANUAL:
                self.previous_state = self.state
                self.manual_departure_lat = self.lat
                self.manual_departure_lon = self.lon
                self.manual_departure_alt = self.alt
            self._set_state(State.MANUAL)
            if self.nav: self.nav.send_velocity(0, 0, 0)
            return

        if skip_speed_clamp:
            # After Y confirmed: approach/return/hover — no speed clamping
            # (it bleeds into vertical via 3D position controller, blocks descent/climb).
            pass
        elif NFZ_DIRECTIONAL and not nfz_inside and nfz_dist < config.NFZ_SLOW_ZONE_M:
            # Ramp: 0 m/s at SCALAR_ZERO_M (2m), linearly up to ZONE_MAX at SLOW_ZONE_M (20m)
            if nfz_dist <= config.NFZ_SCALAR_ZERO_M:
                max_approach = 0.0
            else:
                ratio = (nfz_dist - config.NFZ_SCALAR_ZERO_M) / (config.NFZ_SLOW_ZONE_M - config.NFZ_SCALAR_ZERO_M)
                max_approach = ratio * config.NFZ_ZONE_MAX_SPEED_MPS
            # Compute unit vector toward nearest NFZ boundary point
            best_dist_sq = float('inf')
            nfz_lat, nfz_lon = self.lat, self.lon
            poly = self.geofence.sssi_polygon_gps
            for i in range(len(poly)):
                p1_lat, p1_lon = poly[i]
                p2_lat, p2_lon = poly[(i + 1) % len(poly)]
                e_n = (p2_lat - p1_lat) * 111320
                e_e = (p2_lon - p1_lon) * 111320 * math.cos(math.radians(self.lat))
                d_n = (self.lat - p1_lat) * 111320
                d_e = (self.lon - p1_lon) * 111320 * math.cos(math.radians(self.lat))
                e_len_sq = e_n * e_n + e_e * e_e
                if e_len_sq < 1e-12:
                    t = 0.0
                else:
                    t = max(0.0, min(1.0, (d_n * e_n + d_e * e_e) / e_len_sq))
                c_lat = p1_lat + t * (p2_lat - p1_lat)
                c_lon = p1_lon + t * (p2_lon - p1_lon)
                dsq = ((self.lat - c_lat) * 111320) ** 2 + \
                      ((self.lon - c_lon) * 111320 * math.cos(math.radians(self.lat))) ** 2
                if dsq < best_dist_sq:
                    best_dist_sq = dsq
                    nfz_lat, nfz_lon = c_lat, c_lon
            dx = (nfz_lon - self.lon) * 111320 * math.cos(math.radians(self.lat))
            dy = (nfz_lat - self.lat) * 111320
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.1:
                toward_n = dy / dist
                toward_e = dx / dist
                vn = getattr(self, 'vx', 0)
                ve = getattr(self, 'vy', 0)
                speed = math.sqrt(vn * vn + ve * ve)
                if speed > 0.3:
                    cos_angle = (vn * toward_n + ve * toward_e) / speed
                    if cos_angle > 0.05:
                        allowed = max_approach / cos_angle
                        if allowed < speed:
                            self.nav.last_speed_req = 0
                            self.nav.set_speed(max(0.3, allowed))
        elif not NFZ_DIRECTIONAL and not nfz_inside and nfz_dist < config.NFZ_SLOW_ZONE_M:
            if nfz_dist <= config.NFZ_SCALAR_ZERO_M:
                max_spd = 0.0
            else:
                ratio = (nfz_dist - config.NFZ_SCALAR_ZERO_M) / (config.NFZ_SLOW_ZONE_M - config.NFZ_SCALAR_ZERO_M)
                max_spd = ratio * config.NFZ_ZONE_MAX_SPEED_MPS
            self.nav.last_speed_req = 0
            self.nav.set_speed(max_spd)

        # Repulsive push — only in MANUAL mode (send_velocity fights navigation in other states)
        if self.nav and self.state == State.MANUAL:
            signed_dist = nfz_dist if not nfz_inside else -nfz_dist
            dist_to_inner = signed_dist + config.NFZ_INNER_OFFSET_M
            if 0 < dist_to_inner < config.NFZ_INNER_RANGE_M:
                off_lat, off_lon = self.geofence.repulsive_offset(self.lat, self.lon)
                if abs(off_lat) > 1e-8 or abs(off_lon) > 1e-8:
                    self._last_repulsion_vec = (off_lat, off_lon)
                    lat_m = 111320.0
                    lon_m = 111320.0 * math.cos(math.radians(self.lat))
                    push_n, push_e = -off_lat * lat_m, -off_lon * lon_m
                    if nfz_inside:
                        push_n, push_e = -push_n, -push_e
                    mag = math.sqrt(push_n**2 + push_e**2)
                    if mag > 0.01:
                        s = config.NFZ_PUSH_SPEED_MPS
                        self.nav.send_velocity(push_n/mag*s, push_e/mag*s, 0, current_yaw=0.0)

    def _emergency_rtl(self, reason="unknown"):
        # best-effort RTL. if this fails, ArduCopter's own GCS failsafe should catch it.
        print("=" * 60)
        print(f"  EMERGENCY: Attempting RTL  —  {reason}")
        print("  If RTL fails, ArduCopter GCS failsafe should trigger RTL")
        print("  RC kill switch is always available")
        print("=" * 60)
        try:
            if self.master:
                # Try setting RTL mode directly via MAVLink (mode 6)
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    6, 0, 0, 0, 0, 0)  # 6 = RTL
                print("[EMERGENCY] RTL command sent")
        except Exception as e2:
            print(f"[EMERGENCY] RTL send failed: {e2}")
            print("[EMERGENCY] Relying on ArduCopter GCS failsafe (FS_GCS_ENABLE)")

    def on_dashboard_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0: self.zoom_level = min(self.zoom_level * 1.2, 20.0)
            else: self.zoom_level = max(self.zoom_level / 1.2, 1.0)


# ── Dry-run ───────────────────────────────────────────────────────────

def _dry_run(mission):
    # plot the search pattern and print a summary. no Cube needed.
    print("\n" + "=" * 60)
    print(f"  DRY-RUN: Generating {'spiral' if USE_SPIRAL else 'lawnmower'} search pattern")
    print("=" * 60)

    if not mission.search_poly or len(mission.search_poly) < 3:
        print("  ERROR: No search polygon. Set SEARCH_AREA_GPS in config.py.")
        return

    print(f"\n  Search area: {len(mission.search_poly)} corners")
    for i, pt in enumerate(mission.search_poly):
        gps = mission.geo.pixels_to_gps(pt[0], pt[1]) if hasattr(pt, '__len__') else (0, 0)
        print(f"    {i+1}: ({gps[0]:.6f}, {gps[1]:.6f})")

    drone_gps = (config.REF_LAT, config.REF_LON)
    print(f"\n  Start: ({drone_gps[0]:.6f}, {drone_gps[1]:.6f})  Alt: {config.TARGET_ALT}m  Speed: {config.SEARCH_SPEED_MPS}m/s")

    waypoints = mission.planner.generate_search_pattern(config.REAL_CANVAS_SIZE, config.REAL_CANVAS_SIZE, drone_gps)
    print(f"  Pattern: {'Spiral' if USE_SPIRAL else 'Lawnmower'}")
    if not waypoints:
        print("  ERROR: No waypoints generated!")
        return

    total_dist = 0
    for i in range(1, len(waypoints)):
        prev = waypoints[i-1]; wp = waypoints[i]
        total_dist += 111320 * math.sqrt(
            (wp[0]-prev[0])**2 + ((wp[1]-prev[1])*math.cos(math.radians(wp[0])))**2)

    print(f"  {len(waypoints)} waypoints, {total_dist:.0f}m search path")
    for i, wp in enumerate(waypoints):
        tag = " <START" if i == 0 else (" <END" if i == len(waypoints)-1 else "")
        print(f"    WP {i+1:3d}: ({wp[0]:.6f}, {wp[1]:.6f}){tag}")

    wp0 = waypoints[0]
    transit_dist = 111320 * math.sqrt(
        (wp0[0]-drone_gps[0])**2 + ((wp0[1]-drone_gps[1])*math.cos(math.radians(drone_gps[0])))**2)
    total_time = transit_dist/config.TRANSIT_SPEED_MPS + total_dist/config.SEARCH_SPEED_MPS
    print(f"\n  Transit: {transit_dist:.0f}m  Search: {total_dist:.0f}m  Total: {total_time:.0f}s ({total_time/60:.1f}min)")

    # State machine walkthrough
    print(f"\n  States: INIT->CONNECT->ARM(GPS>=3,sats>=6)->TAKEOFF({config.TARGET_ALT}m)")
    print(f"    ->SEARCH({len(waypoints)}wp)->CENTERING->DESCEND({config.VERIFY_ALT}m)->VERIFY->LAND")

    # Map visualization
    map_img = cv2.imread(config.MAP_FILE)
    if map_img is not None:
        vis = map_img.copy()

        # Flight Area (blue outline) — outermost boundary
        if hasattr(config, 'FLIGHT_AREA_GPS') and len(config.FLIGHT_AREA_GPS) >= 3:
            fa_pts = [tuple(int(c) for c in mission.geo.gps_to_pixels(lat, lon))
                      for lat, lon in config.FLIGHT_AREA_GPS]
            cv2.polylines(vis, [np.array(fa_pts, np.int32)], True, (255, 150, 0), 2)

        # SSSI No-Fly Zone (red, semi-transparent fill + outline)
        if hasattr(config, 'SSSI_GPS') and len(config.SSSI_GPS) >= 3:
            sssi_pts = []
            for lat, lon in config.SSSI_GPS:
                px, py = mission.geo.gps_to_pixels(lat, lon)
                sssi_pts.append((int(px), int(py)))
            sssi_arr = np.array(sssi_pts, np.int32)
            overlay = vis.copy()
            cv2.fillPoly(overlay, [sssi_arr], (0, 0, 200))
            cv2.addWeighted(overlay, 0.3, vis, 0.7, 0, vis)
            cv2.polylines(vis, [sssi_arr], True, (0, 0, 255), 2)
            cx_s, cy_s = sssi_arr.mean(axis=0).astype(int)
            cv2.putText(vis, "SSSI NFZ", (cx_s - 30, cy_s),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        # Search Area (green outline)
        cv2.polylines(vis, [np.array(mission.search_poly, np.int32)], True, (0,255,0), 2)

        # Waypoints + path
        for i, wp in enumerate(waypoints):
            pt = tuple(int(c) for c in mission.geo.gps_to_pixels(wp[0], wp[1]))
            color = (0,0,255) if i==0 else ((255,0,0) if i==len(waypoints)-1 else (255,255,0))
            cv2.circle(vis, pt, 4, color, -1)
            if i > 0:
                prev = tuple(int(c) for c in mission.geo.gps_to_pixels(waypoints[i-1][0], waypoints[i-1][1]))
                cv2.line(vis, prev, pt, (255,255,0), 1)

        # Take-Off/Landing marker (cyan star)
        if hasattr(config, 'TAKEOFF_GPS') and config.TAKEOFF_GPS[0] != 0:
            tol_pt = tuple(int(c) for c in mission.geo.gps_to_pixels(*config.TAKEOFF_GPS))
            cv2.drawMarker(vis, tol_pt, (255, 255, 0), cv2.MARKER_STAR, 20, 2)
            cv2.putText(vis, "TOL", (tol_pt[0]+10, tol_pt[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Drone start position
        dp = tuple(int(c) for c in mission.geo.gps_to_pixels(*drone_gps))
        cv2.drawMarker(vis, dp, (0,255,255), cv2.MARKER_DIAMOND, 15, 2)

        # Title
        cv2.putText(vis, f"DRY-RUN: {len(waypoints)} WPs, {total_dist:.0f}m, ~{total_time/60:.1f}min",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        # Legend
        legend_y = 60
        for label, color in [("Flight Area", (255,150,0)), ("Search Area", (0,255,0)),
                             ("SSSI (NFZ)", (0,0,255)), ("TOL", (255,255,0)),
                             ("WP Start", (0,0,255)), ("WP End", (255,0,0))]:
            cv2.rectangle(vis, (10, legend_y), (25, legend_y+12), color, -1)
            cv2.putText(vis, label, (30, legend_y+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            legend_y += 18

        scale = min(1.0, 900 / vis.shape[0])
        if scale < 1.0:
            vis = cv2.resize(vis, (int(vis.shape[1]*scale), int(vis.shape[0]*scale)))
        if not HEADLESS:
            cv2.imshow("Dry-Run: Search Pattern", vis)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        cv2.imwrite("dry_run_pattern.jpg", vis)
        print(f"  Saved: dry_run_pattern.jpg")
    else:
        print("  (No map.jpg — skipping visualization)")

    print(f"\n  DRY-RUN COMPLETE. No commands sent.")
    print("=" * 60)


if __name__ == "__main__":
    mission = VisualFlightMission()
    mission._clean_detections = CLEAN_DETECTIONS
    if DRY_RUN:
        _dry_run(mission)
    else:
        try:
            mission.run()
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError) as e:
            print(f"\n[LINK LOST] {e}")
            mission._emergency_rtl(reason=str(e))
        except KeyboardInterrupt:
            print("\n[USER] Aborted — attempting RTL before exit")
            mission._emergency_rtl(reason="KeyboardInterrupt")
        finally:
            if hasattr(mission, 'log_file') and mission.log_file:
                mission.log_file.close()
            if not HEADLESS:
                cv2.destroyAllWindows()
