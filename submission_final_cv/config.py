# config.py — mission settings live here
# values only, no real logic except the auto-detect and the KML loader

import os
import subprocess
import platform


# ============================================================
#  MODE
# ============================================================
# SIMULATION = map.jpg + mouse drawing, REAL = live camera + GPS
# override: export DRONE_MODE=REAL
MODE = os.environ.get("DRONE_MODE", "SIMULATION")


# ============================================================
#  CONNECTION
# ============================================================
def _detect_connection():
    # pi serial -> wsl gateway -> localhost, in that order
    env = os.environ.get("DRONE_CONN")
    if env:
        return env
    # pi: if a serial port exists assume mavproxy UDP bridge is up
    for port in ["/dev/ttyAMA0", "/dev/ttyACM0", "/dev/ttyUSB0"]:
        if os.path.exists(port):
            return "udpin:0.0.0.0:14550"
    # WSL: reach the Windows SITL via the default gateway
    if platform.system() == "Linux":
        try:
            with open("/proc/version", "r") as f:
                if "microsoft" in f.read().lower():
                    result = subprocess.run(
                        ["ip", "route", "show", "default"],
                        capture_output=True, text=True,
                    )
                    parts = result.stdout.strip().split()
                    if "via" in parts:
                        gw = parts[parts.index("via") + 1]
                        return f"tcp:{gw}:5762"
        except Exception:
            pass
    # fall back to Windows localhost SITL
    return "tcp:127.0.0.1:5762"

CONNECTION_STR = _detect_connection()  # override: export DRONE_CONN=tcp:IP:PORT
BAUD_RATE  = int(os.environ.get("DRONE_BAUD", 921600))


# ============================================================
#  FLIGHT — altitudes & speeds
# ============================================================
TARGET_ALT = 35.0               # search altitude (m)
VERIFY_ALT = 15.0               # close-up descent altitude (m)

if TARGET_ALT > 50:
    print(f"WARNING: TARGET_ALT={TARGET_ALT}m exceeds R04 limit of 50m, capping")
    TARGET_ALT = 50

TRANSIT_SPEED_MPS = 15.0        # speed to/from search area
SEARCH_SPEED_MPS = 10.0         # default search pass speed
FOCUS_SEARCH_SPEED_MPS = 5.0    # slower in Focus Area (PLB zone)

# altitude-dependent speed, linear ramp between LOW and HIGH alts
SPEED_ALT_LOW = 20.0
SPEED_ALT_HIGH = 50.0
SPEED_AT_LOW = 6.0              # m/s at low alt (less blur)
SPEED_AT_HIGH = 10.0            # m/s at high alt (faster coverage)

def speed_for_altitude(alt):
    # slower at low alt, faster up high
    if alt <= SPEED_ALT_LOW:
        return SPEED_AT_LOW
    if alt >= SPEED_ALT_HIGH:
        return SPEED_AT_HIGH
    ratio = (alt - SPEED_ALT_LOW) / (SPEED_ALT_HIGH - SPEED_ALT_LOW)
    return SPEED_AT_LOW + ratio * (SPEED_AT_HIGH - SPEED_AT_LOW)


# ============================================================
#  CAMERA — sensor & optics
# ============================================================
SENSOR_WIDTH_MM = 5.02          # IMX296 sensor width
FOCAL_LENGTH_MM = 5.46          # calibrated 2026-03-11 (92 cm visible at 1 m)
IMAGE_W = 1456                  # IMX296 native width
IMAGE_H = 1088                  # IMX296 native height
REAL_CAMERA_INDEX = 0           # /dev/video0 on Pi

# white balance (picamera2 only)
CAMERA_AWB_MODE = "auto"        # auto / daylight / cloudy / indoor / manual
CAMERA_COLOUR_GAINS = (1.5, 1.2)  # (red, blue), only used when AWB_MODE = manual
CAMERA_COLOR_CORRECTION = False # software gray-world correction, not needed
CAMERA_FLIP_180 = True          # camera is mounted inverted on the drone
UNDISTORT_ENABLED = False       # flip to True once a valid calibration_data.npz exists
# TODO: turn UNDISTORT_ENABLED back on once we have a fresh IMX296 calibration run


# ============================================================
#  DETECTION — CV thresholds & confirmation
# ============================================================
CONFIDENCE_THRESHOLD = 0.2      # min YOLO confidence, kept low so the operator filters FPs
DETECT_CONFIRM_FRAMES = 3       # consecutive frames before triggering (--smart-detect)
DETECT_LOCK_RADIUS_M = 5.0      # ignore detections outside this from the locked target
MAX_DETECT_QUEUE = 20           # queue cap, oldest dropped when exceeded


# ============================================================
#  TARGET SPECS
# ============================================================
TARGET_REAL_RADIUS_M = 0.15     # dummy radius, 15 cm
DUMMY_HEIGHT_M = 1.8            # dummy height, for FOV maths
CONE_HEIGHT_M = 0.5
PANTS_HEIGHT_M = 0.9
TSHIRT_HEIGHT_M = 0.8
BACKPACK_HEIGHT_M = 0.7


# ============================================================
#  NFZ GEOFENCE
# ============================================================
NFZ_HARD_BOUNDARY_M = 3.0      # inside this => force MANUAL
NFZ_SOFT_BOUNDARY_M = 8.0      # quadratic repulsion zone (legacy --nfz-repel)
NFZ_WAYPOINT_BUFFER_M = 30.0   # skip waypoints within this of NFZ
NFZ_SLOW_ZONE_M = 20.0         # speed scalar field active within this (20m -> 2m ramp)
NFZ_SCALAR_ZERO_M = 2.0        # speed drops to 0 at this distance (full stop zone)
NFZ_MIN_SPEED_MPS = 0.0        # speed at NFZ_SCALAR_ZERO_M and below (0 = full stop)
NFZ_ZONE_MAX_SPEED_MPS = 3.0   # speed at outer edge of slow zone (20m out)
NFZ_INNER_OFFSET_M = 20.0      # inner (pink) polygon offset inside NFZ
NFZ_INNER_RANGE_M   = 23.0     # repulsion active within this from inner polygon (~3m from boundary)
NFZ_PUSH_SPEED_MPS = 5.0       # constant push away from NFZ


# ============================================================
#  SEARCH AREA — zones loaded from KML
# ============================================================
# fallback coords from AENGM0074.kml. load_kml_zones() overwrites these when the KML is present.
# these MUST match the KML, a wrong fallback means wrong search area or missing geofence
SEARCH_AREA_GPS = [
    (51.42326956502679, -2.670948345438704),
    (51.42287025017865, -2.670045428650557),
    (51.42336622593724, -2.668169295906676),
    (51.42421477437771, -2.668809768621569),
    (51.42354069739116, -2.671277780473196),
]
FLIGHT_AREA_GPS = [
    (51.42342595349562, -2.671720766408759),
    (51.42124623420381, -2.670134027271237),
    (51.42244011936099, -2.66568781888585),
    (51.42469179370701, -2.667060227266051),
]
SSSI_GPS = [
    (51.42353586816967, -2.671451754138619),
    (51.42215640321154, -2.669768242108598),
    (51.42267105383615, -2.667705438815299),
    (51.42335592245168, -2.668164601092489),
    (51.42286082606338, -2.670043418345824),
    (51.42326667015552, -2.670965419051837),
    (51.42356862274763, -2.671324297543731),
]
TAKEOFF_GPS = (51.42340640206451, -2.671446029622069)
FOCUS_AREA_GPS = [
    (51.42330493862503, -2.669823704225677),
    (51.42344370699984, -2.669496195078445),
    (51.42352782091507, -2.669800245046278),
    (51.42334972740506, -2.67001828046102),
]

REJECTED_TARGET_RADIUS_M = 5.0  # skip detections near rejected/IOI targets
MAX_RESCAN_PASSES = 3           # altitude-drop rescan passes before giving up
RESCAN_ALT_FACTOR = 0.8         # altitude multiplier per rescan pass
RESCAN_ALT_FLOOR_M = 15.0       # minimum rescan altitude

DIAGONAL_YAW_OFFSET_DEG = 0     # yaw offset at turn (0 = face scan line, None = auto)


# ============================================================
#  SIMULATION MAP
# ============================================================
MAP_FILE = "assets/map.jpg"
DUMMY_FILE = "assets/dummy.png"
CONE_FILE = "assets/cone.png"
PANTS_FILE = "assets/pants.png"
TSHIRT_FILE = "assets/tshirt.png"
BACKPACK_FILE = "assets/backpack.png"
MAP_WIDTH_METERS = 491.9
# map top-left corner, calibrated 2026-04-02, 2-point, ~0.11m avg err
REF_LAT = 51.42512107
REF_LON = -2.67222427


# ============================================================
#  PAYLOAD SERVO — release mechanism (Tarot double-throw)
# ============================================================
SERVO_CHANNEL = 9               # Cube AUX channel — check SERVOx_FUNCTION in MP
SERVO_CLOSE_PWM = 1500          # hold / re-latch
SERVO_PARTIAL_PWM = 1300        # stage 1: partial release
SERVO_FULL_PWM = 1100           # stage 2: full release


# ============================================================
#  MANUAL FLIGHT (reserved — not yet wired)
# ============================================================
MANUAL_FLY_SPEED_MPS = 5.0     # WASD horizontal
MANUAL_CLIMB_RATE_MPS = 2.0    # R/F vertical
MANUAL_YAW_STEP_DEG = 10       # Q/E yaw per keypress
MANUAL_YAW_RATE_DEGS = 30.0    # yaw rotation speed


# ============================================================
#  LOGGING
# ============================================================
LOG_FILE = "logs/flight_log.csv"


# ============================================================
#  KML LOADER
# ============================================================
def load_kml_zones(kml_path="flight_plans/AENGM0074.kml"):
    # parse the KML and fill in the GPS zone globals
    global SEARCH_AREA_GPS, FLIGHT_AREA_GPS, SSSI_GPS, TAKEOFF_GPS, FOCUS_AREA_GPS
    import xml.etree.ElementTree as ET

    if not os.path.exists(kml_path):
        print(f"[CONFIG] KML not found: {kml_path} — using fallback SEARCH_AREA_GPS")
        return False

    ns = {"kml": "http://www.opengis.net/kml/2.2"}
    tree = ET.parse(kml_path)
    root = tree.getroot()

    def parse_coords(coord_text):
        # turn "lon,lat,alt lon,lat,alt ..." into [(lat, lon), ...]
        pts = []
        for token in coord_text.strip().split():
            parts = token.split(",")
            if len(parts) >= 2:
                lon, lat = float(parts[0]), float(parts[1])
                pts.append((lat, lon))
        if len(pts) > 1 and pts[0] == pts[-1]:
            pts = pts[:-1]
        return pts

    for pm in root.iter("{http://www.opengis.net/kml/2.2}Placemark"):
        name_el = pm.find("kml:name", ns)
        if name_el is None:
            continue
        name = name_el.text.strip()

        # Point placemark (take-off)
        point = pm.find(".//kml:Point/kml:coordinates", ns)
        if point is not None:
            parts = point.text.strip().split(",")
            if len(parts) >= 2:
                lon, lat = float(parts[0]), float(parts[1])
                if "take" in name.lower() or "off" in name.lower():
                    TAKEOFF_GPS = (lat, lon)

        # polygon placemarks
        coords = pm.find(
            ".//kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates", ns
        )
        if coords is not None:
            pts = parse_coords(coords.text)
            lower = name.lower()
            if "survey" in lower:
                SEARCH_AREA_GPS = pts
            elif "flight" in lower:
                FLIGHT_AREA_GPS = pts
            elif "sssi" in lower:
                SSSI_GPS = pts
            elif "focus" in lower:
                FOCUS_AREA_GPS = pts

    print(f"[CONFIG] Loaded KML: {kml_path}")
    print(f"  Take-Off:     {TAKEOFF_GPS}")
    print(f"  Survey Area:  {len(SEARCH_AREA_GPS)} corners")
    print(f"  Flight Area:  {len(FLIGHT_AREA_GPS)} corners")
    print(f"  SSSI:         {len(SSSI_GPS)} corners")
    return True


# ============================================================
#  CLI-OVERRIDABLE CONSTANTS (shared between main.py & state_machine.py)
# ============================================================
# these live here to dodge a circular import. main.py's CLI parser
# overwrites them at startup via  config.REAL_CANVAS_SIZE = ...
REAL_CANVAS_SIZE = 4800
SIM_SPEED = 1
BEACON_DELAY = 0
LOCK_YAW = False
