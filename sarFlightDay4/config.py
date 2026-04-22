"""
config.py — All configuration in one place.
============================================
Change values here, nowhere else.
"""

# ═══════════════════════════════════════════════════════════════
#  MAVProxy — started automatically by main.py
# ═══════════════════════════════════════════════════════════════

MAVPROXY_MASTER = "/dev/ttyAMA0"
MAVPROXY_MASTER_BAUD = 921600


MAVPROXY_OUTPUTS = [
    "udp:ROBINCARTER17AB.local:14550",   # Mission Planner on Windows VM
                                   # (MP must be in UDP listen mode on port 14550)
                                   # If hostname doesn't resolve, replace with the VM's IP address
    "udp:127.0.0.1:14551",          # State machine (Pi-local UDP)
    #"tcp:127.0.0.1:5762", # For SITL testing
]

# ═══════════════════════════════════════════════════════════════
#  State machine connection
# ═══════════════════════════════════════════════════════════════

CONNECTION_STRING = "udp:127.0.0.1:14551" # - Real flight
#CONNECTION_STRING = "tcp:127.0.0.1:5762" # - SITL
BAUD_RATE = 921600

# ═══════════════════════════════════════════════════════════════
#  Web dashboard
# ═══════════════════════════════════════════════════════════════

GUI_HOST = "0.0.0.0"       # 0.0.0.0 = accessible from any device on network
GUI_PORT = 5000             # Open http://<pi-ip>:5000 in your browser

# ═══════════════════════════════════════════════════════════════
#  Timing
# ═══════════════════════════════════════════════════════════════

HEARTBEAT_INTERVAL = 2.0
MODE_CHANGE_TIMEOUT = 10.0
MAVPROXY_STARTUP_DELAY = 3.0

# ═══════════════════════════════════════════════════════════════
#  Trigger file (works alongside the GUI)
# ═══════════════════════════════════════════════════════════════

TRIGGER_FILE = "/tmp/sar_trigger.txt"

# ═══════════════════════════════════════════════════════════════
#  Mission log file
# ═══════════════════════════════════════════════════════════════
# Human-readable mission log written throughout each run.
# A new file is created on each startup (timestamped).
# Set LOG_DIR to a path that exists on your system.

import os as _os

# Directory containing this config file (sarFlightDay4/).
# Used to resolve relative file paths regardless of CWD.
_PROJECT_DIR = _os.path.dirname(_os.path.abspath(__file__))

LOG_DIR  = _os.path.expanduser("~/sar_logs")   # ~/sar_logs on Pi, adjust for Windows
LOG_FILE = None    # Set at runtime by mission_log.py — do not edit manually

# ═══════════════════════════════════════════════════════════════
#  KML data file
# ═══════════════════════════════════════════════════════════════

KML_FILE = _os.path.join(_PROJECT_DIR, "AENGM0074.kml")  # Mission KML

# ═══════════════════════════════════════════════════════════════
#  Fence configuration
# ═══════════════════════════════════════════════════════════════

# FENCE_ENABLE: 1 = enabled, 0 = disabled
FENCE_ENABLE_VALUE = 1

# FENCE_TYPE bitmap:
#   bit 0 (1) = max altitude fence
#   bit 1 (2) = circular fence around HOME
#   bit 2 (4) = polygon inclusion/exclusion fences
#   bit 3 (8) = min altitude fence
# We want polygon (4) + max altitude (1) = 5
FENCE_TYPE_VALUE = 4

# FENCE_ACTION for Copter:
#   0 = report only
#   1 = RTL
#   2 = land
#   3 = brake or land (SmartRTL)
#   4 = brake or land
FENCE_ACTION_VALUE = 1     # RTL on breach (recommended for Flight Area)

# FENCE_ALT_MAX: maximum altitude in metres (R04 = 50m)
FENCE_ALT_MAX_VALUE = 50

# FENCE_MARGIN: buffer distance in metres inside the fence
# boundary before triggering a breach.
# 0.5 m = tight tolerance — drone can get very close to the
# boundary before ArduPilot triggers a breach action (RTL).
FENCE_MARGIN_VALUE = 0.5

# ═══════════════════════════════════════════════════════════════
#  ArduPilot Copter flight mode IDs
# ═══════════════════════════════════════════════════════════════

FLIGHT_MODES = {
    "STABILIZE": 0,
    "ALT_HOLD":  2,
    "AUTO":      3,
    "GUIDED":    4,
    "LOITER":    5,
    "RTL":       6,
    "LAND":      9,
}

MODE_NAMES = {v: k for k, v in FLIGHT_MODES.items()}

# ═══════════════════════════════════════════════════════════════
#  Search pattern generation
# ═══════════════════════════════════════════════════════════════
# These are placeholder defaults.  Your CV teammate should
# replace them with values derived from camera FOV, GSD, and
# overlap requirements.

SWATH_WIDTH_M = 10       # Spacing between parallel scan lines (metres) - Unused for demo day
SEARCH_ALT_M  = 25       # AGL altitude for initial search waypoints (metres)
SCAN_HEADING_DEG = 0     # 0 = scan lines run East–West, 90 = North–South

# Lawnmower algorithm selector:
#   "scanline" — Carter's scanline-clip boustrophedon (stdlib only, no pyproj)
#   "greedy"   — Teammate's shapely strip + greedy nearest-end ordering
LAWNMOWER_ALGORITHM = "scanline"

# ═══════════════════════════════════════════════════════════════
#  First flight day — square box pattern
# ═══════════════════════════════════════════════════════════════
# Set FLIGHT_DAY_SIMPLE = True to replace the lawnmower pattern
# with a 30×30 m square box centred on the take-off location.
# The centre is derived from the drone's GPS position at the time
# GENERATE_PATTERN runs.  Set to False to use the full lawnmower.

FLIGHT_DAY_SIMPLE   = False
SQUARE_BOX_SIDE_M   = 30       # Side length of the test box (metres)

# ── Flight day 1 predefined waypoints ──────────────────────────
# Manually surveyed in Mission Planner to avoid buildings at Fenswood.
# Set these before flight day by placing waypoints on the Mission
# Planner map and reading off the lat/lon from the waypoint table.
#
# Orientation guide: place WP1 at the takeoff corner, then build
# the square outward so no leg crosses a building.  The drone flies
# WP1 → WP2 → WP3 → WP4 → WP1 (closing the loop).
#
# Altitude is always SEARCH_ALT_M — do not specify it here.
#
# Leave as an empty list [] to fall back to GPS-centred square box.

PREDEFINED_WAYPOINTS = [
    # (51.422500, -2.671000),   # WP1 — takeoff corner  (fill in before flight)
    # (51.422500, -2.670600),   # WP2
    # (51.422770, -2.670600),   # WP3
    # (51.422770, -2.671000),   # WP4
    # (51.422500, -2.671000),   # WP5 — close loop (repeat WP1)
]

# ═══════════════════════════════════════════════════════════════
#  Map tile for pattern preview
# ═══════════════════════════════════════════════════════════════
# Geo-referenced OSM tile covering Fenswood Farm area.
# Run fetch_map.py once on a machine with internet to download
# real satellite tiles.  The placeholder works for geometry-only
# preview until then.

MAP_TILE_PATH = _os.path.join(_PROJECT_DIR, "fenswood_map.png")

# Exact geo-bounds of the 5×5 tile grid (zoom 17, OSM)
MAP_BOUNDS_NORTH = 51.4266144971
MAP_BOUNDS_SOUTH = 51.4180509986
MAP_BOUNDS_WEST  = -2.6751708984
MAP_BOUNDS_EAST  = -2.6614379883

# Preview output path (served by Flask)
PATTERN_PREVIEW_PATH = _os.path.join(_PROJECT_DIR, "sar_pattern_preview.png")
#PATTERN_PREVIEW_PATH = "/tmp/sar_pattern_preview.png"

# ═══════════════════════════════════════════════════════════════
#  Pre-AUTO check thresholds
# ═══════════════════════════════════════════════════════════════

MIN_GPS_SATELLITES = 8          # Minimum satellite count
MIN_GPS_FIX_TYPE = 3            # 3 = 3D fix (minimum for reliable nav)
MIN_BATTERY_PERCENT = 40        # Don't start search below this %
ALTITUDE_TOLERANCE_M = 5.0      # Altitude must be within ±this of SEARCH_ALT_M
HEARTBEAT_MISS_LIMIT = 5        # Consecutive missed heartbeats before flagging

# ═══════════════════════════════════════════════════════════════
#  Flight speed profile
# ═══════════════════════════════════════════════════════════════
# Speed (m/s) applied at each phase of the mission.
# ArduPilot default WPNAV_SPEED is typically 5 m/s.
#
#  TRANSIT_SPEED_MPS    — SSSI safe corridor to/from search area
#  SEARCH_SPEED_MPS     — search pattern legs
#  FOCUS_SPEED_MPS      — PLB focus area lawnmower
#  APPROACH_SPEED_MPS   — slow approach to landing point after detection
#
# WP_SPEED_SECOND_PASS_MPS is kept for backward compatibility
# (second pass of the initial search pattern uses SEARCH_SPEED_MPS).

TRANSIT_SPEED_MPS        = 8.0   # SSSI corridor (outbound — to search area)
RTL_SPEED_MPS            = 8.0   # SSSI corridor (return home after delivery)
SEARCH_SPEED_MPS         = 4.0   # Search pattern (first and second pass)
FOCUS_SPEED_MPS          = 3.0   # PLB focus area
APPROACH_SPEED_MPS       = 2.0   # Post-detection approach to casualty

WP_SPEED_SECOND_PASS_MPS = SEARCH_SPEED_MPS   # kept for compatibility

# ═══════════════════════════════════════════════════════════════
#  PLB Focus Area — Step 6
# ═══════════════════════════════════════════════════════════════
# Directory where the operator drops the PLB Focus Area KML file.
# The state machine picks up the first .kml file it finds here.
# After processing the file is moved/renamed to avoid re-reads.

PLB_KML_DIR = _os.path.expanduser("~/plb_focus")

# Focus pattern parameters (passed to teammate's waypoint_generator)
FOCUS_SWATH_WIDTH_M  = 5.0      # Strip width for the focus lawnmower (metres)
                                # Camera footprint at 10m AGL: 9.2m × 6.6m
                                # (measured 0.92m × 0.66m at 1m altitude)
                                # Using 5.0m ≈ 75% of short axis — ~20% sidelap
FOCUS_SCAN_ANGLE_DEG = -45      # Scan line rotation (degrees)
FOCUS_ALT_M          = 15       # AGL altitude for focus waypoints (metres)

# ═══════════════════════════════════════════════════════════════
#  Hardcoded waypoints — Flight Day
# ═══════════════════════════════════════════════════════════════
# Each list is [(lat, lon), ...].  Altitude comes from SEARCH_ALT_M.
#
# SEARCH_WAYPOINTS:      The lawnmower/search pattern itself.
# SSSI_NAV_TO_SEARCH:    Safe corridor from takeoff to the first
#                         search waypoint (avoids SSSI no-fly zone).
# SSSI_NAV_TO_HOME:      Safe corridor from the search area back to
#                         the takeoff/home point (avoids SSSI).
#
# The full outbound mission uploaded to the autopilot will be:
#     SSSI_NAV_TO_SEARCH + SEARCH_WAYPOINTS
# so the drone flies the safe corridor then continues into the search.
#
# SAFE_RTL uses SSSI_NAV_TO_HOME when the drone needs to return.
#
# Leave any list empty [] to skip that segment.

SEARCH_WAYPOINTS = [
    (51.42413284, -2.66885908),   # WP1
    (51.42351517, -2.67112058),   # WP2
    (51.42331513, -2.67087753),   # WP3
    (51.42294165, -2.67003302),   # WP4
    (51.42339851, -2.66830482),   # WP5
    (51.42401931, -2.66877339),   # WP6
    (51.42346411, -2.67080618),   # WP7
    (51.42340627, -2.67073591),   # WP8
    (51.42308444, -2.67000820),   # WP9
    (51.42346309, -2.66857588),   # WP10
    (51.42385545, -2.66887202),   # WP11
    (51.42342795, -2.67043723),   # WP12
    (51.42322724, -2.66998338),   # WP13
    (51.42352767, -2.66884693),   # WP14
    (51.42369158, -2.66897064),   # WP15
    (51.42340200, -2.67003087),   # WP16
]

SSSI_NAV_TO_SEARCH = [
    # Outbound: takeoff → around SSSI → into search area
    (51.4234034, -2.6713890),   # NAV1 — near takeoff
    (51.4220921, -2.6697958),   # NAV2
    (51.4226507, -2.6675588),   # NAV3
    (51.4234937, -2.6681381),   # NAV4 — entry to search area
]

SSSI_NAV_TO_HOME = [
    # Inbound: search area → around SSSI → home (reversed corridor)
    (51.4234937, -2.6681381),   # NAV4 — exit search area
    (51.4226507, -2.6675588),   # NAV3
    (51.4220921, -2.6697958),   # NAV2
    (51.4234034, -2.6713890),   # NAV1 — near home/takeoff
]

# ═══════════════════════════════════════════════════════════════
#  CV Detection monitoring — FOCUS state
# ═══════════════════════════════════════════════════════════════
# Directory where the CV pipeline drops detection images.
# Filenames encode the geotag: e.g. "51.4241_-2.6714.jpg"
# FOCUS state watches this directory for new files.

DETECTION_DIR = _os.path.expanduser("~/dima/Group_Proj/detections")

# ═══════════════════════════════════════════════════════════════
#  Satellite background for landing canvas
# ═══════════════════════════════════════════════════════════════
# A screenshot from Google Earth covering the search area.
# The GUI overlays the PLB polygon, detection marker, and NESW
# landing options on top of this image.
# Bounds are the GPS coordinates of the image edges.
#
# To produce: open Google Earth, zoom to cover the full search
# area, screenshot, and note the lat/lon of the image corners.

SATELLITE_IMAGE = _os.path.join(_PROJECT_DIR, "satellite_bg.png")

SATELLITE_BOUNDS_NORTH = 51.42510754
SATELLITE_BOUNDS_SOUTH = 51.42090293
SATELLITE_BOUNDS_WEST  = -2.67524776
SATELLITE_BOUNDS_EAST  = -2.66256298

# ═══════════════════════════════════════════════════════════════
#  Landing standoff radius
# ═══════════════════════════════════════════════════════════════
# Distance (metres) from the detected casualty at which the NESW
# landing points are placed.  The drone will fly to whichever
# cardinal point the operator selects.

LANDING_STANDOFF_M = 7.5

# ═══════════════════════════════════════════════════════════════
#  Ground truth dummy location (fallback for inaccurate geotag)
# ═══════════════════════════════════════════════════════════════
# If the CV geotag is not accurate, the operator can use this
# known-good position instead.  Set to (lat, lon) to enable
# an orange "Use Ground Truth" button on the landing selection
# screen.  Leave as None to hide it.
#
# GROUND_TRUTH_LOCATION = (51.42340, -2.66950)   # example
GROUND_TRUTH_LOCATION =  (51.423527, -2.670937)