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
    #udp:192.168.1.60:14550",     # ← Change to your Windows VM IP
    #udp:127.0.0.1:14550",         # ← State machine connects here
    "tcp:127.0.0.1:5762"
]

# ═══════════════════════════════════════════════════════════════
#  State machine connection
# ═══════════════════════════════════════════════════════════════

#CONNECTION_STRING = "udp:127.0.0.1:14550"
CONNECTION_STRING = "tcp:127.0.0.1:5762"
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
LOG_DIR  = _os.path.expanduser("~/sar_logs")   # ~/sar_logs on Pi, adjust for Windows
LOG_FILE = None    # Set at runtime by mission_log.py — do not edit manually

# ═══════════════════════════════════════════════════════════════
#  KML data file
# ═══════════════════════════════════════════════════════════════

KML_FILE = "AENGM0074.kml"       # Path to mission KML (relative to project root)

# ═══════════════════════════════════════════════════════════════
#  Fence configuration
# ═══════════════════════════════════════════════════════════════

# FENCE_ENABLE: 1 = enabled, 0 = disabled
FENCE_ENABLE_VALUE = 0

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
# boundary before triggering a breach
FENCE_MARGIN_VALUE = 2

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

SWATH_WIDTH_M = 10       # Spacing between parallel scan lines (metres)
SEARCH_ALT_M  = 30       # AGL altitude for search waypoints (metres)
SCAN_HEADING_DEG = 0     # 0 = scan lines run East–West, 90 = North–South

# ═══════════════════════════════════════════════════════════════
#  First flight day — square box pattern
# ═══════════════════════════════════════════════════════════════
# Set FLIGHT_DAY_SIMPLE = True to replace the lawnmower pattern
# with a 30×30 m square box centred on the take-off location.
# The centre is derived from the drone's GPS position at the time
# GENERATE_PATTERN runs.  Set to False to use the full lawnmower.

FLIGHT_DAY_SIMPLE   = True
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
     (51.4233, -2.67145),   # WP1 — takeoff corner  (fill in before flight)
     (51.42278, -2.67110),   # WP2
     (51.42289, -2.67057),   # WP3
     (51.423169, -2.6708),   # WP4
     (51.42305, -2.67128),   # WP5 — close loop (repeat WP1)
]

# ═══════════════════════════════════════════════════════════════
#  Map tile for pattern preview
# ═══════════════════════════════════════════════════════════════
# Geo-referenced OSM tile covering Fenswood Farm area.
# Run fetch_map.py once on a machine with internet to download
# real satellite tiles.  The placeholder works for geometry-only
# preview until then.

MAP_TILE_PATH = "fenswood_map.png"

# Exact geo-bounds of the 5×5 tile grid (zoom 17, OSM)
MAP_BOUNDS_NORTH = 51.4266144971
MAP_BOUNDS_SOUTH = 51.4180509986
MAP_BOUNDS_WEST  = -2.6751708984
MAP_BOUNDS_EAST  = -2.6614379883

# Preview output path (served by Flask)
PATTERN_PREVIEW_PATH = "sar_pattern_preview.png"
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
#  Second-pass search speed
# ═══════════════════════════════════════════════════════════════
# Speed in m/s for the second search pass.  ArduPilot default is
# WPNAV_SPEED (typically 5 m/s).  Reduce for a slower, more
# thorough second sweep.

WP_SPEED_SECOND_PASS_MPS = 2.0
