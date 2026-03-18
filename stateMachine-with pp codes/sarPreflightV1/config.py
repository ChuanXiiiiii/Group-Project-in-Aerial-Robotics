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
    #"udp:192.168.1.22:14550",     # ← Change to your Windows VM IP
    #"udp:127.0.0.1:14551",         # ← State machine connects here
    "tcp:127.0.0.1:5762",
]

# ═══════════════════════════════════════════════════════════════
#  State machine connection
# ═══════════════════════════════════════════════════════════════

#CONNECTION_STRING = "udp:127.0.0.1:14551"
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
#  Trigger file (still works alongside the GUI)
# ═══════════════════════════════════════════════════════════════

TRIGGER_FILE = "/tmp/sar_trigger.txt"

# ═══════════════════════════════════════════════════════════════
#  KML data file
# ═══════════════════════════════════════════════════════════════

KML_FILE = "AENGM0074.kml"       # Path to mission KML (relative to project root)

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
#  Search pattern generation (Step 4)
# ═══════════════════════════════════════════════════════════════
# These are placeholder defaults.  Your CV teammate should
# replace them with values derived from camera FOV, GSD, and
# overlap requirements.

SWATH_WIDTH_M = 10       # Spacing between parallel scan lines (metres)
SEARCH_ALT_M  = 30       # AGL altitude for search waypoints (metres)
SCAN_HEADING_DEG = 0     # 0 = scan lines run East–West, 90 = North–South
PATTERN_ALGORITHM = 1    # 0 = lawnmower, 1 = perimeter_spiral

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

# ═══════════════════════════════════════════════════════════════
#  Preflight check thresholds (Step 5)
# ═══════════════════════════════════════════════════════════════

MIN_GPS_SATELLITES = 8          # Minimum satellite count for GPS check
MIN_GPS_FIX_TYPE = 3            # 3 = 3D fix (minimum for reliable nav)
MIN_BATTERY_PERCENT = 40        # Don't start search below this %
ALTITUDE_TOLERANCE_M = 5.0      # Altitude must be within ±this of SEARCH_ALT_M
HEARTBEAT_MISS_LIMIT = 5        # Consecutive missed heartbeats before flagging