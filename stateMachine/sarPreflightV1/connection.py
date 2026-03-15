"""
connection.py — MAVLink connection and flight-mode helpers.
===========================================================
Shared resource used by all states.
Now also updates SharedStatus so the GUI stays in sync.
"""

import time
from pymavlink import mavutil
from config import FLIGHT_MODES, MODE_NAMES, MODE_CHANGE_TIMEOUT


class DroneConnection:

    def __init__(self, connection_string, baud, shared_status=None):
        self.connection_string = connection_string
        self.baud = baud
        self.conn = None
        self.status = shared_status     # may be None if no GUI

    def connect(self):
        self.log("INIT", f"Connecting to {self.connection_string} ...")
        self.conn = mavutil.mavlink_connection(
            self.connection_string, baud=self.baud
        )

        # Wait specifically for the AUTOPILOT heartbeat.
        # Multiple sources send heartbeats on this link:
        #   sys=1   comp=1  type=2   → autopilot (quadrotor) ← want this
        #   sys=1   comp=0  type=27  → ADSB peripheral       ← skip
        #   sys=255 comp=0  type=6   → MAVProxy GCS          ← skip
        # We look for component 1 (MAV_COMP_ID_AUTOPILOT1) specifically.
        self.log("INIT", "Waiting for autopilot heartbeat ...")
        while True:
            hb = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
            if hb is None:
                self.log("INIT", "No heartbeat yet, retrying ...")
                continue

            # Component 1 = MAV_COMP_ID_AUTOPILOT1 — the flight controller
            if hb.get_srcComponent() != 1:
                continue

            # Found the autopilot — lock on
            self.conn.target_system = hb.get_srcSystem()
            self.conn.target_component = hb.get_srcComponent()
            break

        self.log("INIT", f"Connected to vehicle (system={self.conn.target_system}, "
                         f"component={self.conn.target_component}, "
                         f"type={hb.type})")
        self.log("INIT", f"Filtering: only accepting heartbeats from "
                         f"sys={self.conn.target_system} comp={self.conn.target_component}")

        if self.status:
            self.status.set_connected(True)

    # ── Read current mode ──────────────────────────────────────

    def get_mode(self, retries=3):
        """
        Read current flight mode from the autopilot's heartbeat only.

        Multiple components on the same system send heartbeats:
            sys=1 comp=1 type=2  → autopilot (quadrotor) ← this is the one we want
            sys=1 comp=0 type=27 → ADSB peripheral        ← ignore

        We filter on both system AND component to only read the autopilot.
        Retries up to `retries` times before giving up with UNKNOWN.
        """
        for attempt in range(1, retries + 1):
            deadline = time.time() + 3

            while time.time() < deadline:
                hb = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if hb is None:
                    continue

                src_sys = hb.get_srcSystem()
                src_comp = hb.get_srcComponent()

                # Only accept heartbeats from the autopilot component
                if src_sys != self.conn.target_system:
                    continue
                if src_comp != self.conn.target_component:
                    continue

                mode_num = hb.custom_mode
                mode_name = MODE_NAMES.get(mode_num, f"MODE_{mode_num}")

                if self.status:
                    self.status.set_mode(mode_name)

                return mode_name, mode_num

            # Didn't catch an autopilot heartbeat in this window
            if attempt < retries:
                self.log("MODE", f"No autopilot heartbeat (attempt {attempt}/{retries}), retrying ...")

        self.log("MODE", f"WARNING: No autopilot heartbeat after {retries} attempts — returning UNKNOWN")
        return "UNKNOWN", -1

    # ── Change mode ────────────────────────────────────────────

    def set_mode(self, mode_name):
        if mode_name not in FLIGHT_MODES:
            self.log("MODE", f"ERROR: Unknown mode '{mode_name}'")
            return False

        target_id = FLIGHT_MODES[mode_name]
        current_name, current_id = self.get_mode()

        if current_id == target_id:
            self.log("MODE", f"Already in {mode_name}")
            return True

        self.log("MODE", f"{current_name} → {mode_name} ...")

        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            target_id,
        )

        start = time.time()
        while time.time() - start < MODE_CHANGE_TIMEOUT:
            hb = self.conn.recv_match(
                type="HEARTBEAT", blocking=True, timeout=1
            )
            if hb is None:
                continue
            # Only accept autopilot heartbeats (match system AND component)
            if hb.get_srcSystem() != self.conn.target_system:
                continue
            if hb.get_srcComponent() != self.conn.target_component:
                continue
            if hb.custom_mode == target_id:
                self.log("MODE", f"Confirmed: {mode_name}")
                if self.status:
                    self.status.set_mode(mode_name)
                return True

        self.log("MODE", f"TIMEOUT waiting for {mode_name}")
        return False

    # ── Mission upload ────────────────────────────────────────

    def upload_mission(self, waypoints, timeout=15):
        """
        Upload a list of waypoints to the autopilot using the
        MAVLink mission protocol handshake:

            1. We send MISSION_COUNT (n items)
            2. Autopilot requests each item via MISSION_REQUEST_INT
            3. We reply with the corresponding MISSION_ITEM_INT
            4. Autopilot confirms with MISSION_ACK

        waypoints: list of dicts, each with "lat", "lon", "alt" keys.
                   A home item (seq=0) is prepended automatically.
        Returns True on ACK, False on timeout or error.
        """
        # ── Build the full item list ──
        # Seq 0 = home position placeholder (ArduPilot overwrites on arm)
        # Seq 1+ = actual waypoints
        items = []

        # Home item (seq 0) — coordinates don't matter, ArduPilot
        # replaces them with the actual home on arming.
        items.append({
            "seq": 0,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "lat": 0, "lon": 0, "alt": 0,
        })

        for i, wp in enumerate(waypoints):
            items.append({
                "seq": i + 1,
                "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                "lat": float(wp["lat"]),
                "lon": float(wp["lon"]),
                "alt": float(wp["alt"]),
            })

        count = len(items)
        self.log("MISSION", f"Uploading {count} items ({count - 1} waypoints + home) ...")

        # ── Step 1: Send MISSION_COUNT ──
        self.conn.mav.mission_count_send(
            self.conn.target_system,
            self.conn.target_component,
            count,
        )

        # ── Step 2 & 3: Respond to MISSION_REQUEST_INT messages ──
        sent = set()
        deadline = time.time() + timeout

        while time.time() < deadline:
            msg = self.conn.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True,
                timeout=2,
            )

            if msg is None:
                continue

            msg_type = msg.get_type()

            # ── Step 4: ACK received — we're done ──
            if msg_type == "MISSION_ACK":
                result = msg.type
                if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    self.log("MISSION", f"Upload complete — ACK received ({count - 1} waypoints)")
                    return True
                else:
                    self.log("MISSION", f"Upload REJECTED — ACK type {result}")
                    return False

            # ── Item request — send the requested item ──
            if msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                seq = msg.seq
                if seq < 0 or seq >= count:
                    self.log("MISSION", f"ERROR: autopilot requested seq={seq}, "
                             f"but we only have 0..{count - 1}")
                    return False

                item = items[seq]
                self.conn.mav.mission_item_int_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    item["seq"],
                    item["frame"],
                    item["command"],
                    0,                          # current (0 = not current)
                    1,                          # autocontinue
                    0, 0, 0, 0,                 # params 1-4 (unused for NAV_WAYPOINT)
                    int(item["lat"] * 1e7),     # lat  (degE7)
                    int(item["lon"] * 1e7),     # lon  (degE7)
                    item["alt"],                # alt  (metres, relative)
                )

                sent.add(seq)
                if seq == 0:
                    desc = "HOME (placeholder)"
                else:
                    desc = f"{item['lat']:.6f}, {item['lon']:.6f} @ {item['alt']}m"
                self.log("MISSION", f"  Sent item seq={seq}: {desc}")

        self.log("MISSION", "Upload TIMEOUT — no ACK received")
        return False

    # ── Fence upload ─────────────────────────────────────────

    def upload_fence(self, polygons, timeout=15):
        """
        Upload a list of fence polygons to the autopilot using the
        MAVLink fence protocol (MAV_MISSION_TYPE_FENCE).

        IMPORTANT: Each upload REPLACES the entire fence list on the
        Cube. There is no append operation. To have multiple fences
        active simultaneously, ALL polygons must be included in a
        single upload.

        polygons: list of dicts, each with:
            "type":   "inclusion" or "exclusion"
            "points": [(lat, lon), ...]
            "label":  string (for logging)

        Each polygon's vertices use:
            command = MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION (5001)
                   or MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION (5002)
            param1  = vertex count of THIS polygon (not total count)

        Returns True on ACK, False on timeout or error.
        """
        # ── Build flat item list from polygon groups ──
        items = []
        for poly in polygons:
            fence_type = poly["type"]
            points = poly["points"]
            label = poly.get("label", "unnamed")
            vertex_count = len(points)

            if fence_type == "inclusion":
                cmd = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
            else:
                cmd = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION

            self.log("FENCE", f"  {label}: {fence_type}, {vertex_count} vertices")

            for lat, lon in points:
                items.append({
                    "command": cmd,
                    "vertex_count": vertex_count,
                    "lat": float(lat),
                    "lon": float(lon),
                })

        count = len(items)
        self.log("FENCE", f"Uploading {count} fence items ...")

        # ── Step 1: Send MISSION_COUNT with fence type ──
        self.conn.mav.mission_count_send(
            self.conn.target_system,
            self.conn.target_component,
            count,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
        )

        # ── Step 2 & 3: Respond to requests ──
        sent = set()
        deadline = time.time() + timeout

        while time.time() < deadline:
            msg = self.conn.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True,
                timeout=2,
            )
            if msg is None:
                continue

            msg_type = msg.get_type()

            # ── Step 4: ACK ──
            if msg_type == "MISSION_ACK":
                # Only process fence ACKs
                if hasattr(msg, "mission_type"):
                    if msg.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                        continue

                result = msg.type
                if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    self.log("FENCE", f"Upload complete — ACK received "
                             f"({count} items)")
                    return True
                else:
                    self.log("FENCE", f"Upload REJECTED — ACK type {result}")
                    return False

            # ── Item request ──
            if msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                # Only respond to fence requests
                if hasattr(msg, "mission_type"):
                    if msg.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                        continue

                seq = msg.seq
                if seq < 0 or seq >= count:
                    self.log("FENCE", f"ERROR: requested seq={seq}, "
                             f"but we only have 0..{count - 1}")
                    return False

                item = items[seq]
                self.conn.mav.mission_item_int_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    item["command"],
                    0,                              # current
                    0,                              # autocontinue
                    item["vertex_count"],            # param1 = vertex count of THIS polygon
                    0, 0, 0,                        # params 2-4
                    int(item["lat"] * 1e7),
                    int(item["lon"] * 1e7),
                    0,                              # z (altitude — unused for fence)
                    mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                )
                sent.add(seq)
                self.log("FENCE", f"  Sent item seq={seq}: "
                         f"{item['lat']:.6f}, {item['lon']:.6f}")

        self.log("FENCE", "Upload TIMEOUT — no ACK received")
        return False

    # ── Parameter setting ──────────────────────────────────────

    def set_parameter(self, param_name, value, param_type=None, retries=3):
        """
        Set an ArduPilot parameter via MAVLink.

        param_name: string parameter name (e.g. "FENCE_ENABLE")
        value: numeric value to set
        param_type: MAV_PARAM_TYPE (default: REAL32)
        retries: number of attempts

        Returns True if PARAM_VALUE echoed back confirms the change.
        """
        if param_type is None:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        for attempt in range(1, retries + 1):
            self.log("PARAM", f"Setting {param_name} = {value} "
                     f"(attempt {attempt}/{retries})")

            # Pad param name to 16 bytes as MAVLink requires
            param_id = param_name.encode("utf-8").ljust(16, b"\x00")

            self.conn.mav.param_set_send(
                self.conn.target_system,
                self.conn.target_component,
                param_id,
                float(value),
                param_type,
            )

            # Wait for PARAM_VALUE response
            deadline = time.time() + 5
            while time.time() < deadline:
                msg = self.conn.recv_match(
                    type="PARAM_VALUE", blocking=True, timeout=2
                )
                if msg is None:
                    continue

                received_name = msg.param_id
                if isinstance(received_name, bytes):
                    received_name = received_name.decode("utf-8")
                received_name = received_name.rstrip("\x00")

                if received_name == param_name:
                    if abs(msg.param_value - float(value)) < 0.01:
                        self.log("PARAM", f"Confirmed: {param_name} = {msg.param_value}")
                        return True
                    else:
                        self.log("PARAM", f"WARNING: {param_name} echoed "
                                 f"{msg.param_value}, expected {value}")

            self.log("PARAM", f"No confirmation for {param_name} "
                     f"(attempt {attempt})")

        self.log("PARAM", f"FAILED to set {param_name} after {retries} attempts")
        return False

    # ── Logging ────────────────────────────────────────────────

    def get_telemetry(self):
        """
        Read current telemetry snapshot: altitude, GPS, battery.
        Returns a dict using pymavlink's cached messages — does NOT
        consume from the live buffer, so get_mode/set_mode still work.

        The cache is kept fresh by get_mode() which runs before this
        in any state loop.
        """
        telem = {
            "alt": None,
            "lat": None,
            "lon": None,
            "satellites": None,
            "gps_fix": None,
            "battery_v": None,
            "battery_pct": None,
        }

        # Read from pymavlink's cached last-message-per-type dict.
        msgs = self.conn.messages

        gps = msgs.get("GPS_RAW_INT")
        if gps:
            telem["lat"] = gps.lat / 1e7
            telem["lon"] = gps.lon / 1e7
            telem["alt"] = gps.alt / 1000.0     # mm → m
            telem["satellites"] = gps.satellites_visible
            telem["gps_fix"] = gps.fix_type

        pos = msgs.get("GLOBAL_POSITION_INT")
        if pos:
            telem["alt"] = pos.relative_alt / 1000.0   # mm → m (relative to home)

        bat = msgs.get("SYS_STATUS")
        if bat:
            telem["battery_v"] = bat.voltage_battery / 1000.0   # mV → V
            telem["battery_pct"] = bat.battery_remaining         # -1 if unknown

        return telem

    def get_armed(self):
        """
        Check if the vehicle is armed from the last cached heartbeat.
        Returns True if armed, False if disarmed, None if no data.
        """
        hb = self.conn.messages.get("HEARTBEAT")
        if hb is None:
            return None
        # MAV_MODE_FLAG_SAFETY_ARMED = 128 (bit 7)
        return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def query_mission_count(self, timeout=5):
        """
        Actively query the autopilot for how many mission items are stored.
        This is a request-response exchange, not a cache read.

        Returns the item count (int), or 0 on timeout.
        """
        self.conn.mav.mission_request_list_send(
            self.conn.target_system,
            self.conn.target_component,
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.conn.recv_match(
                type="MISSION_COUNT", blocking=True, timeout=2,
            )
            if msg is None:
                continue
            return msg.count
        self.log("QUERY", "Timeout querying mission count")
        return 0

    def query_fence_count(self, timeout=5):
        """
        Actively query the autopilot for how many fence items are stored.
        Returns the item count (int), or 0 on timeout.
        """
        self.conn.mav.mission_request_list_send(
            self.conn.target_system,
            self.conn.target_component,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.conn.recv_match(
                type="MISSION_COUNT", blocking=True, timeout=2,
            )
            if msg is None:
                continue
            # Only accept fence-type responses
            if hasattr(msg, "mission_type"):
                if msg.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                    continue
            return msg.count
        self.log("QUERY", "Timeout querying fence count")
        return 0

    def log(self, state_name, message):
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{state_name}] {message}"
        print(f"  {line}  ({timestamp})")

        if self.status:
            self.status.set_message(f"{line}  ({timestamp})")