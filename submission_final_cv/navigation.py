"""MAVLink navigation commands for ArduCopter."""

import time
import math

from pymavlink import mavutil


COPTER_MODES = {
    "STABILIZE": 0,
    "ACRO":      1,
    "ALT_HOLD":  2,
    "AUTO":      3,
    "GUIDED":    4,
    "LOITER":    5,
    "RTL":       6,
    "CIRCLE":    7,
    "LAND":      9,
    "DRIFT":    11,
    "SPORT":    13,
    "POSHOLD":  16,
    "BRAKE":    17,
}


class NavigationController:
    """Thin wrapper around pymavlink navigation commands."""

    def __init__(self, master, no_turn=False, get_yaw=None):
        self.master = master
        self.no_turn = no_turn
        self._get_yaw = get_yaw
        self.last_speed_req = 0.0

    # ── Position commands ────────────────────────────────────────────

    def send_global_target(self, lat, lon, alt, yaw=None, vz=0):
        """Fly to (lat, lon, alt) via SET_POSITION_TARGET_GLOBAL_INT.

        vz is a vertical velocity hint in NED (negative=climb, positive=descend),
        used as feed-forward so small altitude changes don't rely on P-gain alone.
        """
        # sanity checks on the target before we send anything
        if math.isnan(lat) or math.isnan(lon) or math.isnan(alt):
            print(f"WARNING: NaN in target position (lat={lat}, lon={lon}, alt={alt}) , skipping")
            return
        if lat < -90 or lat > 90 or lon < -180 or lon > 180:
            print(f"WARNING: GPS out of bounds (lat={lat}, lon={lon}) , skipping")
            return
        # altitude hard cap at 50 m (matches flight area limit)
        if alt < 0 or alt > 50:
            print(f"WARNING: altitude {alt}m exceeds 50m limit, capping to 50m")
            alt = min(alt, 50)

        if yaw is None and self.no_turn and self._get_yaw is not None:
            yaw = self._get_yaw()

        # Type mask: position always used. Enable vz when non-zero for altitude feed-forward.
        #   Bits: 0=x 1=y 2=z 3=vx 4=vy 5=vz 6-8=accel 9=force 10=yaw 11=yaw_rate
        #   0 = use, 1 = ignore
        if yaw is not None:
            #              bit: 11 10 9 8 7 6  5  4  3  2 1 0
            # pos+yaw:          1  0 1 1 1 1  1  1  1  0 0 0 = 0b101111111000
            # pos+vz+yaw:       1  0 1 1 1 1  0  1  1  0 0 0 = 0b101111011000
            mask = 0b101111011000 if vz != 0 else 0b101111111000
            self.master.mav.set_position_target_global_int_send(
                0, self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mask, int(lat * 1e7), int(lon * 1e7), alt,
                0, 0, vz, 0, 0, 0, yaw, 0)
        else:
            #              bit: 11 10 9 8 7 6  5  4  3  2 1 0
            # pos only:          1  1 1 1 1 1  1  1  1  0 0 0 = 0b111111111000
            # pos+vz:            1  1 1 1 1 1  0  1  1  0 0 0 = 0b111111011000
            mask = 0b111111011000 if vz != 0 else 0b111111111000
            self.master.mav.set_position_target_global_int_send(
                0, self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mask, int(lat * 1e7), int(lon * 1e7), alt,
                0, 0, vz, 0, 0, 0, 0, 0)

    def send_velocity(self, vx, vy, vz, yaw_rate=0, current_yaw=None):
        """Body-frame velocity command, rotated to NED before sending."""
        if not self.master:
            return
        if current_yaw is None:
            current_yaw = self._get_yaw() if self._get_yaw is not None else 0.0
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        vx_ned = vx*cos_yaw - vy*sin_yaw
        vy_ned = vx*sin_yaw + vy*cos_yaw
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b010111000111, 0, 0, 0,
            vx_ned, vy_ned, vz, 0, 0, 0, 0, math.radians(yaw_rate))

    # ── Speed ────────────────────────────────────────────────────────

    def set_speed(self, speed_mps):
        """DO_CHANGE_SPEED, throttled to once per 3 s."""
        if time.time() - self.last_speed_req < 3.0:
            return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
            1, speed_mps, -1, 0, 0, 0, 0)
        self.last_speed_req = time.time()

    # ── Arm / Takeoff / Land ─────────────────────────────────────────

    def request_arm(self):
        """Send arm; returns COMMAND_ACK result or None on timeout."""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        # FIXME: 1s blocking timeout can race with other messages on the link
        if ack:
            return ack.result
        return None

    def request_takeoff(self, alt):
        """MAV_CMD_NAV_TAKEOFF to the given altitude (AGL, metres)."""
        if alt <= 0:
            print(f"WARNING: invalid takeoff altitude ({alt}m), must be positive")
            return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt)

    def send_land(self, lat=0, lon=0):
        """MAV_CMD_NAV_LAND; (0, 0) = land at current position."""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0,
            lat, lon, 0)

    # ── Mode changes ─────────────────────────────────────────────────

    def set_mode(self, mode_name):
        """Set flight mode by name."""
        key = mode_name.upper()
        if key not in COPTER_MODES:
            raise ValueError(
                f"Unknown ArduCopter mode '{mode_name}'. "
                f"Known modes: {', '.join(sorted(COPTER_MODES))}")
        mode_id = COPTER_MODES[key]
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0)
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if ack:
            return ack.result
        return None

    # ── Data streams ─────────────────────────────────────────────────

    def request_data_stream(self, rate_hz=10):
        """Request all MAVLink data streams at rate_hz."""
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz, 1)
