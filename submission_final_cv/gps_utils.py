"""Shared GPS math.

Pure functions, no dependencies on mavlink / vision / config objects.
All inputs passed explicitly. WGS-84 Earth radius for all projections.
"""

import math

R_EARTH = 6378137.0  # WGS-84 equatorial radius (m)


def gps_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two lat/lon points in metres (Haversine)."""
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(d_lon / 2) ** 2)
    return R_EARTH * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def calculate_target_from_pixels(
    u: float,
    v: float,
    drone_alt: float,
    drone_yaw: float,
    drone_lat: float,
    drone_lon: float,
    image_w: int,
    image_h: int,
    sensor_width_mm: float,
    focal_length_mm: float,
    verbose: bool = False,
    drone_roll: float = 0.0,
    drone_pitch: float = 0.0,
) -> tuple[float, float]:
    """Project a pixel detection (u, v) to an estimated ground GPS point.

    Pipeline:
        1. GSD (metres / pixel) = (sensor_width_mm * alt) / (focal_length_mm * image_w)
        2. Pixel offset from image center -> camera-frame metres (fwd = -dy, right = +dx)
        3. Rotate by drone yaw -> North/East offsets
        4. Small-angle spherical projection -> lat/lon deltas

    drone_roll / drone_pitch are only reported in verbose mode, not compensated.
    """
    cx = image_w / 2
    cy = image_h / 2

    # ground sample distance at this altitude
    gsd_m = (sensor_width_mm * drone_alt) / (focal_length_mm * image_w)

    # offsets from image centre
    delta_x_px = u - cx
    delta_y_px = v - cy

    # camera-frame: "forward" is up in the image, so -dy
    fwd_m = -delta_y_px * gsd_m
    right_m = delta_x_px * gsd_m

    # rotate by yaw into NED
    yaw_deg = math.degrees(drone_yaw)
    offset_n = fwd_m * math.cos(drone_yaw) - right_m * math.sin(drone_yaw)
    offset_e = fwd_m * math.sin(drone_yaw) + right_m * math.cos(drone_yaw)

    # metre offsets -> degree deltas
    d_lat = (offset_n / R_EARTH) * (180 / math.pi)
    d_lon = (offset_e / (R_EARTH * math.cos(math.radians(drone_lat)))) * (180 / math.pi)

    est_lat = drone_lat + d_lat
    est_lon = drone_lon + d_lon

    if verbose:
        dist_m = math.sqrt(offset_n ** 2 + offset_e ** 2)
        roll_deg = math.degrees(drone_roll)
        pitch_deg = math.degrees(drone_pitch)
        tilt_offset_m = drone_alt * math.sqrt(
            math.tan(drone_pitch) ** 2 + math.tan(drone_roll) ** 2
        ) if (abs(drone_pitch) > 0.001 or abs(drone_roll) > 0.001) else 0.0
        sign = lambda x: "+" if x >= 0 else ""
        print(f"[GPS-EST] Detection at pixel ({u:.0f}, {v:.0f}) in {image_w}x{image_h} frame")
        print(f"  Pixel offset from center: dx={sign(delta_x_px)}{delta_x_px:.0f}px, "
              f"dy={sign(delta_y_px)}{delta_y_px:.0f}px")
        print(f"  GSD = ({sensor_width_mm}mm x {drone_alt:.1f}m) / "
              f"({focal_length_mm}mm x {image_w}px) = {gsd_m:.5f} m/px")
        print(f"  Camera-frame offset: right={sign(right_m)}{right_m:.2f}m, "
              f"forward={sign(fwd_m)}{fwd_m:.2f}m")
        print(f"  Drone yaw: {yaw_deg:.1f} deg -> Rotate to NED:")
        print(f"    North = {fwd_m:+.2f}*cos({yaw_deg:.1f}) - {right_m:+.2f}*sin({yaw_deg:.1f})"
              f" = {offset_n:+.2f}m")
        print(f"    East  = {fwd_m:+.2f}*sin({yaw_deg:.1f}) + {right_m:+.2f}*cos({yaw_deg:.1f})"
              f" = {offset_e:+.2f}m")
        print(f"  Drone GPS: ({drone_lat:.6f}, {drone_lon:.6f}) alt={drone_alt:.1f}m")
        print(f"  Target GPS: ({est_lat:.6f}, {est_lon:.6f})")
        print(f"  Distance from drone: {dist_m:.2f}m")
        if abs(drone_pitch) > 0.001 or abs(drone_roll) > 0.001:
            print(f"  Roll: {roll_deg:+.1f} deg  Pitch: {pitch_deg:+.1f} deg"
                  f" -> tilt offset: {tilt_offset_m:.1f}m (NOT compensated)")
        else:
            print(f"  Roll: {roll_deg:+.1f} deg  Pitch: {pitch_deg:+.1f} deg -> level")

    return est_lat, est_lon


def offset_gps_by_distance(
    lat: float, lon: float, north_m: float, east_m: float
) -> tuple[float, float]:
    """Offset (lat, lon) by a North/East metre displacement. Returns (new_lat, new_lon)."""
    d_lat = (north_m / R_EARTH) * (180 / math.pi)
    d_lon = (east_m / (R_EARTH * math.cos(math.radians(lat)))) * (180 / math.pi)
    return lat + d_lat, lon + d_lon


def landing_offset_7_5m(
    target_lat: float, target_lon: float, direction: str
) -> tuple[float, float]:
    """Landing point 7.5 m from target in a cardinal direction ('n'/'s'/'e'/'w')."""
    offset_dist = 7.5  # metres
    d_lat = 0.0
    d_lon = 0.0

    d = direction.lower()
    if d == 'n':
        d_lat = (offset_dist / R_EARTH) * (180 / math.pi)
    elif d == 's':
        d_lat = -(offset_dist / R_EARTH) * (180 / math.pi)
    elif d == 'e':
        d_lon = (offset_dist / (R_EARTH * math.cos(math.radians(target_lat)))) * (180 / math.pi)
    elif d == 'w':
        d_lon = -(offset_dist / (R_EARTH * math.cos(math.radians(target_lat)))) * (180 / math.pi)
    # unknown direction: no-op (fall through with zero offset)

    return target_lat + d_lat, target_lon + d_lon
