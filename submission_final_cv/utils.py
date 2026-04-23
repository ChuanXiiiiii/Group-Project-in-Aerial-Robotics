"""Geo-pixel utilities and image compositing helpers."""

import math
import cv2

import config


class GeoTransformer:
    """Convert between GPS coordinates and map pixel positions.

    Latitude scale uses the WGS-84 oblateness correction:
        lat_m = 111132.954 - 559.822 * cos(2 * lat)
    Longitude scale contracts toward the poles:
        lon_m = 111132.954 * cos(lat)
    """

    def __init__(self, map_w_px: int) -> None:
        self.pix_per_m: float = map_w_px / config.MAP_WIDTH_METERS

    def gps_to_pixels(self, lat: float, lon: float) -> tuple[int, int]:
        """GPS -> (x, y) pixels on the map (origin = top-left = REF_LAT/REF_LON)."""
        lat_m = 111132.954 - 559.822 * math.cos(2 * math.radians(lat))
        lon_m = 111132.954 * math.cos(math.radians(lat))
        dy = -(lat - config.REF_LAT) * lat_m
        dx = (lon - config.REF_LON) * lon_m
        return int(dx * self.pix_per_m), int(dy * self.pix_per_m)

    def pixels_to_gps(self, x: int, y: int) -> tuple[float, float]:
        # inverse of gps_to_pixels; scale factors taken at REF_LAT
        dx = x / self.pix_per_m
        dy = y / self.pix_per_m
        lat_m = 111132.954 - 559.822 * math.cos(2 * math.radians(config.REF_LAT))
        lon_m = 111132.954 * math.cos(math.radians(config.REF_LAT))
        d_lat = -(dy / lat_m)
        d_lon = dx / lon_m
        return config.REF_LAT + d_lat, config.REF_LON + d_lon


def overlay_image_alpha(
    background,
    overlay,
    x: int,
    y: int,
    target_w: int,
    target_h: int,
    rotation_deg: float = 0,
) -> None:
    """Composite a rotated BGRA overlay onto a BGR background (in place).

    If target_w == 0 it is auto-computed from target_h to keep aspect ratio.
    Regions outside the background are clipped silently.
    """
    if overlay is None:
        return
    h_src, w_src = overlay.shape[:2]

    # auto width from height if width is zero
    if target_w == 0:
        scale = target_h / h_src
        target_w = int(w_src * scale)
    if target_w <= 0 or target_h <= 0:
        return

    try:
        resized = cv2.resize(overlay, (target_w, target_h))
    except Exception:
        # TODO: narrow this - bare Exception swallows real bugs
        return

    # pad to bounding diagonal so rotation doesn't clip corners
    diag = int(math.sqrt(target_w ** 2 + target_h ** 2))
    pad_x = (diag - target_w) // 2
    pad_y = (diag - target_h) // 2
    padded = cv2.copyMakeBorder(
        resized, pad_y, pad_y, pad_x, pad_x, cv2.BORDER_CONSTANT, value=(0, 0, 0, 0)
    )

    # rotate about centre
    h_pad, w_pad = padded.shape[:2]
    rot_matrix = cv2.getRotationMatrix2D((w_pad // 2, h_pad // 2), rotation_deg, 1.0)
    rotated = cv2.warpAffine(padded, rot_matrix, (w_pad, h_pad))

    # placement bounds, clipped to background
    y1 = y - h_pad // 2
    y2 = y1 + h_pad
    x1 = x - w_pad // 2
    x2 = x1 + w_pad
    h_bg, w_bg = background.shape[:2]
    y1_c = max(0, y1)
    y2_c = min(h_bg, y2)
    x1_c = max(0, x1)
    x2_c = min(w_bg, x2)
    if y1_c >= y2_c or x1_c >= x2_c:
        return

    # matching region of the rotated overlay
    ov_y1 = y1_c - y1
    ov_y2 = ov_y1 + (y2_c - y1_c)
    ov_x1 = x1_c - x1
    ov_x2 = ov_x1 + (x2_c - x1_c)
    overlay_crop = rotated[ov_y1:ov_y2, ov_x1:ov_x2]
    bg_crop = background[y1_c:y2_c, x1_c:x2_c]

    # alpha blend per channel: out = (1-a)*bg + a*ov
    if overlay_crop.shape[2] == 4:
        alpha = overlay_crop[:, :, 3] / 255.0
        for c in range(3):
            bg_crop[:, :, c] = (1.0 - alpha) * bg_crop[:, :, c] + alpha * overlay_crop[:, :, c]
    else:
        background[y1_c:y2_c, x1_c:x2_c] = overlay_crop
