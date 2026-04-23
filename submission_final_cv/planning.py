# planning.py - lawnmower + spiral waypoint generation
import cv2
import numpy as np
import math
import config


class PathPlanner:
    """Generates GPS waypoints to cover a search polygon."""

    def __init__(self, geo_transformer, search_poly):
        self.geo = geo_transformer
        self.search_polygon = search_poly
        self.pix_per_m = self.geo.pix_per_m
        self.virtual_polygon = []

    def generate_search_pattern(self, map_w, map_h, drone_gps=None, alt_override=None):
        """Lawnmower pattern aligned to the polygon's longest edge."""
        if len(self.search_polygon) < 3:
            return []
        search_alt = alt_override or config.TARGET_ALT
        print(f"[PLANNER] Calculating Optimum Path (alt={search_alt:.0f}m)")

        # rasterise polygon
        mask = np.zeros((map_h, map_w), dtype=np.uint8)
        poly_pts = np.array([self.search_polygon], dtype=np.int32)
        cv2.fillPoly(mask, poly_pts, 255)

        self.virtual_polygon = poly_pts.reshape(-1, 1, 2)

        # rotate so longest edge is axis-aligned
        rect = cv2.minAreaRect(poly_pts[0])
        (center, size, angle) = rect
        scan_angle = angle + 90 if size[0] < size[1] else angle
        self.last_scan_angle = scan_angle

        rotation_mat = cv2.getRotationMatrix2D(center, scan_angle, 1.0)
        inverse_rotation = cv2.invertAffineTransform(rotation_mat)
        rotated_mask = cv2.warpAffine(mask, rotation_mat, (map_w, map_h))

        # spacing from camera footprint
        ground_footprint_m = (config.SENSOR_WIDTH_MM * search_alt) / config.FOCAL_LENGTH_MM
        if getattr(self, '_no_turn', False):
            aspect = config.IMAGE_H / config.IMAGE_W
            ground_footprint_m = ground_footprint_m * aspect
            overlap = 0.0
        else:
            overlap = 0.2
        swath_m = ground_footprint_m * (1.0 - overlap)
        strip_spacing_px = max(1, int(swath_m * self.pix_per_m))

        points = cv2.findNonZero(rotated_mask)
        if points is None:
            return []
        bbox_x, bbox_y, bbox_w, bbox_h = cv2.boundingRect(points)

        # single-footprint early exit
        bbox_w_m = bbox_w / self.pix_per_m if self.pix_per_m else bbox_w
        bbox_h_m = bbox_h / self.pix_per_m if self.pix_per_m else bbox_h
        if bbox_w_m <= ground_footprint_m and bbox_h_m <= ground_footprint_m:
            cx = bbox_x + bbox_w // 2
            cy = bbox_y + bbox_h // 2
            pt = np.array([[(cx, cy)]], dtype=np.float32)
            pt_orig = cv2.transform(pt, inverse_rotation)[0][0]
            centroid_gps = self.geo.pixels_to_gps(pt_orig[0], pt_orig[1])
            print(f"  Polygon ({bbox_w_m:.0f}x{bbox_h_m:.0f}m) fits in one "
                  f"footprint ({ground_footprint_m:.0f}m). Single centroid waypoint.")
            return [centroid_gps]

        # scan lines in rotated space
        all_strips = []
        inset_px = strip_spacing_px // 3
        bottom_limit = bbox_y + bbox_h - inset_px
        prev_scan_y = -999

        scan_lines = list(range(bbox_y + inset_px, bbox_y + bbox_h, strip_spacing_px))
        if not scan_lines or scan_lines[-1] < bottom_limit:
            scan_lines.append(bottom_limit)

        for scan_y in scan_lines:
            scan_y = min(scan_y, bottom_limit)
            if scan_y == prev_scan_y:
                continue
            prev_scan_y = scan_y
            row = rotated_mask[scan_y, :]
            filled_cols = np.where(row == 255)[0]
            if len(filled_cols) > 0:
                x_start = filled_cols[0] + inset_px
                x_end = filled_cols[-1] - inset_px
                if x_end > x_start:
                    all_strips.append([(x_start, scan_y), (x_end, scan_y)])
                else:
                    x_mid = (filled_cols[0] + filled_cols[-1]) // 2
                    all_strips.append([(x_mid, scan_y), (x_mid, scan_y)])

        # pick start corner nearest drone
        direction = 1

        if drone_gps and all_strips:
            drone_px = self.geo.gps_to_pixels(drone_gps[0], drone_gps[1])

            def _sq_dist_to_map(rotated_pt):
                pt_arr = np.array([[rotated_pt]], dtype=np.float32)
                map_pt = cv2.transform(pt_arr, inverse_rotation)[0][0]
                return (map_pt[0] - drone_px[0]) ** 2 + (map_pt[1] - drone_px[1]) ** 2

            first_strip = all_strips[0]
            last_strip = all_strips[-1]

            d_top_left = _sq_dist_to_map(first_strip[0])
            d_top_right = _sq_dist_to_map(first_strip[1])
            d_bot_left = _sq_dist_to_map(last_strip[0])
            d_bot_right = _sq_dist_to_map(last_strip[1])

            min_dist = min(d_top_left, d_top_right, d_bot_left, d_bot_right)

            if min_dist == d_bot_left or min_dist == d_bot_right:
                all_strips.reverse()
                d_left = _sq_dist_to_map(all_strips[0][0])
                d_right = _sq_dist_to_map(all_strips[0][1])
            else:
                d_left = d_top_left
                d_right = d_top_right

            if d_right < d_left:
                direction = -1

        # un-rotate and zigzag
        waypoints = []
        for strip in all_strips:
            if direction == -1:
                pt_start, pt_end = strip[1], strip[0]
            else:
                pt_start, pt_end = strip[0], strip[1]

            pts_rot = np.array([[pt_start, pt_end]], dtype=np.float32)
            pts_orig = cv2.transform(pts_rot, inverse_rotation)[0]

            waypoints.append(self.geo.pixels_to_gps(pts_orig[0][0], pts_orig[0][1]))
            waypoints.append(self.geo.pixels_to_gps(pts_orig[1][0], pts_orig[1][1]))

            direction *= -1

        # zero-length strips leave duplicates; drop them
        if len(waypoints) >= 2:
            deduped = [waypoints[0]]
            for wp in waypoints[1:]:
                if abs(wp[0] - deduped[-1][0]) > 1e-9 or abs(wp[1] - deduped[-1][1]) > 1e-9:
                    deduped.append(wp)
            if len(deduped) < len(waypoints):
                print(f"  Removed {len(waypoints) - len(deduped)} duplicate waypoints "
                      f"(polygon narrower than footprint in places)")
            waypoints = deduped

        return waypoints

    @staticmethod
    def smooth_waypoints(waypoints, num_arc_points=3):
        """Insert Bezier arc points at U-turn corners."""
        if len(waypoints) < 4:
            return waypoints

        smoothed = [waypoints[0]]

        # TODO: num_arc_points=3 was tuned for 5 m/s; revisit once we fly faster
        for i in range(1, len(waypoints) - 1):
            prev = waypoints[i - 1]
            curr = waypoints[i]
            nxt = waypoints[i + 1]

            if i % 2 == 1:
                # end of strip, insert arc
                for t_idx in range(1, num_arc_points + 1):
                    t = t_idx / (num_arc_points + 1)
                    lat = (1 - t) ** 2 * prev[0] + 2 * (1 - t) * t * curr[0] + t ** 2 * nxt[0]
                    lon = (1 - t) ** 2 * prev[1] + 2 * (1 - t) * t * curr[1] + t ** 2 * nxt[1]
                    smoothed.append((float(lat), float(lon)))
            else:
                smoothed.append(curr)

        smoothed.append(waypoints[-1])
        return smoothed

    def generate_spiral_pattern(self, map_w, map_h, drone_gps=None, alt_override=None):
        """Inward rectangular spiral over the polygon."""
        if len(self.search_polygon) < 3:
            return []

        print("[PLANNER] Calculating Spiral Path")

        mask = np.zeros((map_h, map_w), dtype=np.uint8)
        poly_pts = np.array([self.search_polygon], dtype=np.int32)
        cv2.fillPoly(mask, poly_pts, 255)
        self.virtual_polygon = poly_pts.reshape(-1, 1, 2)

        rect = cv2.minAreaRect(poly_pts[0])
        (center, size, angle) = rect
        scan_angle = angle + 90 if size[0] < size[1] else angle

        rotation_mat = cv2.getRotationMatrix2D(center, scan_angle, 1.0)
        inverse_rotation = cv2.invertAffineTransform(rotation_mat)
        rotated_mask = cv2.warpAffine(mask, rotation_mat, (map_w, map_h))

        search_alt = alt_override or config.TARGET_ALT
        ground_footprint_m = (config.SENSOR_WIDTH_MM * search_alt) / config.FOCAL_LENGTH_MM
        overlap = 0.2
        swath_m = ground_footprint_m * (1.0 - overlap)
        strip_spacing_px  = max(1, int(swath_m * self.pix_per_m))

        points = cv2.findNonZero(rotated_mask)
        if points is None:
            return []
        bbox_x, bbox_y, bbox_w, bbox_h = cv2.boundingRect(points)

        # walk a shrinking rectangle inward
        spiral_pts = []
        top, bottom = bbox_y, bbox_y + bbox_h
        left, right = bbox_x, bbox_x + bbox_w
        half_step = strip_spacing_px // 2

        while top < bottom and left < right:
            # top: left -> right
            for sx in range(left + half_step, right, strip_spacing_px):
                if rotated_mask[min(top + half_step, map_h - 1), sx] == 255:
                    spiral_pts.append((sx, top + half_step))
            top += strip_spacing_px

            # right: top -> bottom
            for sy in range(top + half_step, bottom, strip_spacing_px):
                if rotated_mask[sy, min(right - half_step, map_w - 1)] == 255:
                    spiral_pts.append((right - half_step, sy))
            right -= strip_spacing_px

            # bottom: right -> left
            if top < bottom:
                for sx in range(right - half_step, left, -strip_spacing_px):
                    if rotated_mask[min(bottom - half_step, map_h - 1), sx] == 255:
                        spiral_pts.append((sx, bottom - half_step))
                bottom -= strip_spacing_px

            # left: bottom -> top
            if left < right:
                for sy in range(bottom - half_step, top, -strip_spacing_px):
                    if rotated_mask[sy, max(left + half_step, 0)] == 255:
                        spiral_pts.append((left + half_step, sy))
                left += strip_spacing_px

        if not spiral_pts:
            return []

        pts_arr = np.array([spiral_pts], dtype=np.float32)
        pts_orig = cv2.transform(pts_arr, inverse_rotation)[0]

        waypoints = []
        for pt in pts_orig:
            waypoints.append(self.geo.pixels_to_gps(pt[0], pt[1]))

        # rotate so the spiral starts nearest the drone
        if drone_gps and waypoints:
            drone_px = self.geo.gps_to_pixels(drone_gps[0], drone_gps[1])
            sq_dists = [
                (pt[0] - drone_px[0]) ** 2 + (pt[1] - drone_px[1]) ** 2
                for pt in pts_orig
            ]
            start_idx = sq_dists.index(min(sq_dists))
            waypoints = waypoints[start_idx:] + waypoints[:start_idx]

        print(f"  Spiral: {len(waypoints)} waypoints")
        return waypoints
