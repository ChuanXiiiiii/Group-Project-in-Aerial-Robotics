# =====================================================
# IMPORTS
# External libraries used for:
# - geometry operations (Shapely)
# - numerical operations (NumPy)
# - plotting/debug visualization (Matplotlib)
# - coordinate conversion between GPS and UTM (pyproj)
# =====================================================
import shapely
import shapely.ops
import shapely.plotting
import shapely.affinity
import numpy as np
import matplotlib.pyplot as plt

from pyproj import Transformer
from shapely.geometry import Polygon


# =====================================================
# COORDINATE SYSTEM SETTINGS
# INPUT_EPSG  : GPS coordinates in WGS84 (lon, lat)
# OUTPUT_EPSG : Projected UTM coordinates in meters
#               suitable for Bristol, UK
# =====================================================
INPUT_EPSG = "EPSG:4326"
OUTPUT_EPSG = "EPSG:32630"


# =====================================================
# CORE PLANNER FUNCTIONS
# These functions implement the full planner pipeline:
# GPS polygon -> projected polygon -> strips -> path
# =====================================================
def gps_to_polygon(gps_coords):
    """Convert a list of (lon, lat) GPS tuples into a Shapely polygon in meters."""
    to_meters = Transformer.from_crs(INPUT_EPSG, OUTPUT_EPSG, always_xy=True)
    meter_coords = [to_meters.transform(lon, lat) for lon, lat in gps_coords]
    return Polygon(meter_coords)


def split_poly(poly, width):
    """Split the polygon into horizontal slices of equal width."""
    min_x, min_y, max_x, max_y = poly.bounds
    y_splits = np.arange(min_y + width, max_y, width)

    res = poly
    for y in y_splits:
        split_line = shapely.LineString([(min_x, y), (max_x, y)])
        res = shapely.MultiPolygon(shapely.ops.split(res, split_line))

    return res


def make_strips(splits):
    """Convert clipped polygon slices into rectangular strip envelopes."""
    return shapely.MultiPolygon([g.envelope for g in splits.geoms])


def build_path_greedy(strips):
    """
    Build a lawnmower path using a greedy nearest-strip heuristic.
    Each strip contributes two possible endpoints: left midpoint and right midpoint.
    """
    pairs = []

    for s in strips.geoms:
        min_x, min_y, max_x, max_y = s.bounds
        pt_left = shapely.Point(min_x, 0.5 * (min_y + max_y))
        pt_right = shapely.Point(max_x, 0.5 * (min_y + max_y))
        pairs.append((pt_left, pt_right))

    path = []
    current_pt = shapely.Point(0, 0)

    while pairs:
        dists = [
            (current_pt.distance(pt_left), current_pt.distance(pt_right))
            for pt_left, pt_right in pairs
        ]

        next_strip = np.argmin([min(ds) for ds in dists])
        next_end = np.argmin(dists[next_strip])

        path.append(pairs[next_strip][next_end])
        path.append(pairs[next_strip][1 - next_end])

        current_pt = pairs[next_strip][1 - next_end]
        pairs.pop(next_strip)

    return shapely.LineString(path)


def lawnmower(poly, width, angle=0, use_radians=False):
    """
    Generate a rotated lawnmower coverage path.

    Workflow:
    1. Rotate polygon into the planner frame
    2. Split into equal-width slices
    3. Convert slices into rectangular strips
    4. Build greedy traversal path
    5. Rotate path and strips back to the original frame
    """
    poly_r = shapely.affinity.rotate(poly, -angle, origin=(0, 0), use_radians=use_radians)

    splits_r = split_poly(poly_r, width)
    strips_r = make_strips(splits_r)
    path_r = build_path_greedy(strips_r)

    strips = shapely.affinity.rotate(strips_r, angle, origin=(0, 0), use_radians=use_radians)
    path = shapely.affinity.rotate(path_r, angle, origin=(0, 0), use_radians=use_radians)

    return path, strips


# =====================================================
# WAYPOINT GENERATION
# Convert the final planner path from projected meters
# back into GPS coordinates as mission-style dictionaries.
# =====================================================
def path_to_waypoints(path, altitude=10.0):
    """Convert a planner path into a list of GPS waypoint dictionaries."""
    to_gps = Transformer.from_crs(OUTPUT_EPSG, INPUT_EPSG, always_xy=True)

    waypoints = []

    for x, y in path.coords:
        lon, lat = to_gps.transform(x, y)

        waypoint = {
            "lat": lat,
            "lon": lon,
            "alt": altitude
        }

        waypoints.append(waypoint)

    return waypoints


# =====================================================
# VISUALIZATION FUNCTIONS
# Optional plotting functions for checking the final
# polygon, strip layout, and generated lawnmower path.
# =====================================================
def plot_result(poly, strips, path):
    """Plot the survey polygon, strip envelopes, and final path."""
    _, ax = plt.subplots(figsize=(8, 8))
    shapely.plotting.plot_polygon(poly, ax=ax, color='g', add_points=False)
    shapely.plotting.plot_polygon(strips, ax=ax, color='b', alpha=0.1, add_points=False)
    shapely.plotting.plot_line(path, ax=ax)

    ax.set_aspect("equal")
    ax.set_title(f"Length = {path.length:.1f}")

    plt.tight_layout()
    plt.show()


# =====================================================
# MAIN
# End-to-end test flow:
# 1. Define input polygon coordinates
# 2. Build polygon
# 3. Run lawnmower planner
# 4. Convert path into GPS waypoints
# 5. Plot the result
# =====================================================
def main():
    # Example polygon input from teammate's parser
    # Format must remain: (lon, lat)
    polygon_coords = [
        (-2.6021, 51.4545),
        (-2.6018, 51.4545),
        (-2.6018, 51.4548),
        (-2.6021, 51.4548)
    ]

    # Use the real projected polygon when working with actual GPS input
    # my_polygon = gps_to_polygon(polygon_coords)

    # # Temporary local test polygon used for debugging geometry behavior
    # my_polygon = shapely.Polygon([
    #     (0, 0),
    #     (10, 0),
    #     (15, 8),
    #     (15, 10),
    #     (9, 8),
    #     (0, 12)
    # ])

    my_polygon = gps_to_polygon(polygon_coords)
    # Run lawnmower planner
    path, strips = lawnmower(my_polygon, width=1.2, angle=-45)

    print(f"Lawnmower path length: {path.length:.2f} meters")

    # Convert path -> GPS waypoint dictionaries
    waypoints = path_to_waypoints(path, altitude=10.0)

    print("\nGenerated waypoints:")
    print(waypoints)

    # Optional visualization
    plot_result(my_polygon, strips, path)






# =====================================================
# PUBLIC FUNCTION FOR TEAMMATES
# This is the only function they need to call.
# Input:  list of (lon, lat) tuples
# Output: list of waypoint dictionaries
# =====================================================
def generate_waypoints(polygon_coords, width=1.2, angle=-45, altitude=10.0):

    # Convert GPS polygon -> projected polygon
    poly = gps_to_polygon(polygon_coords)

    # Run lawnmower planner
    path, strips = lawnmower(poly, width, angle)

    # Convert planner path -> GPS waypoints
    waypoints = path_to_waypoints(path, altitude)
    print("I the waypoints generator:")
    return waypoints

if __name__ == "__main__":
    main()