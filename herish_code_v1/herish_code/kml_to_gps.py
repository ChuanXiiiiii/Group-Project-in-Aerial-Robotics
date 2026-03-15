import csv
import xml.etree.ElementTree as ET
from turtledemo.chaos import plot

import matplotlib.pyplot as plt
import numpy as np
import shapely
import shapely.affinity
import shapely.ops
import shapely.plotting
from fontTools.unicodedata import block
from pyproj import Transformer
from shapely.geometry import Polygon


# =========================================================
# SETTINGS
# =========================================================
KML_FILE = "focus_sreach.kml"   # Put your KML file in the same folder as this script
STRIP_WIDTH = 8            # meters
INPUT_EPSG = "EPSG:4326"   # GPS lat/lon
OUTPUT_EPSG = "EPSG:32630" # Bristol / UK = UTM Zone 30N
ALTITUDE= 10.0             # Altitude of the drone during focused mission


# =========================================================
# KML EXTRACTION
# =========================================================
def extract_kml_coords(kml_file):
    """
    Extract outer polygon coordinates from KML as (lon, lat).
    """
    tree = ET.parse(kml_file)
    root = tree.getroot()

    ns = {"kml": "http://www.opengis.net/kml/2.2"}

    outer = root.find(".//kml:outerBoundaryIs/kml:LinearRing/kml:coordinates", ns)
    if outer is None or outer.text is None:
        raise ValueError("No outer polygon boundary found in the KML file.")

    gps_coords = []
    for item in outer.text.strip().split():
        lon, lat, *_ = map(float, item.split(","))
        gps_coords.append((lon, lat))

    # Remove repeated final point if it closes the ring
    if len(gps_coords) > 1 and gps_coords[0] == gps_coords[-1]:
        gps_coords = gps_coords[:-1]

    return gps_coords


# =========================================================
# GEOMETRY HELPERS
# =========================================================
def split_poly(poly, width):
    x_min, y_min, x_max, y_max = poly.bounds
    y_splits = np.arange(y_min + width, y_max, width)
    res = poly

    for y in y_splits:
        split_line = shapely.LineString([(x_min, y), (x_max, y)])
        res = shapely.MultiPolygon(shapely.ops.split(res, split_line))

    return res


def make_strips(splits):
    return shapely.MultiPolygon([g.envelope for g in splits.geoms])


def build_path_simple(strips):
    points = []

    for i, s in enumerate(strips.geoms):
        x_min, y_min, x_max, y_max = s.bounds
        pt_left = shapely.Point(x_min, 0.5 * (y_min + y_max))
        pt_right = shapely.Point(x_max, 0.5 * (y_min + y_max))

        if i % 2 == 0:
            points.append(pt_left)
            points.append(pt_right)
        else:
            points.append(pt_right)
            points.append(pt_left)

    return shapely.LineString(points)


def build_path_greedy(strips):
    pairs = []

    for s in strips.geoms:
        x_min, y_min, x_max, y_max = s.bounds
        pt_left = shapely.Point(x_min, 0.5 * (y_min + y_max))
        pt_right = shapely.Point(x_max, 0.5 * (y_min + y_max))
        pairs.append((pt_left, pt_right))

    path_points = []
    current_pt = shapely.Point(0, 0)

    while pairs:
        dists = [
            (current_pt.distance(pt_left), current_pt.distance(pt_right))
            for pt_left, pt_right in pairs
        ]
        next_strip = np.argmin([min(ds) for ds in dists])
        next_end = np.argmin(dists[next_strip])

        path_points.append(pairs[next_strip][next_end])
        path_points.append(pairs[next_strip][1 - next_end])
        current_pt = pairs[next_strip][1 - next_end]
        pairs.pop(next_strip)

    return shapely.LineString(path_points)


# =========================================================
# PLOTTING
# =========================================================
def plot_result(my_poly, strips, path, title_suffix=""):
    _, ax = plt.subplots(figsize=(8, 8))
    shapely.plotting.plot_polygon(my_poly, ax=ax, color='g', add_points=False)
    shapely.plotting.plot_polygon(strips, ax=ax, color='b', alpha=0.1, add_points=False)
    shapely.plotting.plot_line(path, ax=ax)
    ax.set_title(f'Length = {path.length:.1f} {title_suffix}')

    plt.tight_layout()
    plt.show()


# =========================================================
# EXPORT
# =========================================================
def extract_waypoints(path, to_gps, altitude=ALTITUDE):
    target_locations = []
    for point in path.coords:
        x, y = point
        lon, lat = to_gps.transform(x, y)
        target_locations.append((lat, lon, altitude))
    return target_locations

#=========================================================
# GENERATE WAYPOINT TO BE CALLED BY set_mission
#=========================================================

def generate_target_locations(kml_file=KML_FILE, strip_width=STRIP_WIDTH, altitude=ALTITUDE):
    gps_coords = extract_kml_coords(kml_file)

    to_meters = Transformer.from_crs(INPUT_EPSG, OUTPUT_EPSG, always_xy=True)
    to_gps = Transformer.from_crs(OUTPUT_EPSG, INPUT_EPSG, always_xy=True)

    meter_coords = [to_meters.transform(lon, lat) for lon, lat in gps_coords]
    my_poly = Polygon(meter_coords)

    splits = split_poly(my_poly, strip_width)
    strips = make_strips(splits)
    greedy_path = build_path_greedy(strips)

    return extract_waypoints(greedy_path, to_gps, altitude)

# =========================================================
# MAIN
# =========================================================
def main():
    # --- 1. Extract coordinates from KML ---
    gps_coords = extract_kml_coords(KML_FILE)

    print("Extracted GPS coordinates:")
    for c in gps_coords:
        print(c)

    print("\nNumber of points:", len(gps_coords))

    # --- 2. Setup projections ---
    to_meters = Transformer.from_crs(INPUT_EPSG, OUTPUT_EPSG, always_xy=True)
    to_gps = Transformer.from_crs(OUTPUT_EPSG, INPUT_EPSG, always_xy=True)

    # --- 3. Convert GPS -> meters ---
    meter_coords = [to_meters.transform(lon, lat) for lon, lat in gps_coords]

    print("\nFirst GPS coordinate:", gps_coords[0])
    print("First projected coordinate:", meter_coords[0])

    # --- 4. Create polygon ---
    my_poly = Polygon(meter_coords)

    print("\nPolygon validity:", my_poly.is_valid)
    print(f"Polygon area: {my_poly.area:.2f} square meters")

    # --- 5. Polygon dimensions ---
    min_x, min_y, max_x, max_y = my_poly.bounds
    width_from_poly = max_x - min_x
    height_from_poly = max_y - min_y

    print(f"Width of polygon bounding box: {width_from_poly:.2f} meters")
    print(f"Height of polygon bounding box: {height_from_poly:.2f} meters")

    # --- 6. Split polygon into strips ---
    splits = split_poly(my_poly, STRIP_WIDTH)
    print("\nPolygon split into strips.")

    # --- 7. Create strip envelopes ---
    strips = make_strips(splits)
    print("Strip envelopes created.")

    # --- 8. Build simple path ---
    simple_path = build_path_simple(strips)
    print(f"\nSimple path length: {simple_path.length:.2f} meters")

    # --- 9. Build greedy path ---
    greedy_path = build_path_greedy(strips)
    print(f"Greedy path length: {greedy_path.length:.2f} meters")

    # --- 10. Extract greedy path waypoints back to GPS ---
    #waypoints_gps = extract_waypoints(greedy_path, to_gps)
    target_locations = extract_waypoints(greedy_path, to_gps, altitude=10.0)

    print("\nWaypoint dictionary:")
    print(target_locations)

    # --- 11. Visualization ---
    #plot_result(my_poly, strips, simple_path, "(Simple Path)")
    #plot_result(my_poly, strips, greedy_path, "(Greedy Path)")

if __name__ == "__main__":
    main()