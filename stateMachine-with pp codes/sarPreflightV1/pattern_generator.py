"""Pattern generation dispatcher.

State-machine naming:
algorithm0 -> lawnmower
algorithm1 -> perimeter_spiral
"""

from path_planner.lawnmower import generate_path as lawnmower
from path_planner.perimeter_spiral import generate_path as perimeter_spiral


_ALGORITHMS = {
    0: lawnmower,
    1: perimeter_spiral,
}

PATTERN_NAMES = {
    0: "lawnmower",
    1: "perimeter_spiral",
}


def generate_pattern(polygon, spacing_m, alt_m, heading_deg=0, algorithm_id=0):
    """Generate waypoints using selected planner algorithm.

    Returns a list of {"lat", "lon", "alt"} dicts.
    """
    if algorithm_id not in _ALGORITHMS:
        raise ValueError(
            f"Unsupported algorithm_id={algorithm_id}. "
            f"Expected one of: {sorted(_ALGORITHMS.keys())}"
        )

    planner = _ALGORITHMS[algorithm_id]
    return planner(
        polygon=polygon,
        spacing_m=spacing_m,
        alt_m=alt_m,
        heading_deg=heading_deg,
    )


def generate_lawnmower(polygon, spacing_m, alt_m, heading_deg=0):
    """Backward-compatible wrapper to lawnmower."""
    return generate_pattern(
        polygon=polygon,
        spacing_m=spacing_m,
        alt_m=alt_m,
        heading_deg=heading_deg,
        algorithm_id=0,
    )


def generate_perimeter(polygon, spacing_m, alt_m, heading_deg=0):
    """Wrapper to perimeter_spiral."""
    return generate_pattern(
        polygon=polygon,
        spacing_m=spacing_m,
        alt_m=alt_m,
        heading_deg=heading_deg,
        algorithm_id=1,
    )
