"""
states/upload_fence.py — UPLOAD_FENCE state.
==============================================
Subroutine state: uploads one or more fence polygons to the
autopilot, then returns to the calling state.

Before transitioning here, the caller must set:
    shared["fence_polygons"] — list of polygon dicts:
        [
            {
                "type": "inclusion" | "exclusion",
                "points": [(lat, lon), (lat, lon), ...],
                "label": "Flight Area",       # for logging
            },
            ...
        ]
    shared["return_to"] — state to return to (default: "IDLE")

CRITICAL DESIGN NOTE — Why all fences are uploaded together:
    The MAVLink fence protocol (MAV_MISSION_TYPE_FENCE) replaces
    the ENTIRE fence list on each upload. There is no "append"
    operation. This means:

    1. If you upload Flight Area (inclusion) + SSSI (exclusion),
       the Cube stores both.
    2. If you later upload a PLB Focus Area (inclusion) separately,
       it REPLACES fences 1+2 — the Flight Area and SSSI vanish.

    Therefore, every fence upload must include ALL active fences
    in a single transaction. The base fences (Flight Area + SSSI)
    are stored in shared["base_fence_polygons"] so that a future
    REPLAN state can re-upload them alongside a new Focus Area fence.

On success, sets shared["fence_uploaded"] = True.
On failure, sets shared["fence_uploaded"] = False.
Always returns to the caller either way.
"""

from states.base import BaseState
from pymavlink import mavutil
from config import (
    FENCE_ENABLE_VALUE,
    FENCE_TYPE_VALUE,
    FENCE_ACTION_VALUE,
    FENCE_ALT_MAX_VALUE,
    FENCE_MARGIN_VALUE,
)


class UploadFenceState(BaseState):

    name = "UPLOAD_FENCE"

    def enter(self):
        polygons = self.shared.get("fence_polygons", [])
        return_to = self.shared.get("return_to", "IDLE")
        total_points = sum(len(p["points"]) for p in polygons)
        self.log(f"Preparing to upload {len(polygons)} polygon(s) "
                 f"({total_points} total vertices), return to {return_to}")

    def _set_fence_parameters(self):
        """
        Push the fence parameters from config.py to the autopilot.
        These must be set before the fence polygons are uploaded so
        that ArduPilot knows what action to take on breach.
        """
        params = [
            ("FENCE_ENABLE", float(FENCE_ENABLE_VALUE)),
            ("FENCE_TYPE",   float(FENCE_TYPE_VALUE)),
            ("FENCE_ACTION", float(FENCE_ACTION_VALUE)),
            ("FENCE_ALT_MAX", float(FENCE_ALT_MAX_VALUE)),
            ("FENCE_MARGIN", float(FENCE_MARGIN_VALUE)),
        ]
        for name, value in params:
            try:
                self.drone.set_parameter_nowait(name, value)
                self.log(f"  {name} = {value}")
            except Exception as e:
                self.log(f"  WARNING: failed to set {name}: {e}")

    def execute(self):
        polygons = self.shared.get("fence_polygons", [])
        return_to = self.shared.pop("return_to", "IDLE")

        # ── Validate ──
        if not polygons:
            self.log("ERROR: No fence polygons in shared data")
            self.shared["fence_uploaded"] = False
            return return_to

        for i, poly in enumerate(polygons):
            points = poly.get("points", [])
            fence_type = poly.get("type", "unknown")
            label = poly.get("label", f"polygon {i+1}")

            if len(points) < 3:
                self.log(f"ERROR: {label} has {len(points)} points "
                         f"(minimum 3 for a polygon)")
                self.shared["fence_uploaded"] = False
                return return_to

            if fence_type not in ("inclusion", "exclusion"):
                self.log(f"ERROR: {label} has unknown type '{fence_type}' "
                         f"(must be 'inclusion' or 'exclusion')")
                self.shared["fence_uploaded"] = False
                return return_to

        # ── Set fence parameters on autopilot ──
        self.log("Setting fence parameters on autopilot:")
        self._set_fence_parameters()

        # ── Upload ──
        success = self.drone.upload_fence(polygons)
        self.shared["fence_uploaded"] = success

        if success:
            labels = [p.get("label", "?") for p in polygons]
            self.log(f"Fence upload SUCCEEDED — {', '.join(labels)}")
        else:
            self.log("Fence upload FAILED")

        # Don't pop fence_polygons — the caller may want to re-use them.
        # Store base fences for potential later re-upload (e.g. future REPLAN state).
        if success and "base_fence_polygons" not in self.shared:
            self.shared["base_fence_polygons"] = polygons

        return return_to

    def exit(self):
        pass
