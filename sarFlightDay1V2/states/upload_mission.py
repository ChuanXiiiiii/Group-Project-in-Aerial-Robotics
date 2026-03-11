"""
states/upload_mission.py — UPLOAD_MISSION state.
==================================================
Subroutine state: uploads a waypoint list to the autopilot
then returns to the calling state.

Before transitioning here, the caller must set:
    shared["waypoints"]  — list of dicts: [{"lat": ..., "lon": ..., "alt": ...}, ...]
    shared["return_to"]  — state name to return to after upload (default: "IDLE")

The upload uses the MAVLink mission protocol (MISSION_COUNT →
MISSION_REQUEST_INT → MISSION_ITEM_INT → MISSION_ACK).

NOTE: This state pops shared["waypoints"] on entry so the list is
consumed.  Callers that need to reuse the waypoints (e.g. the second
pass in SearchState) must store them separately in shared before
transitioning here — see shared["original_waypoints"] set by IdleState.

On success, sets shared["mission_uploaded"] = True.
On failure, sets shared["mission_uploaded"] = False.
Always returns to the caller either way.
"""

from states.base import BaseState


class UploadMissionState(BaseState):

    name = "UPLOAD_MISSION"

    def enter(self):
        waypoints = self.shared.get("waypoints", [])
        return_to = self.shared.get("return_to", "IDLE")
        self.log(f"Preparing to upload {len(waypoints)} waypoint(s), "
                 f"return to {return_to}")

    def execute(self):
        waypoints = self.shared.pop("waypoints", [])
        return_to = self.shared.pop("return_to", "IDLE")

        # ── Validate ──
        if not waypoints:
            self.log("ERROR: No waypoints in shared data — nothing to upload")
            self.shared["mission_uploaded"] = False
            return return_to

        # ── Log what we're uploading ──
        for i, wp in enumerate(waypoints):
            self.log(f"  WP {i + 1}: {wp['lat']:.6f}, {wp['lon']:.6f} @ {wp['alt']}m")

        # ── Upload ──
        success = self.drone.upload_mission(waypoints)
        self.shared["mission_uploaded"] = success

        if success:
            self.log(f"Mission uploaded successfully — {len(waypoints)} waypoint(s)")
        else:
            self.log("Mission upload FAILED")

        return return_to

    def exit(self):
        pass
