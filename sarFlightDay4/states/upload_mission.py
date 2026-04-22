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
            # ── Reset current waypoint to the start of the new mission ──
            # Without this, the autopilot may resume from whatever sequence
            # number it was on before the upload (e.g. waypoint 5 from a
            # previous SEARCH mission), causing the drone to skip the
            # beginning of the new mission entirely.
            try:
                self.drone.conn.mav.mission_set_current_send(
                    self.drone.conn.target_system,
                    self.drone.conn.target_component,
                    0,   # seq 0 = home → autopilot will advance to seq 1
                )
                self.log("Waypoint index reset to start of mission")
            except Exception as e:
                self.log(f"WARNING: failed to reset waypoint index: {e}")

            # ── Clear stale mission-progress cache ──
            # pymavlink's message cache (conn.messages) retains the last
            # MISSION_ITEM_REACHED and MISSION_CURRENT from the PREVIOUS
            # mission.  If the new mission's monitor loop reads these
            # before fresh messages arrive, the secondary completion
            # check ("last_reached > 0 and current_wp == 0") fires
            # immediately — the drone thinks the new mission is already
            # done.  Purging these two keys forces the monitors to wait
            # for real progress from the new mission.
            for stale_key in ("MISSION_ITEM_REACHED", "MISSION_CURRENT"):
                self.drone.conn.messages.pop(stale_key, None)
            self.log("Cleared stale mission progress cache")
        else:
            self.log("Mission upload FAILED")

        return return_to

    def exit(self):
        pass
