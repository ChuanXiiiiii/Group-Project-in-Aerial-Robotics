"""
states/__init__.py — State registry.
======================================
Maps state name strings to state classes.  The machine uses this
dict to look up and instantiate states by name.  This is the ONLY
place that imports state classes.

Flying day state set:
    Operational:  IDLE, PRE_AUTO_CHECK, SEARCH
    Subroutines:  CHANGE_MODE, UPLOAD_MISSION, UPLOAD_FENCE,
                  GENERATE_PATTERN

To add a future state (e.g. REPLAN for PLB response):
    1. Create states/replan.py with a class inheriting BaseState
    2. Import it below
    3. Add it to STATE_CLASSES with its name string as the key
"""

from states.idle             import IdleState
from states.pre_auto_check   import PreAutoCheckState
from states.search           import SearchState
from states.change_mode      import ChangeModeState
from states.upload_mission   import UploadMissionState
from states.upload_fence     import UploadFenceState
from states.generate_pattern import GeneratePatternState
from states.guided_takeoff   import GuidedTakeoffState

# ┌──────────────────────────────────────────────────────┐
# │  STATE REGISTRY                                      │
# │  key   = state name (returned by execute())          │
# │  value = state class (instantiated by the machine)   │
# └──────────────────────────────────────────────────────┘

STATE_CLASSES = {
    # ── Operational ──────────────────────────────────────
    "IDLE":             IdleState,
    "PRE_AUTO_CHECK":   PreAutoCheckState,
    "SEARCH":           SearchState,

    # ── Optional / utility ───────────────────────────────
    "GUIDED_TAKEOFF":   GuidedTakeoffState,

    # ── Subroutines ──────────────────────────────────────
    "CHANGE_MODE":      ChangeModeState,
    "UPLOAD_MISSION":   UploadMissionState,
    "UPLOAD_FENCE":     UploadFenceState,
    "GENERATE_PATTERN": GeneratePatternState,
}

INITIAL_STATE = "IDLE"
