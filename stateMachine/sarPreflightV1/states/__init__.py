"""
states/__init__.py — State registry.
=====================================
Add new states here. The machine uses this dict to look up
states by name. This is the ONLY place that imports state classes.

To add a new state:
    1. Create states/my_new_state.py with a class inheriting BaseState
    2. Import it here
    3. Add it to STATE_CLASSES with its name as the key
"""

from states.wait import WaitState
from states.change_mode import ChangeModeState
from states.timed_hold import TimedHoldState
from states.upload_mission import UploadMissionState
from states.upload_fence import UploadFenceState
from states.generate_pattern import GeneratePatternState
from states.search import SearchState

# ┌──────────────────────────────────────────────────────┐
# │  STATE REGISTRY                                      │
# │  key   = state name (returned by execute())          │
# │  value = state class (instantiated by the machine)   │
# └──────────────────────────────────────────────────────┘

STATE_CLASSES = {
    "WAIT":             WaitState,
    "CHANGE_MODE":      ChangeModeState,
    "TIMED_HOLD":       TimedHoldState,
    "UPLOAD_MISSION":   UploadMissionState,
    "UPLOAD_FENCE":     UploadFenceState,
    "GENERATE_PATTERN": GeneratePatternState,
    "SEARCH":           SearchState,
}

INITIAL_STATE = "WAIT"
