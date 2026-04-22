"""
states/__init__.py — State registry.
======================================
Maps state name strings to state classes.  The machine uses this
dict to look up and instantiate states by name.  This is the ONLY
place that imports state classes.

State set:
    Operational:  IDLE, GUIDED_TAKEOFF, PRE_AUTO_CHECK, SEARCH, REPLAN,
                  FOCUS, DELIVER
    Subroutines:  CHANGE_MODE, UPLOAD_MISSION, UPLOAD_FENCE,
                  GENERATE_PATTERN
"""

from states.idle             import IdleState
from states.pre_auto_check   import PreAutoCheckState
from states.search           import SearchState
from states.replan           import ReplanState
from states.focus            import FocusState
from states.deliver          import DeliverState
from states.safe_rtl         import SafeRtlState
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
    "REPLAN":           ReplanState,
    "FOCUS":            FocusState,
    "DELIVER":          DeliverState,

    # ── Optional / utility ───────────────────────────────
    "GUIDED_TAKEOFF":   GuidedTakeoffState,
    "SAFE_RTL":         SafeRtlState,

    # ── Subroutines ──────────────────────────────────────
    "CHANGE_MODE":      ChangeModeState,
    "UPLOAD_MISSION":   UploadMissionState,
    "UPLOAD_FENCE":     UploadFenceState,
    "GENERATE_PATTERN": GeneratePatternState,
}

INITIAL_STATE = "IDLE"
