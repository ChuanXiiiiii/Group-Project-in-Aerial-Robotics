"""
states/change_mode.py — CHANGE_MODE state.
===========================================
Sends mode-change command, verifies, then returns to the
calling state.

Other states can "call" CHANGE_MODE by setting:
    shared["requested_mode"] = "LOITER"
    shared["return_to"]      = "MY_STATE"     # optional, defaults to WAIT
    return "CHANGE_MODE"

Transitions:
    → shared["return_to"]   (default: "WAIT")
"""

from states.base import BaseState


class ChangeModeState(BaseState):

    name = "CHANGE_MODE"

    def enter(self):
        target = self.shared.get("requested_mode", "???")
        return_to = self.shared.get("return_to", "WAIT")
        self.log(f"Target mode: {target}  (return to: {return_to})")

    def execute(self):
        target = self.shared.get("requested_mode")
        return_to = self.shared.pop("return_to", "WAIT")

        if target is None:
            self.log(f"ERROR: No mode requested — returning to {return_to}")
            return return_to

        self.log(f"Calling set_mode({target}) ...")
        success = self.drone.set_mode(target)

        if success:
            self.log(f"Mode change to {target} SUCCEEDED — returning to {return_to}")
        else:
            self.log(f"Mode change to {target} FAILED — returning to {return_to}")

        self.shared.pop("requested_mode", None)
        return return_to
