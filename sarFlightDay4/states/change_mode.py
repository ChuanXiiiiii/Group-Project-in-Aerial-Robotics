"""
states/change_mode.py — CHANGE_MODE state.
===========================================
Subroutine state: sends a mode-change command, waits for
confirmation, then returns to the calling state.

Before transitioning here, the caller must set:
    shared["requested_mode"] = "LOITER"   (or any valid mode)
    shared["return_to"]      = "MY_STATE" (optional, defaults to "IDLE")

Transitions:
    → shared["return_to"]  (default: "IDLE")
"""

from states.base import BaseState


class ChangeModeState(BaseState):

    name = "CHANGE_MODE"

    def enter(self):
        target = self.shared.get("requested_mode", "???")
        return_to = self.shared.get("return_to", "IDLE")
        self.log(f"Target mode: {target}  (return to: {return_to})")

    def execute(self):
        target = self.shared.get("requested_mode")
        return_to = self.shared.pop("return_to", "IDLE")

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

    def exit(self):
        pass
