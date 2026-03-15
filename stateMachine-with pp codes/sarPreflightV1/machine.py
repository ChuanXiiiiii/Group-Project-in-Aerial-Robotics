"""
machine.py — State machine runner.
===================================
Runs enter → execute → exit for the current state, then transitions.
Now updates SharedStatus so the GUI can display current state.
"""

from states import STATE_CLASSES, INITIAL_STATE


class StateMachine:

    def __init__(self, drone, shared_status=None):
        self.drone = drone
        self.shared = {}                # data bag between states
        self.current_name = None
        self.running = True
        self.status = shared_status     # GUI status (may be None)

    def run(self):
        self.current_name = INITIAL_STATE

        while self.running:

            # ── Look up state class ──
            state_class = STATE_CLASSES.get(self.current_name)
            if state_class is None:
                print(f"\n[MACHINE] FATAL: Unknown state '{self.current_name}'")
                break

            # ── Create instance ──
            state = state_class(drone=self.drone, shared=self.shared)

            # ── Tell GUI we're in this state ──
            if self.status:
                self.status.set_state(self.current_name, transitioning=False)

            # ── Lifecycle ──
            print(f"\n{'='*50}")
            print(f"  ENTERING STATE: {self.current_name}")
            print(f"{'='*50}\n")

            state.enter()
            next_name = state.execute()
            state.exit()

            # ── Validate transition ──
            if next_name not in STATE_CLASSES:
                print(f"\n[MACHINE] FATAL: '{self.current_name}' "
                      f"returned unknown state '{next_name}'")
                break

            # ── Transition ──
            if next_name != self.current_name:
                print(f"\n  >>> TRANSITION: {self.current_name} → {next_name}")

                # Brief amber flash on GUI during transition
                if self.status:
                    self.status.set_state(next_name, transitioning=True)

            self.current_name = next_name

        print("\n[MACHINE] Stopped")
