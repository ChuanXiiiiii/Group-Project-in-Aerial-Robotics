"""
states/base.py — Abstract base class for all states.
=====================================================
Every state MUST inherit from this and implement execute().

Rules:
    1. execute() blocks until a transition condition is met.
    2. execute() returns the NAME (string) of the next state.
    3. States never import or call other states directly.
    4. States use self.drone (DroneConnection) for all hardware.
    5. States use self.shared (dict) for passing data between states.
"""

from abc import ABC, abstractmethod


class BaseState(ABC):

    name = "UNNAMED"

    def __init__(self, drone, shared):
        self.drone = drone
        self.shared = shared

    def log(self, message):
        """Shortcut: log with this state's name as prefix."""
        self.drone.log(self.name, message)

    @abstractmethod
    def enter(self):
        """Called once when the machine transitions INTO this state."""
        pass

    @abstractmethod
    def execute(self):
        """
        Main logic loop. Blocks until transition condition met.
        Returns the name (string) of the next state.
        """
        pass

    def exit(self):
        """Called once when transitioning OUT. Override if needed."""
        pass
