"""
shared.py — Thread-safe shared status between state machine and GUI.
====================================================================
The state machine WRITES status here.
The GUI READS status and WRITES commands here.
A threading.Lock protects all access.
"""

import threading
import time


class SharedStatus:
    """
    Thread-safe store for state machine status and GUI commands.

    Status (written by state machine, read by GUI):
        current_state   : "IDLE", "SEARCH", etc.
        current_mode    : "STABILIZE", "LOITER", etc.
        transitioning   : True while switching states
        last_message    : most recent log message
        connected       : True once heartbeat received

    Commands (written by GUI, read by state machine):
        pending_command : command dict, or None
    """

    def __init__(self):
        self._lock = threading.Lock()

        # ── Status (state machine → GUI) ──
        self._current_state = "STARTING"
        self._current_mode = "UNKNOWN"
        self._transitioning = False
        self._last_message = ""
        self._connected = False
        self._last_update = 0.0

        # ── Commands (GUI → state machine) ──
        self._pending_command = None

        # ── Extra data (state machine → GUI, generic) ──
        self._extra = {}

    # ── Status getters (GUI reads these) ──────────────────────

    def get_status(self):
        """Return a snapshot dict of all status fields."""
        with self._lock:
            status = {
                "current_state": self._current_state,
                "current_mode": self._current_mode,
                "transitioning": self._transitioning,
                "last_message": self._last_message,
                "connected": self._connected,
                "last_update": self._last_update,
            }
            status.update(self._extra)
            return status

    def get_extra(self, key, default=None):
        """Read a single extra field (thread-safe)."""
        with self._lock:
            return self._extra.get(key, default)

    # ── Status setters (state machine writes these) ───────────

    def set_state(self, state_name, transitioning=False):
        with self._lock:
            self._current_state = state_name
            self._transitioning = transitioning
            self._last_update = time.time()

    def set_mode(self, mode_name):
        with self._lock:
            self._current_mode = mode_name
            self._last_update = time.time()

    def set_connected(self, connected):
        with self._lock:
            self._connected = connected
            self._last_update = time.time()

    def set_message(self, message):
        with self._lock:
            self._last_message = message
            self._last_update = time.time()

    def set_extra(self, key, value):
        """Set an extra status field (state machine → GUI)."""
        with self._lock:
            self._extra[key] = value
            self._last_update = time.time()

    # ── Command queue (GUI writes, state machine reads) ───────

    def send_command(self, command):
        """
        GUI calls this to send a command to the state machine.

        Commands are dicts with a "type" field, for example:
            {"type": "setup_mission"}
            {"type": "confirm_pattern"}
            {"type": "open_checklist"}
            {"type": "start_search"}
            {"type": "cancel_search"}
        """
        with self._lock:
            self._pending_command = command

    def consume_command(self):
        """
        State machine calls this to check for a pending command.
        Returns the command dict and clears it, or None if empty.
        """
        with self._lock:
            cmd = self._pending_command
            self._pending_command = None
            return cmd
