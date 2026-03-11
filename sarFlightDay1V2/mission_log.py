"""
mission_log.py — Human-readable mission log file writer.
=========================================================
Writes a plain text log of the mission to ~/sar_logs/ (or LOG_DIR
from config).  A new timestamped file is created on each startup.

What gets logged:
    • Startup / shutdown
    • State machine transitions  (IDLE → SEARCH, etc.)
    • Flight mode changes commanded by the state machine
    • Flight mode changes detected from outside the state machine
      (safety pilot override, RC switch, etc.)
    • Waypoint reached events during search
    • Pass 1 / Pass 2 transitions
    • Mission cancel / PLB events
    • Any ERROR or WARNING lines

Format example:
    ══════════════════════════════════════════════════════
    SAR MISSION LOG — 2025-03-11 14:32:01
    ══════════════════════════════════════════════════════

    14:32:01  [STARTUP]      System initialised — log started
    14:32:04  [CONNECT]      Connected to vehicle (sys=1 comp=1)
    14:32:06  [STATE]        IDLE
    14:33:12  [STATE]        GUIDED_TAKEOFF
    14:33:45  [TAKEOFF]      Reached 30.0m AGL — hovering in GUIDED
    14:33:46  [STATE]        IDLE
    14:34:01  [STATE]        PRE_AUTO_CHECK
    14:34:55  [STATE]        SEARCH
    14:34:55  [MODE CMD]     AUTO  (commanded by state machine)
    14:34:57  [MODE CHANGE]  AUTO
    14:35:10  [WAYPOINT]     WP 1 reached
    14:35:24  [WAYPOINT]     WP 2 reached
    14:35:38  [WAYPOINT]     WP 3 reached
    14:35:52  [WAYPOINT]     WP 4 reached
    14:35:53  [SEARCH]       Pass 1 complete — starting Pass 2
    14:36:45  [SEARCH]       Pass 2 complete — mission finished
    14:36:46  [MODE CMD]     GUIDED  (commanded by state machine)
    14:36:46  [STATE]        IDLE
    14:36:46  [SHUTDOWN]     System stopped

Usage:
    import mission_log as mlog
    mlog.init()                         # call once at startup
    mlog.state("IDLE")                  # state transition
    mlog.mode_cmd("AUTO")               # mode we commanded
    mlog.mode_change("AUTO")            # mode detected from heartbeat
    mlog.waypoint(seq, total)           # waypoint reached
    mlog.event("SEARCH", "Pass 1 complete")
    mlog.warning("GPS fix lost")
    mlog.error("Mission upload failed")
    mlog.shutdown()                     # call on exit
"""

import os
import threading
import time
import config

_lock     = threading.Lock()
_fh       = None          # open file handle
_log_path = None          # path to current log file
_last_mode = None         # track last seen mode to detect external changes


# ── Init / shutdown ──────────────────────────────────────────────

def init():
    """
    Open a new timestamped log file.  Call once at startup before
    the state machine thread starts.
    """
    global _fh, _log_path

    log_dir = getattr(config, "LOG_DIR", os.path.expanduser("~/sar_logs"))
    os.makedirs(log_dir, exist_ok=True)

    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    _log_path = os.path.join(log_dir, f"mission_{timestamp}.log")

    # Store path back into config so other modules can read it
    config.LOG_FILE = _log_path

    _fh = open(_log_path, "w", buffering=1, encoding="utf-8")

    _write_header(timestamp)
    _write("STARTUP", "System initialised — log started")
    print(f"[LOG] Mission log: {_log_path}")


def shutdown():
    """Write final entry and close the file."""
    global _fh
    if _fh is None:
        return
    _write("SHUTDOWN", "System stopped")
    _write_separator()
    with _lock:
        _fh.close()
        _fh = None


# ── Log functions (call from anywhere) ──────────────────────────

def connected(sys_id, comp_id):
    _write("CONNECT", f"Connected to vehicle (sys={sys_id} comp={comp_id})")


def state(state_name):
    """Log a state machine transition."""
    _write("STATE", state_name)


def mode_cmd(mode_name):
    """Log a flight mode change commanded by the state machine."""
    _write("MODE CMD", f"{mode_name}  (commanded by state machine)")


def mode_change(mode_name):
    """
    Log a flight mode update read from the heartbeat.
    Only writes when the mode actually changes, to avoid spamming the log.
    Distinguishes external changes (pilot override) from state machine commands.
    """
    global _last_mode
    with _lock:
        if mode_name == _last_mode:
            return
        _last_mode = mode_name
    _write("MODE CHANGE", mode_name)


def mode_override(old_mode, new_mode):
    """Log an unexpected mode change — safety pilot override."""
    _write("OVERRIDE", f"Safety pilot: {old_mode} → {new_mode}")


def waypoint(seq, total):
    """Log a waypoint reached event."""
    _write("WAYPOINT", f"WP {seq} reached  ({seq}/{total})")


def takeoff(alt_m):
    """Log takeoff complete."""
    _write("TAKEOFF", f"Reached {alt_m:.1f}m AGL — hovering in GUIDED")


def event(tag, message):
    """Log a named event (free-form)."""
    _write(tag.upper(), message)


def warning(message):
    _write("WARNING", message)


def error(message):
    _write("ERROR", message)


# ── Internal helpers ─────────────────────────────────────────────

def _now():
    return time.strftime("%H:%M:%S")


def _write(tag, message):
    if _fh is None:
        return
    line = f"{_now()}  [{tag:<14}]  {message}\n"
    with _lock:
        _fh.write(line)


def _write_separator():
    if _fh is None:
        return
    with _lock:
        _fh.write("-" * 60 + "\n")


def _write_header(timestamp):
    header = (
        "=" * 60 + "\n"
        f"SAR MISSION LOG -- {timestamp.replace('_', ' ')}\n"
        "=" * 60 + "\n\n"
    )
    with _lock:
        _fh.write(header)
