"""
cv_comm.py — Communication with the CV detection pipeline.
===========================================================
Provides an atomic state-file mechanism for the state machine to tell
the CV script which mode it should be operating in.

State file:  ~/detections/cv_mode

Modes (written by state machine, read by CV):
    PASSIVE    — no detection needed, general camera stream
    DETECTING  — actively run dummy detection on camera frames
    GEOTAG     — drone is hovering stationary; produce a geotagged
                 image from the downward camera
"""

import os
import tempfile
from config import DETECTION_DIR

CV_MODE_FILE = os.path.join(DETECTION_DIR, "cv_mode")


def write_cv_mode(mode):
    """
    Atomically overwrite the CV mode file with *mode*.

    Uses write-to-temp + rename so the CV script never reads a
    half-written value.  Safe on ext4 / tmpfs / overlayfs.
    """
    os.makedirs(DETECTION_DIR, exist_ok=True)
    tmp_fd, tmp_path = tempfile.mkstemp(dir=DETECTION_DIR, prefix=".cv_mode_")
    try:
        os.write(tmp_fd, f"{mode}\n".encode())
        os.close(tmp_fd)
        os.replace(tmp_path, CV_MODE_FILE)
    except Exception:
        try:
            os.close(tmp_fd)
        except OSError:
            pass
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise


def read_cv_mode():
    """
    Read the current CV mode (convenience for the CV side).
    Returns the mode string, or "PASSIVE" if the file does not exist.
    """
    try:
        with open(CV_MODE_FILE, "r") as f:
            return f.read().strip()
    except FileNotFoundError:
        return "PASSIVE"
