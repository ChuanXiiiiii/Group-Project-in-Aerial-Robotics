"""
main.py — Entry point for the SAR companion computer.
======================================================
Starts three things:
    1. MAVProxy (background subprocess)
    2. State machine (background thread)
    3. Web dashboard (main thread, Flask)

Usage:
    python3 main.py                        # full startup
    python3 main.py --no-proxy             # skip MAVProxy
    python3 main.py --no-gui               # skip web dashboard
    python3 main.py --no-proxy --no-gui    # state machine only
"""

import sys
import time
import threading
import argparse
import subprocess
import shutil
from config import (
    CONNECTION_STRING, BAUD_RATE,
    MAVPROXY_MASTER, MAVPROXY_MASTER_BAUD, MAVPROXY_OUTPUTS,
    MAVPROXY_STARTUP_DELAY,
    GUI_HOST, GUI_PORT,
)
from shared import SharedStatus
from connection import DroneConnection
from machine import StateMachine


# ── MAVProxy ───────────────────────────────────────────────────

def start_mavproxy():
    """Launch MAVProxy as a background subprocess."""
    # Check it's available
    if shutil.which("mavproxy.py") is None:
        try:
            subprocess.run(
                [sys.executable, "-m", "MAVProxy.mavproxy", "--help"],
                capture_output=True, timeout=5
            )
        except (subprocess.SubprocessError, FileNotFoundError):
            print("[PROXY] ERROR: MAVProxy not found.")
            print("[PROXY] Install: pip install MAVProxy --break-system-packages")
            return None

    cmd = [
        sys.executable, "-m", "MAVProxy.mavproxy",
        f"--master={MAVPROXY_MASTER},{MAVPROXY_MASTER_BAUD}",
        "--daemon",
        "--state-basedir=/tmp",
    ]
    for output in MAVPROXY_OUTPUTS:
        cmd.append(f"--out={output}")

    print(f"[PROXY] Starting MAVProxy ...")
    print(f"[PROXY] Master: {MAVPROXY_MASTER} @ {MAVPROXY_MASTER_BAUD}")
    for output in MAVPROXY_OUTPUTS:
        print(f"[PROXY] Output: {output}")

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    print(f"[PROXY] Waiting {MAVPROXY_STARTUP_DELAY}s for startup ...")
    time.sleep(MAVPROXY_STARTUP_DELAY)

    if proc.poll() is not None:
        stderr = proc.stderr.read().decode()
        print(f"[PROXY] ERROR: MAVProxy exited immediately")
        print(f"[PROXY] stderr: {stderr}")
        return None

    print(f"[PROXY] MAVProxy running (PID {proc.pid})")
    return proc


# ── State machine thread ──────────────────────────────────────

def run_state_machine(drone, shared_status):
    """Target function for the background thread."""
    sm = StateMachine(drone, shared_status)
    sm.run()


# ── Main ──────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="SAR Companion Computer")
    parser.add_argument("--port", default=CONNECTION_STRING)
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--no-proxy", action="store_true",
                        help="Skip starting MAVProxy")
    parser.add_argument("--no-gui", action="store_true",
                        help="Skip web dashboard (run state machine only)")
    args = parser.parse_args()

    proxy_proc = None
    shared_status = SharedStatus()

    try:
        # ── Step 1: MAVProxy ──
        if not args.no_proxy:
            proxy_proc = start_mavproxy()
            if proxy_proc is None:
                print("[MAIN] Cannot start MAVProxy — exiting")
                return
        else:
            print("[MAIN] Skipping MAVProxy (--no-proxy)")

        # ── Step 2: Connect to drone ──
        drone = DroneConnection(args.port, args.baud, shared_status)
        drone.connect()

        # ── Step 3: Start state machine in background thread ──
        sm_thread = threading.Thread(
            target=run_state_machine,
            args=(drone, shared_status),
            daemon=True,        # dies when main thread exits
        )
        sm_thread.start()
        print("[MAIN] State machine thread started")

        # ── Step 4: Start web dashboard (or block) ──
        if not args.no_gui:
            print(f"[MAIN] Dashboard at http://<pi-ip>:{GUI_PORT}")
            from gui import create_app
            app = create_app(shared_status)
            # use_reloader=False is critical — reloader spawns
            # a child process which breaks our threading
            app.run(host=GUI_HOST, port=GUI_PORT,
                    debug=False, use_reloader=False)
        else:
            print("[MAIN] No GUI (--no-gui). Press Ctrl+C to stop.")
            # Keep main thread alive so the daemon thread runs
            while True:
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n[EXIT] Stopped by user")

    finally:
        if proxy_proc is not None and proxy_proc.poll() is None:
            print(f"[PROXY] Stopping MAVProxy (PID {proxy_proc.pid}) ...")
            proxy_proc.terminate()
            try:
                proxy_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proxy_proc.kill()
            print("[PROXY] MAVProxy stopped")


if __name__ == "__main__":
    main()
