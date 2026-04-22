"""
gui.py — Web dashboard for the SAR state machine (flying day).
===============================================================
Serves the single-page operator interface at http://<pi-ip>:5000

Panels:
    • Connection status bar       — heartbeat dot, current mode
    • State machine row           — display-only state indicators
    • IDLE setup panel            — Setup Mission / Confirm Pattern /
                                    Open Pre-AUTO Checklist buttons
    • Pre-AUTO checklist panel    — 10 conditions + START SEARCH gate
    • Search progress panel       — waypoint progress, pass badge,
                                    PLB ACTIVATED, CANCEL MISSION

Removed from bench version:
    • Flight Mode command buttons  (direct mode switching)
    • Timed Hold panel             (bench-test only)
    • Manual waypoint upload table (managed by state machine now)

API endpoints:
    GET  /                          Dashboard HTML
    GET  /api/status                JSON status (polled every 500ms)
    GET  /api/pattern_preview       Pattern preview PNG
    POST /api/setup_mission         Trigger fence + pattern chain
    POST /api/confirm_pattern       Upload confirmed pattern waypoints
    POST /api/cancel_pattern        Cancel pending pattern
    POST /api/open_checklist        Transition to PRE_AUTO_CHECK
    POST /api/pre_auto_toggle       Toggle manual checkbox
    POST /api/start_search          Gate-checked START SEARCH
    POST /api/cancel_search         Abort active search
    POST /api/plb_activated         Mark PLB activated (stub)
    POST /api/guided_takeoff        Arm + takeoff to search alt
    POST /api/confirm_deploy         Confirm payload has been deployed
    POST /api/complete_mission       Relaunch drone and RTH
    POST /api/cancel_deliver         Cancel DELIVER state
    POST /api/load_plb_kml          Tell REPLAN to load the KML from PLB_KML_DIR
    POST /api/confirm_focus_pattern Confirm the generated focus pattern
    POST /api/cancel_replan         Cancel REPLAN and return to IDLE
    POST /api/cancel_focus          Cancel active FOCUS search
"""

from flask import Flask, jsonify, request, send_file
from config import (
    FLIGHT_MODES, DETECTION_DIR,
    SATELLITE_IMAGE,
    SATELLITE_BOUNDS_NORTH, SATELLITE_BOUNDS_SOUTH,
    SATELLITE_BOUNDS_EAST,  SATELLITE_BOUNDS_WEST,
    LANDING_STANDOFF_M,
    PLB_KML_DIR,
    GROUND_TRUTH_LOCATION,
)
from states import STATE_CLASSES
import os
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)
_shared = None   # injected by create_app()


def create_app(shared_status):
    global _shared
    _shared = shared_status
    return app


# ── API helpers ────────────────────────────────────────────────

def _ok(**kwargs):
    return jsonify({"ok": True, **kwargs})

def _err(msg, code=400):
    return jsonify({"error": msg}), code


# ── Status ─────────────────────────────────────────────────────

@app.route("/api/status")
def api_status():
    status = _shared.get_status()
    status["registered_states"] = list(STATE_CLASSES.keys())
    return jsonify(status)


# ── Pattern preview image ──────────────────────────────────────

@app.route("/api/pattern_preview")
def api_pattern_preview():
    from config import PATTERN_PREVIEW_PATH
    if os.path.exists(PATTERN_PREVIEW_PATH):
        return send_file(PATTERN_PREVIEW_PATH, mimetype="image/png")
    return _err("No preview available", 404)


# ── IDLE setup sequence ────────────────────────────────────────

@app.route("/api/setup_mission", methods=["POST"])
def api_setup_mission():
    """Trigger fence upload → generate pattern chain from IDLE."""
    _shared.send_command({"type": "setup_mission"})
    return _ok()


@app.route("/api/confirm_pattern", methods=["POST"])
def api_confirm_pattern():
    """Confirm staged pattern and upload to autopilot."""
    _shared.send_command({"type": "confirm_pattern"})
    return _ok()


@app.route("/api/cancel_pattern", methods=["POST"])
def api_cancel_pattern():
    """Cancel pending pattern."""
    _shared.send_command({"type": "cancel_pattern"})
    return _ok()


@app.route("/api/open_checklist", methods=["POST"])
def api_open_checklist():
    """Open Pre-AUTO checklist (IDLE must have mission uploaded)."""
    _shared.send_command({"type": "open_checklist"})
    return _ok()


# ── Pre-AUTO checklist ─────────────────────────────────────────

@app.route("/api/pre_auto_toggle", methods=["POST"])
def api_pre_auto_toggle():
    data  = request.get_json() or {}
    field = data.get("field")
    if field not in ("wp_verified", "pilot_ready"):
        return _err(f"Unknown field: {field}")
    _shared.send_command({"type": "pre_auto_toggle", "field": field})
    return _ok(field=field)


@app.route("/api/start_search", methods=["POST"])
def api_start_search():
    """Request START SEARCH (state machine gate-checks conditions)."""
    _shared.send_command({"type": "start_search"})
    return _ok()


@app.route("/api/cancel_checklist", methods=["POST"])
def api_cancel_checklist():
    """Return from Pre-AUTO checklist to IDLE."""
    _shared.send_command({"type": "cancel_checklist"})
    return _ok()


@app.route("/api/guided_takeoff", methods=["POST"])
def api_guided_takeoff():
    """Arm in GUIDED and climb to search altitude."""
    _shared.send_command({"type": "guided_takeoff"})
    return _ok()


# ── Search controls ────────────────────────────────────────────

@app.route("/api/cancel_search", methods=["POST"])
def api_cancel_search():
    """Abort the active search mission."""
    _shared.send_command({"type": "cancel_search"})
    return _ok()


@app.route("/api/plb_activated", methods=["POST"])
def api_plb_activated():
    """
    Mark PLB as activated.  Sets a flag in SharedStatus extras that
    SearchState._check_plb() reads each cycle.  This does NOT send a
    command through the command queue (which is single-slot) so it
    cannot be accidentally cleared by another command.
    """
    _shared.set_extra("plb_button_pressed", True)
    return _ok()


# ── DELIVER controls ──────────────────────────────────────────

@app.route("/api/detection_image")
def api_detection_image():
    """Serve the detection image file from DETECTION_DIR."""
    filename = request.args.get("file", "")
    if not filename:
        return _err("No file specified", 400)
    # Sanitise — only allow basename
    filename = os.path.basename(filename)
    filepath = os.path.join(DETECTION_DIR, filename)
    if os.path.exists(filepath):
        return send_file(filepath, mimetype="image/png")
    return _err("Detection image not found", 404)


@app.route("/api/satellite_bg")
def api_satellite_bg():
    """Serve the satellite background image for the landing canvas."""
    if os.path.exists(SATELLITE_IMAGE):
        return send_file(SATELLITE_IMAGE, mimetype="image/png")
    return _err("Satellite image not found", 404)


@app.route("/api/confirm_detection", methods=["POST"])
def api_confirm_detection():
    """Operator confirms the detection is the dummy."""
    _shared.send_command({"type": "confirm_detection"})
    return _ok()


@app.route("/api/reject_detection", methods=["POST"])
def api_reject_detection():
    """Operator rejects the detection — return to FOCUS."""
    _shared.send_command({"type": "reject_detection"})
    return _ok()


@app.route("/api/confirm_landing", methods=["POST"])
def api_confirm_landing():
    """Confirm selected landing point (lat/lon in body)."""
    data = request.get_json() or {}
    lat = data.get("lat")
    lon = data.get("lon")
    if lat is None or lon is None:
        return _err("Missing lat/lon")
    _shared.send_command({"type": "confirm_landing", "lat": float(lat), "lon": float(lon)})
    return _ok()


@app.route("/api/confirm_deploy", methods=["POST"])
def api_confirm_deploy():
    """Confirm payload has been deployed on the ground."""
    _shared.send_command({"type": "confirm_deploy"})
    return _ok()


@app.route("/api/complete_mission", methods=["POST"])
def api_complete_mission():
    """Relaunch the drone and return to home via GUIDED."""
    _shared.send_command({"type": "complete_mission"})
    return _ok()


@app.route("/api/cancel_deliver", methods=["POST"])
def api_cancel_deliver():
    """Cancel DELIVER and return to IDLE."""
    _shared.send_command({"type": "cancel_deliver"})
    return _ok()


# ── REPLAN controls ───────────────────────────────────────────

@app.route("/api/upload_plb_coords", methods=["POST"])
def api_upload_plb_coords():
    """
    Parse pasted <LinearRing> XML, extract the <coordinates> text,
    wrap it in a minimal KML file, and write it to PLB_KML_DIR so
    that REPLAN's existing load flow picks it up.
    """
    import re
    raw = request.json.get("coords", "").strip()
    if not raw:
        return jsonify({"ok": False, "error": "No coordinates provided"}), 400

    # Extract just the coordinate text from whatever was pasted.
    # Handles full <LinearRing><coordinates>...</coordinates></LinearRing>
    # or bare coordinate strings.
    m = re.search(r"<coordinates[^>]*>(.*?)</coordinates>", raw, re.DOTALL)
    coord_text = m.group(1).strip() if m else raw.strip()

    # Quick sanity check: must contain at least one comma (lon,lat,alt)
    if "," not in coord_text:
        return jsonify({"ok": False, "error": "No valid coordinates found"}), 400

    kml_content = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<kml xmlns="http://www.opengis.net/kml/2.2">\n'
        '<Document>\n'
        '  <Placemark>\n'
        '    <name>Focus Area</name>\n'
        '    <Polygon>\n'
        '      <outerBoundaryIs>\n'
        '        <LinearRing>\n'
        f'          <coordinates>{coord_text}</coordinates>\n'
        '        </LinearRing>\n'
        '      </outerBoundaryIs>\n'
        '    </Polygon>\n'
        '  </Placemark>\n'
        '</Document>\n'
        '</kml>\n'
    )

    import glob as _glob
    os.makedirs(PLB_KML_DIR, exist_ok=True)
    # Remove any old KML files so _find_kml() picks up the fresh one
    for old in _glob.glob(os.path.join(PLB_KML_DIR, "*.kml")):
        try:
            os.remove(old)
        except OSError:
            pass
    kml_path = os.path.join(PLB_KML_DIR, "plb_focus_area.kml")
    with open(kml_path, "w") as f:
        f.write(kml_content)

    return _ok(message=f"KML written to {kml_path}")


@app.route("/api/load_plb_kml", methods=["POST"])
def api_load_plb_kml():
    """Tell REPLAN to pick up the KML file from PLB_KML_DIR."""
    _shared.send_command({"type": "load_plb_kml"})
    return _ok()


@app.route("/api/confirm_focus_pattern", methods=["POST"])
def api_confirm_focus_pattern():
    """Confirm the generated focus pattern and proceed to upload."""
    _shared.send_command({"type": "confirm_focus_pattern"})
    return _ok()


@app.route("/api/cancel_replan", methods=["POST"])
def api_cancel_replan():
    """Cancel REPLAN and return to IDLE."""
    _shared.send_command({"type": "cancel_replan"})
    return _ok()


# ── FOCUS controls ────────────────────────────────────────────

@app.route("/api/cancel_focus", methods=["POST"])
def api_cancel_focus():
    """Cancel active FOCUS search."""
    _shared.send_command({"type": "cancel_focus"})
    return _ok()


# ── Dashboard page ─────────────────────────────────────────────

@app.route("/")
def dashboard():
    return _DASHBOARD_HTML


# ══════════════════════════════════════════════════════════════════════
#  DASHBOARD HTML
# ══════════════════════════════════════════════════════════════════════

_DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SAR Drone — Flying Day</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body {
  font-family: -apple-system,'Segoe UI',Roboto,monospace;
  background:#1a1a2e; color:#e0e0e0; min-height:100vh; padding:24px;
}
h1 { font-size:20px; font-weight:600; margin-bottom:8px; color:#fff; }

/* ── Status bar ── */
.status-bar { display:flex; align-items:center; gap:12px; margin-bottom:24px;
  font-size:13px; color:#888; }
.dot { width:10px; height:10px; border-radius:50%; background:#555; }
.dot.on  { background:#4caf50; }
.dot.off { background:#f44336; }

/* ── Sections ── */
.section { margin-bottom:28px; }
.section-label { font-size:11px; font-weight:600; text-transform:uppercase;
  letter-spacing:1.5px; color:#666; margin-bottom:10px; }

/* ── State buttons ── */
.btn-row { display:flex; flex-wrap:wrap; gap:8px; }
.btn {
  padding:10px 20px; border:2px solid #333; border-radius:8px;
  background:#2a2a3e; color:#888; font-family:inherit; font-size:13px;
  font-weight:600; cursor:pointer; transition:all 0.2s;
}
.btn.active   { background:#1b5e20; border-color:#4caf50; color:#fff;
  box-shadow:0 0 12px rgba(76,175,80,.3); }
.btn.transit  { background:#e65100; border-color:#ff9800; color:#fff;
  animation:pulse 1s ease-in-out infinite; }

/* ── Action buttons ── */
.action-btn {
  padding:12px 24px; border:2px solid; border-radius:8px;
  font-family:inherit; font-size:13px; font-weight:700;
  cursor:pointer; transition:all 0.2s; letter-spacing:.5px;
}
.action-btn:disabled { opacity:.4; cursor:not-allowed; }
.btn-blue  { background:#0d47a1; border-color:#42a5f5; color:#fff; }
.btn-green { background:#1b5e20; border-color:#4caf50; color:#fff; }
.btn-red   { background:#b71c1c; border-color:#ef5350; color:#fff; }
.btn-amber { background:#e65100; border-color:#ff9800; color:#fff; }
.btn-grey  { background:#2a2a3e; border-color:#555;    color:#888; }

/* ── Panels ── */
.panel {
  background:#16213e; border:1px solid #333; border-radius:8px; padding:16px;
}

/* ── Telemetry grid ── */
.telem-grid { display:grid; grid-template-columns:repeat(4,1fr); gap:10px; }
.telem-item { display:flex; flex-direction:column; }
.telem-label { font-size:10px; color:#666; text-transform:uppercase;
  letter-spacing:.5px; }
.telem-value { font-size:18px; font-weight:700; color:#e0e0e0; font-family:monospace; }
.telem-value.ok   { color:#4caf50; }
.telem-value.warn { color:#ff9800; }
.telem-value.bad  { color:#f44336; }

/* ── Checklist ── */
.check-grid { display:grid; grid-template-columns:auto 1fr auto;
  gap:5px 12px; align-items:center; }
.pf-dot { width:12px; height:12px; border-radius:50%;
  background:#333; border:1px solid #555; display:inline-block; }
.pf-dot.pass { background:#4caf50; border-color:#66bb6a;
  box-shadow:0 0 6px rgba(76,175,80,.4); }
.pf-dot.fail { background:#f44336; border-color:#ef5350;
  box-shadow:0 0 6px rgba(244,67,54,.4); }
.pf-label  { font-size:13px; color:#ccc; }
.pf-detail { font-size:11px; font-family:monospace; color:#666; text-align:right; }
.pf-check  { width:14px; height:14px; accent-color:#4caf50; cursor:pointer; }

/* ── Search panel ── */
#search-panel { display:none; border:2px solid #3949ab; border-radius:8px;
  padding:16px; background:#1a237e; }
#search-panel.visible { display:block; }
.search-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:10px 20px;
  margin-bottom:12px; }
.search-item { display:flex; flex-direction:column; }
.search-lbl  { font-size:10px; color:#9fa8da; text-transform:uppercase; }
.search-val  { font-size:18px; font-weight:700; color:#e8eaf6; font-family:monospace; }
.prog-bar    { background:#283593; border-radius:4px; height:8px;
  margin:8px 0; overflow:hidden; }
.prog-fill   { height:100%; background:#4caf50; border-radius:4px;
  transition:width .5s; width:0%; }
.prog-fill.paused   { background:#ff9800; }
.prog-fill.complete { background:#2196f3; }
.badge { display:inline-block; padding:3px 10px; border-radius:4px;
  font-size:12px; font-weight:700; text-transform:uppercase; }
.badge.flying   { background:#1b5e20; color:#a5d6a7; }
.badge.paused   { background:#e65100; color:#ffcc80; animation:pulse 1.5s infinite; }
.badge.complete { background:#0d47a1; color:#90caf9; }
.badge.pass2    { background:#4a148c; color:#ce93d8; }
.badge.init     { background:#37474f; color:#b0bec5; }
.badge.error    { background:#b71c1c; color:#ef9a9a; }
.search-controls { display:flex; gap:10px; margin-top:14px; flex-wrap:wrap; }

/* ── Pattern preview ── */
#pattern-preview { display:none; margin-top:12px; }
#pattern-img { width:100%; border-radius:6px; border:2px solid #333; }
.pattern-actions { display:flex; gap:8px; margin-top:10px; }

/* ── Takeoff status ── */
#takeoff-section { display:none; }
#takeoff-section.visible { display:flex; }
.takeoff-climbing { color:#ff9800; font-weight:700; animation:pulse 1s ease-in-out infinite; }
.takeoff-complete { color:#4caf50; font-weight:700; }
.takeoff-error    { color:#f44336; font-weight:700; }

/* ── PLB notice ── */
#plb-notice { display:none; background:#4a1a1a; border:2px solid #ef5350;
  border-radius:8px; padding:12px 16px; color:#ef9a9a; font-weight:600;
  font-size:13px; margin-top:12px; }

/* ── Replan panel ── */
#replan-panel { display:none; border:2px solid #ff9800; border-radius:8px;
  padding:16px; background:#3e2723; }
#replan-panel.visible { display:block; }
.replan-status-msg { font-size:14px; font-weight:600; color:#ffcc80;
  margin-bottom:12px; }
.replan-controls { display:flex; gap:10px; margin-top:14px; flex-wrap:wrap; }

/* ── Focus panel ── */
#focus-panel { display:none; border:2px solid #7c4dff; border-radius:8px;
  padding:16px; background:#1a0033; }
#focus-panel.visible { display:block; }
.focus-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:10px 20px;
  margin-bottom:12px; }
.focus-item { display:flex; flex-direction:column; }
.focus-lbl  { font-size:10px; color:#b39ddb; text-transform:uppercase; }
.focus-val  { font-size:18px; font-weight:700; color:#ede7f6; font-family:monospace; }
.badge.detection { background:#1b5e20; color:#a5d6a7; animation:pulse 1s infinite; }

/* ── Deliver panel ── */
#deliver-panel { display:none; border:2px solid #ff5722; border-radius:8px;
  padding:16px; background:#3e1a00; }
#deliver-panel.visible { display:block; }
.deliver-msg { font-size:14px; font-weight:600; color:#ffab91; margin-bottom:12px; }
.deliver-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:10px 20px;
  margin-bottom:12px; }
.deliver-item { display:flex; flex-direction:column; }
.deliver-lbl { font-size:10px; color:#ffab91; text-transform:uppercase; }
.deliver-val { font-size:18px; font-weight:700; color:#fbe9e7; font-family:monospace; }
.badge.landed  { background:#e65100; color:#ffcc80; }
.badge.deploy  { background:#1b5e20; color:#a5d6a7; }
.badge.rth     { background:#0d47a1; color:#90caf9; animation:pulse 1.5s infinite; }
.badge.mission-complete { background:#1b5e20; color:#a5d6a7; }
/* ── Detection confirmation ── */
#det-confirm-section { margin:12px 0; }
#det-confirm-section img { max-width:100%; border-radius:6px; border:2px solid #ff5722; }
/* ── Landing canvas ── */
#landing-canvas-section { margin:12px 0; }
#landing-canvas-overview { border:2px solid #555; border-radius:6px;
  width:100%; background:#0a0a1a; }
#landing-canvas { border:2px solid #555; border-radius:6px; cursor:crosshair;
  width:100%; background:#0a0a1a; }
#landing-coords { font-size:12px; color:#ffcc80; margin:6px 0; font-family:monospace; }

/* ── Mission complete panel ── */
#mission-complete-panel { display:none; border:2px solid #4caf50; border-radius:8px;
  padding:20px; background:#0d3717; text-align:center; }
#mission-complete-panel.visible { display:block; }
#mission-complete-panel h3 { font-size:22px; color:#a5d6a7; margin-bottom:8px;
  letter-spacing:0.1em; }
#mission-complete-panel .mc-sub { font-size:13px; color:#81c784; margin-bottom:4px; }

/* ── Safe RTL panel ── */
#safe-rtl-panel { display:none; border:2px solid #2196f3; border-radius:8px;
  padding:16px; background:#0d2137; }
#safe-rtl-panel.visible { display:block; }
.safe-rtl-msg { font-size:14px; font-weight:600; color:#90caf9; margin-bottom:12px; }
.safe-rtl-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:10px 20px;
  margin-bottom:12px; }
.safe-rtl-item { display:flex; flex-direction:column; }
.safe-rtl-lbl { font-size:10px; color:#64b5f6; text-transform:uppercase; }
.safe-rtl-val { font-size:18px; font-weight:700; color:#e3f2fd; font-family:monospace; }

/* ── Log ── */
.log-box { margin-top:24px; padding:14px 18px; background:#16213e;
  border:1px solid #333; border-radius:8px; font-size:13px;
  font-family:monospace; color:#aaa; min-height:44px; display:flex; align-items:center; }
.log-box .lbl { color:#555; margin-right:10px; }

@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.5} }
</style>
</head>
<body>

<h1>SAR Drone — Flying Day Dashboard</h1>
<div class="status-bar">
  <div class="dot" id="conn-dot"></div>
  <span id="conn-text">Connecting...</span>
</div>

<!-- ── State machine row ── -->
<div class="section">
  <div class="section-label">State Machine</div>
  <div class="btn-row" id="state-buttons"></div>
</div>

<!-- ── IDLE setup panel ── -->
<div class="section">
  <div class="section-label">Mission Setup</div>
  <div class="panel">
    <!-- Live telemetry -->
    <div class="telem-grid" style="margin-bottom:16px;">
      <div class="telem-item">
        <span class="telem-label">Mode</span>
        <span class="telem-value" id="tl-mode">—</span>
      </div>
      <div class="telem-item">
        <span class="telem-label">GPS fix / sats</span>
        <span class="telem-value" id="tl-gps">—</span>
      </div>
      <div class="telem-item">
        <span class="telem-label">Altitude (m)</span>
        <span class="telem-value" id="tl-alt">—</span>
      </div>
      <div class="telem-item">
        <span class="telem-label">Battery</span>
        <span class="telem-value" id="tl-bat">—</span>
      </div>
    </div>
    <!-- Setup buttons -->
    <div style="display:flex; gap:10px; flex-wrap:wrap; align-items:center;">
      <button class="action-btn btn-blue" id="btn-setup" onclick="setupMission()">
        ① Setup Mission
      </button>
      <span id="setup-status" style="font-size:12px; color:#888;"></span>
    </div>
    <!-- Guided Takeoff — shown when disarmed on ground -->
    <div id="takeoff-section" style="margin-top:12px; display:flex; gap:10px; align-items:center; flex-wrap:wrap;">
      <button class="action-btn btn-amber" id="btn-takeoff" onclick="guidedTakeoff()">
        ↑ Guided Takeoff
      </button>
      <span id="takeoff-status" style="font-size:12px; color:#888;"></span>
    </div>
    <!-- Pattern preview -->
    <div id="pattern-preview">
      <img id="pattern-img" src="" alt="Pattern preview">
      <div style="margin:8px 0; font-size:12px; color:#9fa8da;" id="pattern-info"></div>
      <div class="pattern-actions">
        <button class="action-btn btn-green" onclick="confirmPattern()" style="flex:1;">
          ✓ Confirm &amp; Upload Pattern
        </button>
        <button class="action-btn btn-grey" onclick="cancelPattern()" style="flex:0 0 auto;">
          ✕ Cancel
        </button>
      </div>
    </div>
    <!-- Checklist gate -->
    <div style="margin-top:14px; display:flex; gap:10px; align-items:center;">
      <button class="action-btn btn-blue" id="btn-checklist"
              onclick="openChecklist()" disabled>
        ② Open Pre-AUTO Checklist
      </button>
      <span id="checklist-gate-hint" style="font-size:12px; color:#666;">
        Upload mission first
      </span>
    </div>
    <!-- PLB notice (shown after returning from SEARCH with PLB flag) -->
    <div id="plb-notice">
      ⚠ PLB has been activated — drone switching to GUIDED and entering REPLAN.
    </div>
  </div>
</div>

<!-- ── Pre-AUTO checklist ── -->
<div class="section" id="checklist-section" style="display:none;">
  <div class="section-label">Pre-AUTO Checklist</div>
  <div class="panel" style="border-color:#42a5f5;">
    <div class="check-grid">

      <span id="pf-hb-dot"      class="pf-dot"></span>
      <span class="pf-label">Heartbeat</span>
      <span id="pf-hb-det"      class="pf-detail">—</span>

      <span id="pf-gps-dot"     class="pf-dot"></span>
      <span class="pf-label">GPS quality</span>
      <span id="pf-gps-det"     class="pf-detail">—</span>

      <span id="pf-bat-dot"     class="pf-dot"></span>
      <span class="pf-label">Battery</span>
      <span id="pf-bat-det"     class="pf-detail">—</span>

      <span id="pf-mode-dot"    class="pf-dot"></span>
      <span class="pf-label">Mode = GUIDED</span>
      <span id="pf-mode-det"    class="pf-detail">—</span>

      <span id="pf-arm-dot"     class="pf-dot"></span>
      <span class="pf-label">Armed</span>
      <span id="pf-arm-det"     class="pf-detail">—</span>

      <span id="pf-alt-dot"     class="pf-dot"></span>
      <span class="pf-label">Altitude in range</span>
      <span id="pf-alt-det"     class="pf-detail">—</span>

      <span id="pf-miss-dot"    class="pf-dot"></span>
      <span class="pf-label">Mission uploaded</span>
      <span id="pf-miss-det"    class="pf-detail">—</span>

      <span id="pf-fence-dot"   class="pf-dot"></span>
      <span class="pf-label">Fences uploaded</span>
      <span id="pf-fence-det"   class="pf-detail">—</span>

      <input type="checkbox" id="pf-pilot-cb" class="pf-check"
             onchange="toggleCheck('pilot_ready')">
      <span class="pf-label" style="cursor:pointer;"
            onclick="document.getElementById('pf-pilot-cb').click()">
        Safety pilot ready
      </span>
      <span class="pf-detail">operator</span>

    </div>
    <div style="margin-top:16px; display:flex; align-items:center; gap:12px;">
      <button class="action-btn btn-green" id="btn-start-search"
              onclick="startSearch()" disabled
              style="padding:14px 32px; font-size:14px; letter-spacing:1px;">
        START SEARCH
      </button>
      <span id="ready-label" style="font-size:13px; font-weight:600; color:#555;">
        NOT READY
      </span>
      <button class="action-btn btn-grey" onclick="cancelChecklist()"
              style="margin-left:auto; padding:10px 18px; font-size:12px;">
        Cancel
      </button>
    </div>
  </div>
</div>

<!-- ── Search progress panel ── -->
<div class="section">
  <div class="section-label">Search Progress</div>
  <div id="search-panel">
    <div style="display:flex; justify-content:space-between; align-items:center;
                margin-bottom:10px;">
      <span id="search-badge" class="badge init">INITIALISING</span>
      <span id="search-pass"  style="color:#9fa8da; font-size:13px; font-weight:700;"></span>
      <span id="search-elapsed" style="color:#9fa8da; font-size:13px;">0:00</span>
    </div>
    <div class="search-grid">
      <div class="search-item">
        <span class="search-lbl">Waypoint</span>
        <span class="search-val" id="s-wp">0 / 0</span>
      </div>
      <div class="search-item">
        <span class="search-lbl">Progress</span>
        <span class="search-val" id="s-pct">0%</span>
      </div>
      <div class="search-item">
        <span class="search-lbl">Altitude</span>
        <span class="search-val" id="s-alt">0.0 m</span>
      </div>
      <div class="search-item">
        <span class="search-lbl">Latitude</span>
        <span class="search-val" id="s-lat">—</span>
      </div>
      <div class="search-item">
        <span class="search-lbl">Longitude</span>
        <span class="search-val" id="s-lon">—</span>
      </div>
      <div class="search-item">
        <span class="search-lbl">Battery</span>
        <span class="search-val" id="s-bat">—</span>
      </div>
    </div>
    <div class="prog-bar">
      <div class="prog-fill" id="prog-fill"></div>
    </div>
    <div class="search-controls">
      <button class="action-btn btn-red" id="btn-cancel-search"
              onclick="cancelSearch()" disabled>
        CANCEL MISSION
      </button>
      <button class="action-btn btn-amber" id="btn-plb"
              onclick="plbActivated()" disabled>
        PLB ACTIVATED
      </button>
    </div>
  </div>
</div>

<!-- ── REPLAN panel ── -->
<div class="section">
  <div class="section-label">PLB Replan</div>
  <div id="replan-panel">
    <div class="replan-status-msg" id="replan-msg">Waiting for PLB Focus Area KML...</div>
    <!-- Focus preview (reuses pattern preview endpoint) -->
    <div id="focus-preview" style="display:none; margin:12px 0;">
      <img id="focus-preview-img" src="" alt="Focus pattern preview"
           style="width:100%; border-radius:6px; border:2px solid #555;">
      <div style="margin:8px 0; font-size:12px; color:#ffcc80;" id="focus-info"></div>
    </div>
    <div style="margin:10px 0;">
      <label style="font-size:12px;color:#aaa;display:block;margin-bottom:4px;">
        Paste PLB coordinates (&lt;LinearRing&gt; block):
      </label>
      <textarea id="plb-coords-input" rows="4"
                style="width:100%;box-sizing:border-box;background:#1a2435;
                       color:#e0e0e0;border:1px solid #555;border-radius:6px;
                       padding:8px;font-family:monospace;font-size:11px;
                       resize:vertical;"
                placeholder="<LinearRing><coordinates>-2.669...,51.423...,0 ...</coordinates></LinearRing>"></textarea>
    </div>
    <div class="replan-controls">
      <button class="action-btn btn-blue" id="btn-load-kml"
              onclick="loadPlbKml()">
        Load PLB Coords
      </button>
      <button class="action-btn btn-green" id="btn-confirm-focus"
              onclick="confirmFocusPattern()" style="display:none;">
        Confirm Focus Pattern
      </button>
      <button class="action-btn btn-amber" id="btn-cancel-replan"
              onclick="cancelReplan()">
        Cancel &amp; Resume Search
      </button>
    </div>
  </div>
</div>

<!-- ── FOCUS progress panel ── -->
<div class="section">
  <div class="section-label">Focus Search</div>
  <div id="focus-panel">
    <div style="display:flex; justify-content:space-between; align-items:center;
                margin-bottom:10px;">
      <span id="focus-badge" class="badge init">INITIALISING</span>
      <span id="focus-elapsed" style="color:#b39ddb; font-size:13px;">0:00</span>
    </div>
    <div class="focus-grid">
      <div class="focus-item">
        <span class="focus-lbl">Waypoint</span>
        <span class="focus-val" id="f-wp">0 / 0</span>
      </div>
      <div class="focus-item">
        <span class="focus-lbl">Progress</span>
        <span class="focus-val" id="f-pct">0%</span>
      </div>
      <div class="focus-item">
        <span class="focus-lbl">Altitude</span>
        <span class="focus-val" id="f-alt">0.0 m</span>
      </div>
      <div class="focus-item">
        <span class="focus-lbl">Latitude</span>
        <span class="focus-val" id="f-lat">—</span>
      </div>
      <div class="focus-item">
        <span class="focus-lbl">Longitude</span>
        <span class="focus-val" id="f-lon">—</span>
      </div>
      <div class="focus-item">
        <span class="focus-lbl">Battery</span>
        <span class="focus-val" id="f-bat">—</span>
      </div>
    </div>
    <div class="prog-bar">
      <div class="prog-fill" id="focus-prog-fill"></div>
    </div>
    <div id="focus-detection-alert" style="display:none; margin:10px 0; padding:10px;
         background:#1b5e20; border:2px solid #4caf50; border-radius:6px;
         color:#a5d6a7; font-weight:700; font-size:14px;">
      CASUALTY DETECTED — <span id="det-coords"></span>
    </div>
    <div style="display:flex; gap:10px; margin-top:14px;">
      <button class="action-btn btn-red" id="btn-cancel-focus"
              onclick="cancelFocus()" disabled>
        CANCEL FOCUS
      </button>
    </div>
  </div>
</div>

<!-- ── DELIVER panel ── -->
<div class="section">
  <div class="section-label">Deliver Payload</div>
  <div id="deliver-panel">
    <div style="display:flex; justify-content:space-between; align-items:center;
                margin-bottom:10px;">
      <span id="deliver-badge" class="badge init">INITIALISING</span>
      <span id="deliver-target" style="color:#ffab91; font-size:12px;"></span>
    </div>
    <div class="deliver-msg" id="deliver-msg">Initialising ...</div>

    <!-- Phase 1: Detection confirmation -->
    <div id="det-confirm-section" style="display:none;">
      <div style="font-size:13px; color:#ffcc80; margin-bottom:8px;">
        Is this the casualty / dummy?
      </div>
      <img id="det-img" src="" alt="Detection image">
      <div style="display:flex; gap:10px; margin-top:10px;">
        <button class="action-btn btn-green" onclick="confirmDetection()">
          Yes — Confirm Detection
        </button>
        <button class="action-btn btn-red" onclick="rejectDetection()">
          No — Reject &amp; Resume Search
        </button>
      </div>
    </div>

    <!-- Phase 2: NESW landing point selection with satellite background -->
    <div id="landing-canvas-section" style="display:none;">
      <div style="font-size:13px; color:#ffcc80; margin-bottom:8px;">
        Select a landing direction. The drone will land at the chosen point.
      </div>
      <!-- Two canvases stacked: overview (top, wide) + detail (bottom, wide) -->
      <div style="display:flex; flex-direction:column; gap:6px;">
        <div>
          <div style="font-size:11px; color:#aaa; margin-bottom:4px; text-align:center;">OVERVIEW — full area</div>
          <canvas id="landing-canvas-overview" width="600" height="320"></canvas>
        </div>
        <div>
          <div style="font-size:11px; color:#aaa; margin-bottom:4px; text-align:center;">DETAIL — landing options</div>
          <canvas id="landing-canvas" width="600" height="400"></canvas>
        </div>
      </div>
      <div id="landing-coords" style="font-size:12px; color:#ffcc80; margin:6px 0; font-family:monospace;">
        Select N, E, S, or W below
      </div>
      <!-- Compass formation: N top-centre, W left / E right, S bottom-centre -->
      <div style="display:grid; grid-template-columns:1fr 1fr 1fr; grid-template-rows:auto auto auto; gap:6px; margin-top:10px; max-width:320px; margin-left:auto; margin-right:auto;">
        <div></div>
        <button class="action-btn" id="btn-nesw-N"
                style="background:#1565c0; color:#fff; font-weight:bold; padding:10px 0;"
                onclick="selectNESW('N')">
          N
        </button>
        <div></div>
        <button class="action-btn" id="btn-nesw-W"
                style="background:#1565c0; color:#fff; font-weight:bold; padding:10px 0;"
                onclick="selectNESW('W')">
          W
        </button>
        <div></div>
        <button class="action-btn" id="btn-nesw-E"
                style="background:#1565c0; color:#fff; font-weight:bold; padding:10px 0;"
                onclick="selectNESW('E')">
          E
        </button>
        <div></div>
        <button class="action-btn" id="btn-nesw-S"
                style="background:#1565c0; color:#fff; font-weight:bold; padding:10px 0;"
                onclick="selectNESW('S')">
          S
        </button>
        <div></div>
      </div>
      <div style="display:flex; gap:10px; margin-top:10px; align-items:center;">
        <button class="action-btn btn-green" id="btn-confirm-landing"
                onclick="confirmLandingPoint()" disabled>
          Confirm Landing Point
        </button>
        <div style="flex:1;"></div>
        <button class="action-btn" id="btn-ground-truth"
                style="display:none; background:#e65100; border-color:#ff9800; color:#fff;
                       font-size:12px; padding:10px 16px;"
                onclick="useGroundTruth()">
          Use Ground Truth
        </button>
        <button class="action-btn btn-red" onclick="cancelDeliver()">
          Cancel
        </button>
      </div>
    </div>

    <!-- Phase 3+: Telemetry display -->
    <div id="deliver-telem-section" style="display:none;">
      <div class="deliver-grid">
        <div class="deliver-item">
          <span class="deliver-lbl">Distance</span>
          <span class="deliver-val" id="d-dist">—</span>
        </div>
        <div class="deliver-item">
          <span class="deliver-lbl">Altitude</span>
          <span class="deliver-val" id="d-alt">—</span>
        </div>
        <div class="deliver-item">
          <span class="deliver-lbl">Position</span>
          <span class="deliver-val" id="d-pos">—</span>
        </div>
      </div>
    </div>

    <!-- Phase 5: Deploy / Phase 6: Complete -->
    <div id="deliver-action-buttons" style="display:none;">
      <div style="display:flex; gap:10px; margin-top:14px; flex-wrap:wrap;">
        <button class="action-btn btn-green" id="btn-confirm-deploy"
                onclick="confirmDeploy()" style="display:none;">
          Confirm Deploy
        </button>
        <button class="action-btn btn-blue" id="btn-complete-mission"
                onclick="completeMission()" style="display:none;">
          Complete Mission (Relaunch &amp; Return Home)
        </button>
        <button class="action-btn btn-red" id="btn-cancel-deliver"
                onclick="cancelDeliver()">
          Cancel
        </button>
      </div>
    </div>
  </div>
</div>

<!-- ── SAFE RTL panel ── -->
<div class="section">
  <div class="section-label">Return Home (SSSI-Safe Corridor)</div>
  <div id="safe-rtl-panel">
    <div style="display:flex; justify-content:space-between; align-items:center;
                margin-bottom:10px;">
      <span id="srtl-badge" class="badge init">INITIALISING</span>
      <span id="srtl-elapsed" style="color:#90caf9; font-size:13px;">0:00</span>
    </div>
    <div class="safe-rtl-msg" id="srtl-msg">Returning home via safe corridor ...</div>
    <div class="safe-rtl-grid">
      <div class="safe-rtl-item">
        <span class="safe-rtl-lbl">Reached</span>
        <span class="safe-rtl-val" id="srtl-wp">0 / 0</span>
      </div>
      <div class="safe-rtl-item">
        <span class="safe-rtl-lbl">Progress</span>
        <span class="safe-rtl-val" id="srtl-pct">0%</span>
      </div>
      <div class="safe-rtl-item">
        <span class="safe-rtl-lbl">Altitude</span>
        <span class="safe-rtl-val" id="srtl-alt">—</span>
      </div>
      <div class="safe-rtl-item">
        <span class="safe-rtl-lbl">Latitude</span>
        <span class="safe-rtl-val" id="srtl-lat">—</span>
      </div>
      <div class="safe-rtl-item">
        <span class="safe-rtl-lbl">Longitude</span>
        <span class="safe-rtl-val" id="srtl-lon">—</span>
      </div>
      <div class="safe-rtl-item">
        <span class="safe-rtl-lbl">Battery</span>
        <span class="safe-rtl-val" id="srtl-bat">—</span>
      </div>
    </div>
    <div class="prog-bar">
      <div class="prog-fill" id="srtl-prog-fill"></div>
    </div>
  </div>
</div>

<!-- ── MISSION COMPLETE panel ── -->
<div class="section">
  <div id="mission-complete-panel">
    <h3>MISSION COMPLETE</h3>
    <div class="mc-sub">Drone has returned to home and landed safely.</div>
    <div class="mc-sub">Payload deployed. No further actions required.</div>
  </div>
</div>

<div class="log-box">
  <span class="lbl">LOG:</span>
  <span id="log-msg">Waiting for connection...</span>
</div>

<script>
let _init = false;

// ── Polling ──────────────────────────────────────────────────
async function poll() {
  try {
    const r = await fetch('/api/status');
    const s = await r.json();
    update(s);
  } catch(e) {
    document.getElementById('conn-dot').className = 'dot off';
    document.getElementById('conn-text').textContent = 'GUI disconnected';
  }
}

function update(s) {
  // Connection
  const dot = document.getElementById('conn-dot');
  const txt = document.getElementById('conn-text');
  if (s.connected) {
    dot.className = 'dot on';
    txt.textContent = 'Connected — ' + s.current_mode;
  } else {
    dot.className = 'dot off';
    txt.textContent = 'Waiting for heartbeat...';
  }

  // State buttons (build once)
  if (!_init && s.registered_states) {
    buildStateButtons(s.registered_states);
    _init = true;
  }
  if (s.registered_states) {
    s.registered_states.forEach(name => {
      const b = document.getElementById('state-' + name);
      if (!b) return;
      b.className = 'btn';
      if (name === s.current_state)
        b.classList.add(s.transitioning ? 'transit' : 'active');
    });
  }

  // Log
  if (s.last_message)
    document.getElementById('log-msg').textContent = s.last_message;

  // Idle status
  updateIdlePanel(s);

  // Mode telemetry
  document.getElementById('tl-mode').textContent = s.current_mode || '—';

  // Checklist panel visibility
  const inCheck = (s.current_state === 'PRE_AUTO_CHECK');
  document.getElementById('checklist-section').style.display =
    inCheck ? 'block' : 'none';

  // Checklist data
  if (s.pre_auto_checks) updateChecklist(s.pre_auto_checks);

  // Search panel
  updateSearchPanel(s);

  // Replan panel
  updateReplanPanel(s);

  // Focus panel
  updateFocusPanel(s);

  // Deliver panel
  updateDeliverPanel(s);

  // Safe RTL panel
  updateSafeRtlPanel(s);

  // Mission complete panel — terminal state, no further actions
  const mcPanel = document.getElementById('mission-complete-panel');
  if (s.mission_complete) {
    mcPanel.className = 'visible';
  } else {
    mcPanel.className = '';
  }
}

function buildStateButtons(states) {
  const row = document.getElementById('state-buttons');
  row.innerHTML = '';
  states.forEach(name => {
    const b = document.createElement('button');
    b.className = 'btn';
    b.id = 'state-' + name;
    b.textContent = name;
    row.appendChild(b);
  });
}

// ── IDLE panel ───────────────────────────────────────────────
function updateIdlePanel(s) {
  const idle = s.idle_status || {};

  // Telemetry values
  const gps = s.current_mode !== 'UNKNOWN'
    ? (idle.gps_fix != null ? 'fix:' + idle.gps_fix + ' ' + idle.satellites + 'sats' : '—')
    : '—';
  document.getElementById('tl-gps').textContent = gps;
  document.getElementById('tl-alt').textContent = idle.alt != null ? idle.alt.toFixed(1) : '—';
  const bat = idle.battery_pct != null && idle.battery_pct >= 0
    ? idle.battery_pct + '%' : '—';
  document.getElementById('tl-bat').textContent = bat;

  // Setup button state
  const inIdle = (s.current_state === 'IDLE');
  const btnSetup = document.getElementById('btn-setup');
  const setupStatus = document.getElementById('setup-status');

  if (idle.fence_uploaded && idle.mission_uploaded) {
    setupStatus.textContent = '✓ Fences and mission uploaded';
    setupStatus.style.color = '#4caf50';
  } else if (idle.fence_uploaded && !idle.mission_uploaded) {
    setupStatus.textContent = 'Fences uploaded — confirm pattern to upload mission';
    setupStatus.style.color = '#ff9800';
  } else if (s.current_state === 'UPLOAD_FENCE' || s.current_state === 'GENERATE_PATTERN') {
    setupStatus.textContent = 'Setting up...';
    setupStatus.style.color = '#42a5f5';
  } else {
    setupStatus.textContent = '';
  }

  // Pattern preview
  const preview = document.getElementById('pattern-preview');
  const patStatus = s.pattern_status || 'idle';
  if (patStatus === 'awaiting_confirmation') {
    preview.style.display = 'block';
    document.getElementById('pattern-img').src =
      '/api/pattern_preview?t=' + Date.now();
    const info = s.pending_pattern_info;
    document.getElementById('pattern-info').textContent =
      info ? info.wp_count + ' waypoints' +
        (info.oob_count > 0 ? ' — ⚠ ' + info.oob_count + ' out of bounds' : '') : '';
  } else {
    preview.style.display = 'none';
  }

  // Checklist gate button
  const btnCL = document.getElementById('btn-checklist');
  const hint  = document.getElementById('checklist-gate-hint');
  if (idle.mission_uploaded) {
    btnCL.disabled = false;
    hint.textContent = '';
    btnCL.className = 'action-btn btn-blue';
  } else {
    btnCL.disabled = true;
    hint.textContent = 'Upload mission first';
  }

  // PLB notice — hide if mission is complete (PLB flags are cleared
  // on successful completion, but check both for safety)
  document.getElementById('plb-notice').style.display =
    (idle.plb_triggered && !s.mission_complete) ? 'block' : 'none';

  // Guided Takeoff section
  updateTakeoffSection(s, idle);
}

function updateTakeoffSection(s, idle) {
  const section  = document.getElementById('takeoff-section');
  const btn      = document.getElementById('btn-takeoff');
  const statusEl = document.getElementById('takeoff-status');
  const inIdle   = (s.current_state === 'IDLE');
  const inTakeoff = (s.current_state === 'GUIDED_TAKEOFF');

  // Show when in IDLE (not already airborne) or actively taking off
  const armed = s.idle_status && s.idle_status.armed;
  section.className = (inIdle || inTakeoff) ? 'visible' : '';

  // Button state
  btn.disabled = !inIdle || (inIdle && armed === true);

  // Status text
  const to = s.guided_takeoff_status;
  if (inTakeoff && to) {
    const cls = to.status && to.status.startsWith('CLIMBING')
      ? 'takeoff-climbing'
      : to.status === 'COMPLETE' ? 'takeoff-complete' : 'takeoff-error';
    statusEl.className = cls;
    const altStr = to.current_alt != null
      ? ` ${to.current_alt}m / ${to.target_alt}m` : '';
    statusEl.textContent = to.status + altStr;
  } else if (inIdle && to) {
    if (to.status === 'COMPLETE') {
      statusEl.className = 'takeoff-complete';
      statusEl.textContent = `✓ Reached ${to.current_alt}m — hovering in GUIDED`;
    } else if (to.status === 'TIMEOUT') {
      statusEl.className = 'takeoff-error';
      statusEl.textContent = `Timeout — reached ${to.current_alt}m`;
    } else {
      statusEl.className = '';
      statusEl.textContent = '';
    }
  } else {
    statusEl.className = '';
    statusEl.textContent = inIdle ? 'Arms and climbs to search altitude in GUIDED' : '';
  }
}

// ── Checklist ────────────────────────────────────────────────
function updateChecklist(checks) {
  const items = [
    { id:'hb',    key:'heartbeat' },
    { id:'gps',   key:'gps' },
    { id:'bat',   key:'battery' },
    { id:'mode',  key:'mode' },
    { id:'arm',   key:'armed' },
    { id:'alt',   key:'altitude' },
    { id:'miss',  key:'mission' },
    { id:'fence', key:'fences' },
  ];
  items.forEach(({id, key}) => {
    const dot = document.getElementById('pf-' + id + '-dot');
    const det = document.getElementById('pf-' + id + '-det');
    const c   = checks[key];
    if (dot && c) dot.className = 'pf-dot ' + (c.pass ? 'pass' : 'fail');
    if (det && c) det.textContent = c.detail || '';
  });
  // Manual checkbox
  const pilotCb = document.getElementById('pf-pilot-cb');
  if (checks.pilot_ready)  pilotCb.checked = checks.pilot_ready.pass;
  // START SEARCH gate
  const btn   = document.getElementById('btn-start-search');
  const label = document.getElementById('ready-label');
  if (checks.all_ready) {
    btn.disabled = false;
    btn.className = 'action-btn btn-green';
    btn.style.boxShadow = '0 0 16px rgba(76,175,80,.5)';
    label.textContent = 'ALL CHECKS PASSED';
    label.style.color = '#4caf50';
  } else {
    btn.disabled = true;
    btn.className = 'action-btn btn-grey';
    btn.style.boxShadow = 'none';
    label.textContent = 'NOT READY';
    label.style.color = '#555';
  }
}

// ── Search panel ─────────────────────────────────────────────
function updateSearchPanel(s) {
  const panel = document.getElementById('search-panel');
  const data  = s.search_progress || null;
  const state = s.current_state || '';

  const show = (state === 'SEARCH') ||
               (state === 'CHANGE_MODE' && data) ||
               (data !== null);
  panel.className = show ? 'visible' : '';

  const inSearch = (state === 'SEARCH');
  document.getElementById('btn-cancel-search').disabled = !inSearch;
  document.getElementById('btn-plb').disabled = !inSearch;

  if (!data) return;

  document.getElementById('s-wp').textContent =
    data.current_wp + ' / ' + data.total_wps;
  document.getElementById('s-pct').textContent = data.progress_pct + '%';
  document.getElementById('s-alt').textContent = data.alt + ' m';
  document.getElementById('s-lat').textContent =
    typeof data.lat === 'number' ? data.lat.toFixed(6) : '—';
  document.getElementById('s-lon').textContent =
    typeof data.lon === 'number' ? data.lon.toFixed(6) : '—';
  document.getElementById('s-bat').textContent =
    data.battery_pct >= 0 ? data.battery_pct + '%' : '—';
  document.getElementById('search-elapsed').textContent = data.elapsed || '0:00';
  document.getElementById('search-pass').textContent =
    data.second_pass ? '— PASS 2' : '';

  // Progress bar
  const fill = document.getElementById('prog-fill');
  fill.style.width = data.progress_pct + '%';
  const st = data.status || '';
  fill.className = 'prog-fill' +
    (st.includes('PAUSE')   ? ' paused'   : '') +
    (st === 'COMPLETE'      ? ' complete' : '');

  // Badge
  const badge = document.getElementById('search-badge');
  badge.textContent = st;
  badge.className = 'badge ' +
    (st === 'FLYING'             ? 'flying'   :
     st.includes('PAUSE')        ? 'paused'   :
     st === 'COMPLETE'           ? 'complete' :
     st === 'PASS 1 COMPLETE' ||
     st === 'STARTING PASS 2'    ? 'pass2'    :
     st.includes('ERROR') ||
     st.includes('ABORT') ||
     st === 'CANCELLED'          ? 'error'    : 'init');
}

// ── API calls ────────────────────────────────────────────────
async function post(url) {
  await fetch(url, { method:'POST',
    headers:{'Content-Type':'application/json'}, body:'{}' });
}

function setupMission() { post('/api/setup_mission'); }
function confirmPattern() { post('/api/confirm_pattern'); }
function cancelPattern()  { post('/api/cancel_pattern'); }
function openChecklist()  { post('/api/open_checklist'); }
function cancelChecklist(){ post('/api/cancel_checklist'); }
function startSearch()    { post('/api/start_search'); }
function guidedTakeoff()  {
  if (!confirm('Arm and climb to search altitude in GUIDED mode?')) return;
  post('/api/guided_takeoff');
}

async function toggleCheck(field) {
  await fetch('/api/pre_auto_toggle', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({field})
  });
}

function cancelSearch() {
  if (!confirm('Cancel the search mission and return to IDLE?')) return;
  post('/api/cancel_search');
}

function plbActivated() {
  if (!confirm('Confirm PLB activation. The drone will switch to GUIDED and enter REPLAN. Continue?')) return;
  post('/api/plb_activated');
}

function confirmDetection() {
  if (!confirm('Confirm this is the casualty/dummy? The drone will proceed to land nearby.')) return;
  post('/api/confirm_detection');
}
function rejectDetection() {
  if (!confirm('Reject this detection? The drone will resume the focus search.')) return;
  post('/api/reject_detection');
}
function confirmLandingPoint() {
  if (!_selectedLanding) return;
  if (!confirm('Confirm landing at ' + _selectedLanding.lat.toFixed(6) + ', ' + _selectedLanding.lon.toFixed(6) + '?')) return;
  fetch('/api/confirm_landing', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({lat: _selectedLanding.lat, lon: _selectedLanding.lon})
  });
}
function confirmDeploy() {
  if (!confirm('Confirm payload has been deployed?')) return;
  post('/api/confirm_deploy');
}
function completeMission() {
  if (!confirm('Relaunch the drone and return to home via GUIDED?')) return;
  post('/api/complete_mission');
}
function cancelDeliver() {
  if (!confirm('Cancel delivery?')) return;
  post('/api/cancel_deliver');
}

function useGroundTruth() {
  if (!GROUND_TRUTH || !_canvasData) return;
  if (!confirm('Override geotag with ground truth location (' +
               GROUND_TRUTH.lat.toFixed(6) + ', ' + GROUND_TRUTH.lon.toFixed(6) + ')?')) return;

  // Recompute NESW points around the ground truth location
  const R = 6371000.0;
  const d = STANDOFF_M;
  const lat = GROUND_TRUTH.lat;
  const lon = GROUND_TRUTH.lon;
  const dlat = (d / R) * (180 / Math.PI);
  const dlon = (d / (R * Math.cos(lat * Math.PI / 180))) * (180 / Math.PI);

  const newNesw = {
    N: {lat: lat + dlat, lon: lon,        label: 'N'},
    E: {lat: lat,        lon: lon + dlon,  label: 'E'},
    S: {lat: lat - dlat, lon: lon,        label: 'S'},
    W: {lat: lat,        lon: lon - dlon,  label: 'W'},
  };

  // Update canvas data with corrected coordinates
  _canvasData.detection_lat = lat;
  _canvasData.detection_lon = lon;
  _canvasData.nesw_points   = newNesw;

  // Reset selection
  _selectedLanding = null;
  _selectedDirection = null;
  document.getElementById('btn-confirm-landing').disabled = true;
  document.getElementById('landing-coords').textContent = 'Ground truth applied — select N, E, S, or W';

  // Mark button as used
  const btn = document.getElementById('btn-ground-truth');
  btn.style.background = '#4a6600';
  btn.style.borderColor = '#8bc34a';
  btn.textContent = 'Ground Truth Applied';
  btn.disabled = true;

  // Reset NESW button colours
  ['N','E','S','W'].forEach(d => {
    const b = document.getElementById('btn-nesw-' + d);
    if (b) { b.style.background = '#1565c0'; b.style.boxShadow = 'none'; }
  });

  // Redraw canvases with corrected location
  drawOverviewCanvas(_canvasData);
  drawLandingCanvas(_canvasData);
}

async function loadPlbKml() {
  const ta = document.getElementById('plb-coords-input');
  const coords = ta ? ta.value.trim() : '';
  if (coords) {
    // Upload pasted coordinates → writes KML file on disk
    try {
      const r = await fetch('/api/upload_plb_coords', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({coords}),
      });
      const j = await r.json();
      if (!j.ok) { alert('Coord upload failed: ' + (j.error || 'unknown')); return; }
    } catch(e) { alert('Coord upload error: ' + e); return; }
  }
  // Now tell REPLAN to load the KML (whether from paste or pre-existing file)
  post('/api/load_plb_kml');
}
function confirmFocusPattern() {
  if (!confirm('Upload confirmed focus pattern and begin FOCUS search?')) return;
  post('/api/confirm_focus_pattern');
}
function cancelReplan() {
  if (!confirm('Cancel replan and resume the original search mission?')) return;
  post('/api/cancel_replan');
}
function cancelFocus() {
  if (!confirm('Cancel the focus search and return to IDLE?')) return;
  post('/api/cancel_focus');
}

// ── Replan panel ────────────────────────────────────────────
function updateReplanPanel(s) {
  const panel = document.getElementById('replan-panel');
  const state = s.current_state || '';
  const data  = s.replan_status || null;

  const show = (state === 'REPLAN') || (data !== null);
  panel.className = show ? 'visible' : '';
  if (!show) return;

  const msg = document.getElementById('replan-msg');
  const btnLoad    = document.getElementById('btn-load-kml');
  const btnConfirm = document.getElementById('btn-confirm-focus');
  const preview    = document.getElementById('focus-preview');
  const info       = document.getElementById('focus-info');

  const status = data ? data.status : 'waiting_for_kml';

  if (status === 'waiting_for_kml') {
    msg.textContent = 'Paste PLB coordinates below, then press Load.';
    btnLoad.style.display = '';
    btnConfirm.style.display = 'none';
    preview.style.display = 'none';
  } else if (status === 'no_kml_found') {
    msg.textContent = 'No coordinates or KML found. Paste coordinates and try again.';
    msg.style.color = '#ef5350';
    btnLoad.style.display = '';
    btnConfirm.style.display = 'none';
    preview.style.display = 'none';
  } else if (status === 'awaiting_confirmation') {
    msg.textContent = 'Focus pattern generated — review and confirm.';
    msg.style.color = '#ffcc80';
    btnLoad.style.display = 'none';
    btnConfirm.style.display = '';
    preview.style.display = 'block';
    document.getElementById('focus-preview-img').src =
      '/api/pattern_preview?t=' + Date.now();
    if (data.info && data.info.wp_count) {
      info.textContent = data.info.wp_count + ' waypoints';
    }
  } else if (status === 'error') {
    msg.textContent = 'Error during replan — check logs.';
    msg.style.color = '#ef5350';
    btnLoad.style.display = '';
    btnConfirm.style.display = 'none';
    preview.style.display = 'none';
  } else {
    msg.textContent = 'Replan in progress...';
    msg.style.color = '#ffcc80';
    btnLoad.style.display = 'none';
    btnConfirm.style.display = 'none';
  }
}

// ── Focus panel ─────────────────────────────────────────────
function updateFocusPanel(s) {
  const panel = document.getElementById('focus-panel');
  const data  = s.focus_progress || null;
  const state = s.current_state || '';

  const show = (state === 'FOCUS') ||
               (state === 'CHANGE_MODE' && data) ||
               (data !== null);
  panel.className = show ? 'visible' : '';

  const inFocus = (state === 'FOCUS');
  document.getElementById('btn-cancel-focus').disabled = !inFocus;

  if (!data) return;

  document.getElementById('f-wp').textContent =
    data.current_wp + ' / ' + data.total_wps;
  document.getElementById('f-pct').textContent = data.progress_pct + '%';
  document.getElementById('f-alt').textContent = data.alt + ' m';
  document.getElementById('f-lat').textContent =
    typeof data.lat === 'number' ? data.lat.toFixed(6) : '—';
  document.getElementById('f-lon').textContent =
    typeof data.lon === 'number' ? data.lon.toFixed(6) : '—';
  document.getElementById('f-bat').textContent =
    data.battery_pct >= 0 ? data.battery_pct + '%' : '—';
  document.getElementById('focus-elapsed').textContent = data.elapsed || '0:00';

  // Progress bar
  const fill = document.getElementById('focus-prog-fill');
  fill.style.width = data.progress_pct + '%';
  const st = data.status || '';
  fill.className = 'prog-fill' +
    (st.includes('PAUSE')     ? ' paused'   : '') +
    (st.includes('COMPLETE')  ? ' complete'  : '') +
    (st.includes('DETECTION') ? ' complete'  : '');

  // Badge
  const badge = document.getElementById('focus-badge');
  badge.textContent = st;
  badge.className = 'badge ' +
    (st === 'FLYING'              ? 'flying'    :
     st.includes('PAUSE')         ? 'paused'    :
     st.includes('COMPLETE')      ? 'complete'  :
     st.includes('DETECTION')     ? 'detection' :
     st.includes('ERROR') ||
     st === 'CANCELLED'           ? 'error'     : 'init');

  // Detection alert
  const detAlert = document.getElementById('focus-detection-alert');
  if (data.detection_lat != null && data.detection_lon != null) {
    detAlert.style.display = 'block';
    document.getElementById('det-coords').textContent =
      data.detection_lat.toFixed(6) + ', ' + data.detection_lon.toFixed(6);
  } else if (st.includes('DETECTION')) {
    detAlert.style.display = 'block';
    document.getElementById('det-coords').textContent = 'see logs';
  } else {
    detAlert.style.display = 'none';
  }
}

// ── Safe RTL panel ──────────────────────────────────────────
function updateSafeRtlPanel(s) {
  const panel = document.getElementById('safe-rtl-panel');
  const data  = s.safe_rtl_status || null;
  const state = s.current_state || '';

  const show = (state === 'SAFE_RTL') || (data !== null);
  panel.className = show ? 'visible' : '';
  if (!show) return;

  const st = data ? (data.status || '') : '';
  const msg = document.getElementById('srtl-msg');
  const badge = document.getElementById('srtl-badge');

  // Waypoint progress — show reached (completed) waypoints, not current heading target
  document.getElementById('srtl-wp').textContent =
    data && data.reached_wp != null ? data.reached_wp + ' / ' + data.total_wps : '0 / 0';
  document.getElementById('srtl-pct').textContent =
    data && data.progress_pct != null ? data.progress_pct + '%' : '0%';

  // Telemetry
  document.getElementById('srtl-alt').textContent =
    data && data.alt != null ? data.alt + ' m' : '—';
  document.getElementById('srtl-lat').textContent =
    data && typeof data.lat === 'number' ? data.lat.toFixed(6) : '—';
  document.getElementById('srtl-lon').textContent =
    data && typeof data.lon === 'number' ? data.lon.toFixed(6) : '—';
  document.getElementById('srtl-bat').textContent =
    data && data.battery_pct != null && data.battery_pct >= 0
      ? data.battery_pct + '%' : '—';

  // Elapsed time
  document.getElementById('srtl-elapsed').textContent =
    data && data.elapsed ? data.elapsed : '0:00';

  // Progress bar
  const pct = data && data.progress_pct != null ? data.progress_pct : 0;
  const fill = document.getElementById('srtl-prog-fill');
  fill.style.width = pct + '%';
  fill.className = 'prog-fill' +
    (st.includes('PAUSED')   ? ' paused'   : '') +
    (st.includes('COMPLETE') ? ' complete'  : '');

  // Badge
  badge.textContent = st;
  badge.className = 'badge ' +
    (st.includes('RETURNING')        ? 'rth'              :
     st.includes('LANDING')          ? 'paused'           :
     st.includes('MISSION COMPLETE') ? 'mission-complete' :
     st.includes('PAUSED')           ? 'paused'           :
     st.includes('UPLOADING')        ? 'init'             :
     st.includes('WAITING')          ? 'paused'           :
     st.includes('ERROR') ||
     st.includes('TIMEOUT')          ? 'error'            : 'init');

  // Message
  if (st.includes('UPLOADING'))       msg.textContent = 'Uploading safe corridor mission ...';
  else if (st.includes('RETURNING'))  msg.textContent = 'Flying home via SSSI-safe corridor ...';
  else if (st.includes('LANDING'))    msg.textContent = 'Landing at home ...';
  else if (st.includes('COMPLETE'))   msg.textContent = 'Mission complete — landed at home.';
  else if (st.includes('PAUSED'))     msg.textContent = 'Paused — pilot has control.';
  else if (st.includes('WAITING'))    msg.textContent = 'Waiting for GUIDED mode ...';
  else                                msg.textContent = st || 'Initialising ...';
}

// ── Deliver panel ───────────────────────────────────────────
let _selectedLanding = null;
let _selectedDirection = null;
let _canvasDrawn = false;
let _canvasData = null;  // cached polygon/detection data for canvas

// Satellite background config (injected from config.py)
const SAT_BOUNDS = {
  north: """ + str(SATELLITE_BOUNDS_NORTH) + """,
  south: """ + str(SATELLITE_BOUNDS_SOUTH) + """,
  east:  """ + str(SATELLITE_BOUNDS_EAST)  + """,
  west:  """ + str(SATELLITE_BOUNDS_WEST)  + """
};
const STANDOFF_M = """ + str(LANDING_STANDOFF_M) + """;

// Ground truth fallback (from config.py) — null if not set
const GROUND_TRUTH = """ + (
    "null" if GROUND_TRUTH_LOCATION is None
    else "{lat: %s, lon: %s}" % (GROUND_TRUTH_LOCATION[0], GROUND_TRUTH_LOCATION[1])
) + """;

// Preload satellite background image
const _satImg = new Image();
let _satImgLoaded = false;
_satImg.onload = function() { _satImgLoaded = true; };
_satImg.onerror = function() { console.warn('Satellite bg not available — using plain background'); };
_satImg.src = '/api/satellite_bg?t=' + Date.now();

function updateDeliverPanel(s) {
  const panel = document.getElementById('deliver-panel');
  const data  = s.deliver_status || null;
  const state = s.current_state || '';

  const show = (state === 'DELIVER') || (state === 'CHANGE_MODE' && data) || (data !== null);
  panel.className = show ? 'visible' : '';
  if (!show) return;

  const st  = data ? (data.status || '') : '';
  const msg = document.getElementById('deliver-msg');
  const badge = document.getElementById('deliver-badge');

  // Derive phase from status string first (robust), fall back to phase number.
  // deliver.py phases: 1=confirm detection, 2=wait geotag, 3=select landing,
  //   4=approach, 5=land, 6=deploy payload, 7=relaunch & RTH
  const phaseNum = data ? (data.phase || 0) : 0;
  const isPhase1  = st.includes('CONFIRM DETECTION') || phaseNum === 1;
  const isGeotag  = st.includes('WAITING FOR GEOTAG') || phaseNum === 2;
  const isPhase2  = st.includes('SELECT LANDING')     || phaseNum === 3;
  const isPhase3  = (st.includes('APPROACH') && !isPhase1 && !isPhase2 && !isGeotag) || phaseNum === 4;
  const isPhase4  = (st.includes('LANDING') && !isPhase1 && !isPhase2 && !isGeotag)  || phaseNum === 5;
  const isPhase5  = (st.includes('DEPLOY') || st.includes('LANDED'))                 || phaseNum === 6;
  const isPhase6  = (st.includes('RETURNING') || st.includes('ARMING') ||
                     st.includes('CLIMBING')   || st.includes('HOME')   ||
                     st.includes('MISSION COMPLETE') || st.includes('COMPLETE MISSION'))
                     || phaseNum === 7;

  // Section visibility — reset all first
  const detSection    = document.getElementById('det-confirm-section');
  const canvasSection = document.getElementById('landing-canvas-section');
  const telemSection  = document.getElementById('deliver-telem-section');
  const actionButtons = document.getElementById('deliver-action-buttons');
  const btnDeploy     = document.getElementById('btn-confirm-deploy');
  const btnComplete   = document.getElementById('btn-complete-mission');

  detSection.style.display    = 'none';
  canvasSection.style.display = 'none';
  telemSection.style.display  = 'none';
  actionButtons.style.display = 'none';
  btnDeploy.style.display     = 'none';
  btnComplete.style.display   = 'none';

  // Detection target header
  const tgt = document.getElementById('deliver-target');
  if (data && data.detection_lat != null) {
    tgt.textContent = 'Target: ' + data.detection_lat.toFixed(5) + ', ' + data.detection_lon.toFixed(5);
  }

  // Badge
  badge.textContent = st;
  badge.className = 'badge ' +
    (isPhase1                         ? 'init'             :
     isPhase2                         ? 'init'             :
     st.includes('APPROACH')          ? 'flying'           :
     st.includes('LANDING')           ? 'paused'           :
     st.includes('LANDED')            ? 'landed'           :
     st.includes('DEPLOY')            ? 'deploy'           :
     st.includes('CLIMBING')          ? 'paused'           :
     st.includes('RETURNING')         ? 'rth'              :
     st.includes('MISSION COMPLETE')  ? 'mission-complete' :
     st.includes('WAITING')           ? 'paused'           :
     st.includes('PRESS')             ? 'deploy'           :
     st.includes('ERROR') ||
     st.includes('TIMEOUT')           ? 'error'            : 'init');

  // ── Phase 1: Detection confirmation ──
  if (isPhase1) {
    msg.textContent = 'Is this the casualty? Confirm or reject the detection.';
    detSection.style.display = 'block';
    const detFile = data ? data.detection_file : '';
    if (detFile) {
      const img = document.getElementById('det-img');
      const newSrc = '/api/detection_image?file=' + encodeURIComponent(detFile);
      // Only update src if changed (avoid reload flicker)
      if (!img.src.includes(encodeURIComponent(detFile))) {
        img.src = newSrc + '&t=' + Date.now();
      }
    }
    return;
  }

  // ── Geotag wait: hovering for accurate position fix ──
  if (isGeotag) {
    msg.textContent = 'Hovering — waiting for CV pipeline to produce geotag image ...';
    // Show telemetry while waiting (altitude etc. are useful feedback)
    telemSection.style.display = 'block';
    document.getElementById('d-dist').textContent = '—';
    document.getElementById('d-alt').textContent =
      data && data.alt != null ? data.alt + ' m' : '—';
    document.getElementById('d-pos').textContent =
      data && data.lat != null ? data.lat.toFixed(5) + ', ' + data.lon.toFixed(5) : '—';
    return;
  }

  // ── Phase 2: Landing point selection ──
  if (isPhase2) {
    msg.textContent = 'Select a landing point within the red circle (5–10m from casualty).';
    canvasSection.style.display = 'block';
    // Show ground truth button only if configured
    const gtBtn = document.getElementById('btn-ground-truth');
    if (GROUND_TRUTH && gtBtn) {
      gtBtn.style.display = '';
    }
    if (data && (data.focus_polygon || data.detection_lat != null)) {
      drawOverviewCanvas(data);
      drawLandingCanvas(data);
    }
    return;
  }

  // ── Phase 3+: Telemetry + action buttons ──
  telemSection.style.display = 'block';
  document.getElementById('d-dist').textContent =
    data && data.dist != null ? data.dist + ' m' : '—';
  document.getElementById('d-alt').textContent =
    data && data.alt != null ? data.alt + ' m' : '—';
  document.getElementById('d-pos').textContent =
    data && data.lat != null ? data.lat.toFixed(5) + ', ' + data.lon.toFixed(5) : '—';

  if (isPhase3) {
    msg.textContent = 'Flying slowly to selected landing point ...';
    actionButtons.style.display = 'block';
  } else if (isPhase4) {
    msg.textContent = 'Descending to land ...';
    actionButtons.style.display = 'block';
  } else if (isPhase5) {
    actionButtons.style.display = 'block';
    if (st.includes('DEPLOYED') || st.includes('PRESS')) {
      msg.textContent = 'Payload deployed — press Complete Mission to relaunch and return home.';
      btnComplete.style.display = '';
    } else {
      msg.textContent = 'Landed — deploy payload and confirm.';
      btnDeploy.style.display = '';
    }
  } else if (isPhase6) {
    actionButtons.style.display = 'block';
    if (st.includes('ARMING') || st.includes('CLIMBING')) {
      msg.textContent = 'Relaunching ...';
    } else if (st.includes('RETURNING') || st.includes('HOME')) {
      msg.textContent = 'Returning to take-off location via GUIDED ...';
    } else if (st.includes('LANDING AT HOME')) {
      msg.textContent = 'Landing at home ...';
    } else if (st.includes('MISSION COMPLETE')) {
      msg.textContent = 'Mission complete — drone has landed at home.';
    } else if (st.includes('PRESS')) {
      msg.textContent = 'Payload deployed. Press Complete Mission to relaunch.';
      btnComplete.style.display = '';
    } else if (st.includes('PAUSED')) {
      msg.textContent = 'Paused — pilot has control.';
    } else {
      msg.textContent = st || 'Returning home ...';
    }
  } else {
    msg.textContent = st || 'Initialising ...';
    actionButtons.style.display = 'block';
  }
}

// ── Interactive landing canvas with satellite background + NESW ──

function drawLandingCanvas(data) {
  const canvas = document.getElementById('landing-canvas');
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;

  _canvasData = data;

  const detLat = data.detection_lat;
  const detLon = data.detection_lon;
  const poly   = data.focus_polygon || [];
  const radius = data.standoff_radius_m || STANDOFF_M;
  const nesw   = data.nesw_points || {};

  // Parse polygon — handle both [lat,lon] arrays and {lat,lon} objects
  const polyParsed = poly.map(p => {
    if (Array.isArray(p)) return {lat: p[0], lon: p[1]};
    if (p.lat != null) return {lat: p.lat, lon: p.lon};
    return {lat: 0, lon: 0};
  });

  // Compute geo bounds: polygon + detection + NESW points + padding
  // Then adjust so the aspect ratio matches canvas W:H accounting
  // for cos(lat) — 1° longitude is physically narrower than 1° lat.
  let allLats = [detLat], allLons = [detLon];
  polyParsed.forEach(p => { allLats.push(p.lat); allLons.push(p.lon); });
  Object.values(nesw).forEach(p => { allLats.push(p.lat); allLons.push(p.lon); });

  const minLat = Math.min(...allLats), maxLat = Math.max(...allLats);
  const minLon = Math.min(...allLons), maxLon = Math.max(...allLons);
  const padLat = (maxLat - minLat) * 0.25 || 0.0003;
  const padLon = (maxLon - minLon) * 0.25 || 0.0003;
  let bMinLat = minLat - padLat, bMaxLat = maxLat + padLat;
  let bMinLon = minLon - padLon, bMaxLon = maxLon + padLon;

  const mPerDegLat = 111320;
  const mPerDegLon = 111320 * Math.cos(detLat * Math.PI / 180);

  // Aspect-correct: expand the narrower axis to fill the canvas
  const latSpanM = (bMaxLat - bMinLat) * mPerDegLat;
  const lonSpanM = (bMaxLon - bMinLon) * mPerDegLon;
  const canvasRatio = W / H;
  const geoRatio = lonSpanM / latSpanM;
  if (geoRatio < canvasRatio) {
    // Need wider lon span
    const targetLonM = latSpanM * canvasRatio;
    const extraDeg = (targetLonM - lonSpanM) / mPerDegLon / 2;
    bMinLon -= extraDeg; bMaxLon += extraDeg;
  } else {
    // Need taller lat span
    const targetLatM = lonSpanM / canvasRatio;
    const extraDeg = (targetLatM - latSpanM) / mPerDegLat / 2;
    bMinLat -= extraDeg; bMaxLat += extraDeg;
  }

  function gps2px(lat, lon) {
    const x = ((lon - bMinLon) / (bMaxLon - bMinLon)) * W;
    const y = ((bMaxLat - lat) / (bMaxLat - bMinLat)) * H;
    return [x, y];
  }
  _canvasData._gps2px = gps2px;

  // ── Background: satellite image or dark fallback ──
  ctx.fillStyle = '#0a0a1a';
  ctx.fillRect(0, 0, W, H);
  if (_satImgLoaded) {
    const tl = gps2px(SAT_BOUNDS.north, SAT_BOUNDS.west);
    const br = gps2px(SAT_BOUNDS.south, SAT_BOUNDS.east);
    ctx.drawImage(_satImg, tl[0], tl[1], br[0] - tl[0], br[1] - tl[1]);
    // Semi-transparent overlay so markers pop against the image
    ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
    ctx.fillRect(0, 0, W, H);
  }

  // ── Focus polygon (purple) ──
  if (polyParsed.length >= 3) {
    ctx.beginPath();
    const p0 = gps2px(polyParsed[0].lat, polyParsed[0].lon);
    ctx.moveTo(p0[0], p0[1]);
    for (let i = 1; i < polyParsed.length; i++) {
      const pi = gps2px(polyParsed[i].lat, polyParsed[i].lon);
      ctx.lineTo(pi[0], pi[1]);
    }
    ctx.closePath();
    ctx.fillStyle = 'rgba(124, 77, 255, 0.2)';
    ctx.fill();
    ctx.strokeStyle = '#7c4dff';
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  // ── Standoff circle (red, dashed) ──
  const detPx = gps2px(detLat, detLon);
  const radiusDegLat = radius / mPerDegLat;
  const radiusPxY = (radiusDegLat / (bMaxLat - bMinLat)) * H;
  ctx.beginPath();
  ctx.arc(detPx[0], detPx[1], radiusPxY, 0, Math.PI * 2);
  ctx.setLineDash([6, 4]);
  ctx.strokeStyle = 'rgba(244, 67, 54, 0.7)';
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = 'rgba(244, 67, 54, 0.08)';
  ctx.fill();

  // ── Detection point (casualty marker — orange) ──
  ctx.beginPath();
  ctx.arc(detPx[0], detPx[1], 7, 0, Math.PI * 2);
  ctx.fillStyle = '#ff5722';
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.fillStyle = '#ffab91';
  ctx.font = 'bold 13px monospace';
  ctx.textAlign = 'center';
  ctx.fillText('CASUALTY', detPx[0], detPx[1] - 14);

  // ── NESW landing markers ──
  const neswColors = {N:'#42a5f5', E:'#66bb6a', S:'#ffa726', W:'#ab47bc'};
  const neswLabels = {N:'N', E:'E', S:'S', W:'W'};
  Object.entries(nesw).forEach(([dir, pt]) => {
    const px = gps2px(pt.lat, pt.lon);
    const color = neswColors[dir] || '#fff';
    const isSelected = (_selectedDirection === dir);
    const markerR = isSelected ? 14 : 10;

    // Outer circle
    ctx.beginPath();
    ctx.arc(px[0], px[1], markerR, 0, Math.PI * 2);
    ctx.fillStyle = isSelected ? color : 'rgba(0,0,0,0.6)';
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = isSelected ? 3 : 2;
    ctx.stroke();

    // Direction letter
    ctx.fillStyle = isSelected ? '#000' : color;
    ctx.font = 'bold ' + (isSelected ? '14' : '12') + 'px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(neswLabels[dir], px[0], px[1]);
    ctx.textBaseline = 'alphabetic';  // reset

    // Coordinate label below marker
    ctx.fillStyle = '#ccc';
    ctx.font = '10px monospace';
    const labelY = dir === 'N' ? px[1] - markerR - 6 : px[1] + markerR + 12;
    ctx.fillText(pt.lat.toFixed(6) + ', ' + pt.lon.toFixed(6), px[0], labelY);

    // Selection highlight ring
    if (isSelected) {
      ctx.beginPath();
      ctx.arc(px[0], px[1], markerR + 5, 0, Math.PI * 2);
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  });

  // ── Dashed lines from casualty to each NESW point ──
  ctx.setLineDash([4, 4]);
  ctx.lineWidth = 1;
  Object.entries(nesw).forEach(([dir, pt]) => {
    const px = gps2px(pt.lat, pt.lon);
    ctx.strokeStyle = (_selectedDirection === dir)
      ? (neswColors[dir] || '#fff') : 'rgba(255,255,255,0.2)';
    ctx.beginPath();
    ctx.moveTo(detPx[0], detPx[1]);
    ctx.lineTo(px[0], px[1]);
    ctx.stroke();
  });
  ctx.setLineDash([]);

  // ── Distance + standoff label ──
  ctx.fillStyle = 'rgba(255,255,255,0.5)';
  ctx.font = '11px monospace';
  ctx.textAlign = 'center';
  ctx.fillText(STANDOFF_M.toFixed(1) + 'm standoff', detPx[0], detPx[1] + radiusPxY + 16);

  _canvasDrawn = true;
}

// ── Overview canvas: zoomed out, simple standoff circle + NESW labels ──

function drawOverviewCanvas(data) {
  const canvas = document.getElementById('landing-canvas-overview');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;

  const detLat = data.detection_lat;
  const detLon = data.detection_lon;
  const radius = data.standoff_radius_m || STANDOFF_M;
  const nesw   = data.nesw_points || {};
  const poly   = data.focus_polygon || [];

  // Parse polygon
  const polyParsed = poly.map(p => {
    if (Array.isArray(p)) return {lat: p[0], lon: p[1]};
    if (p.lat != null) return {lat: p.lat, lon: p.lon};
    return {lat: 0, lon: 0};
  });

  // Compute bounds from satellite image extent for full zoomed-out view.
  // Then aspect-correct so the canvas doesn't distort the geography.
  let bMinLat, bMaxLat, bMinLon, bMaxLon;
  if (SAT_BOUNDS && SAT_BOUNDS.north) {
    bMinLat = SAT_BOUNDS.south;
    bMaxLat = SAT_BOUNDS.north;
    bMinLon = SAT_BOUNDS.west;
    bMaxLon = SAT_BOUNDS.east;
  } else {
    let allLats = [detLat], allLons = [detLon];
    polyParsed.forEach(p => { allLats.push(p.lat); allLons.push(p.lon); });
    Object.values(nesw).forEach(p => { allLats.push(p.lat); allLons.push(p.lon); });
    const minLat = Math.min(...allLats), maxLat = Math.max(...allLats);
    const minLon = Math.min(...allLons), maxLon = Math.max(...allLons);
    const padLat = (maxLat - minLat) * 1.5 || 0.002;
    const padLon = (maxLon - minLon) * 1.5 || 0.002;
    bMinLat = minLat - padLat; bMaxLat = maxLat + padLat;
    bMinLon = minLon - padLon; bMaxLon = maxLon + padLon;
  }

  const mPerDegLat = 111320;
  const mPerDegLon = 111320 * Math.cos(detLat * Math.PI / 180);

  // Aspect-correct: expand the narrower axis to fill the canvas
  const ovLatSpanM = (bMaxLat - bMinLat) * mPerDegLat;
  const ovLonSpanM = (bMaxLon - bMinLon) * mPerDegLon;
  const ovCanvasRatio = W / H;
  const ovGeoRatio = ovLonSpanM / ovLatSpanM;
  if (ovGeoRatio < ovCanvasRatio) {
    const targetLonM = ovLatSpanM * ovCanvasRatio;
    const extraDeg = (targetLonM - ovLonSpanM) / mPerDegLon / 2;
    bMinLon -= extraDeg; bMaxLon += extraDeg;
  } else {
    const targetLatM = ovLonSpanM / ovCanvasRatio;
    const extraDeg = (targetLatM - ovLatSpanM) / mPerDegLat / 2;
    bMinLat -= extraDeg; bMaxLat += extraDeg;
  }

  function gps2px(lat, lon) {
    const x = ((lon - bMinLon) / (bMaxLon - bMinLon)) * W;
    const y = ((bMaxLat - lat) / (bMaxLat - bMinLat)) * H;
    return [x, y];
  }

  // ── Background: satellite image or dark ──
  ctx.fillStyle = '#0a0a1a';
  ctx.fillRect(0, 0, W, H);
  if (_satImgLoaded) {
    const tl = gps2px(SAT_BOUNDS.north, SAT_BOUNDS.west);
    const br = gps2px(SAT_BOUNDS.south, SAT_BOUNDS.east);
    ctx.drawImage(_satImg, tl[0], tl[1], br[0] - tl[0], br[1] - tl[1]);
    ctx.fillStyle = 'rgba(0, 0, 0, 0.25)';
    ctx.fillRect(0, 0, W, H);
  }

  // ── Focus polygon outline (purple, subtle) ──
  if (polyParsed.length >= 3) {
    ctx.beginPath();
    const p0 = gps2px(polyParsed[0].lat, polyParsed[0].lon);
    ctx.moveTo(p0[0], p0[1]);
    for (let i = 1; i < polyParsed.length; i++) {
      const pi = gps2px(polyParsed[i].lat, polyParsed[i].lon);
      ctx.lineTo(pi[0], pi[1]);
    }
    ctx.closePath();
    ctx.fillStyle = 'rgba(124, 77, 255, 0.15)';
    ctx.fill();
    ctx.strokeStyle = '#7c4dff';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  // ── Standoff circle (solid white) ──
  const detPx = gps2px(detLat, detLon);
  const radiusDegLat = radius / mPerDegLat;
  const radiusPxY = (radiusDegLat / (bMaxLat - bMinLat)) * H;
  ctx.beginPath();
  ctx.arc(detPx[0], detPx[1], radiusPxY, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
  ctx.lineWidth = 2;
  ctx.stroke();

  // ── Casualty cross (white X) ──
  const crossSize = 6;
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(detPx[0] - crossSize, detPx[1] - crossSize);
  ctx.lineTo(detPx[0] + crossSize, detPx[1] + crossSize);
  ctx.moveTo(detPx[0] + crossSize, detPx[1] - crossSize);
  ctx.lineTo(detPx[0] - crossSize, detPx[1] + crossSize);
  ctx.stroke();

  // ── NESW labels on the circle ──
  const neswColors = {N:'#42a5f5', E:'#66bb6a', S:'#ffa726', W:'#ab47bc'};
  const positions = {
    N: gps2px(detLat + radiusDegLat, detLon),
    S: gps2px(detLat - radiusDegLat, detLon),
    E: gps2px(detLat, detLon + radius / mPerDegLon),
    W: gps2px(detLat, detLon - radius / mPerDegLon),
  };
  // Label offsets to sit just outside the circle
  const labelOffsets = {N:[0,-12], S:[0,16], E:[14,4], W:[-14,4]};

  Object.entries(positions).forEach(([dir, px]) => {
    const color = neswColors[dir];
    const isSelected = (_selectedDirection === dir);

    // Dot on circle
    ctx.beginPath();
    ctx.arc(px[0], px[1], isSelected ? 6 : 4, 0, Math.PI * 2);
    ctx.fillStyle = isSelected ? color : 'rgba(255,255,255,0.8)';
    ctx.fill();
    if (isSelected) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Letter label
    const off = labelOffsets[dir];
    ctx.fillStyle = isSelected ? color : '#fff';
    ctx.font = 'bold ' + (isSelected ? '14' : '12') + 'px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(dir, px[0] + off[0], px[1] + off[1]);
  });

  // ── Label: standoff distance ──
  ctx.fillStyle = 'rgba(255,255,255,0.5)';
  ctx.font = '10px monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'alphabetic';
  ctx.fillText(STANDOFF_M.toFixed(1) + 'm standoff', detPx[0], detPx[1] + radiusPxY + 24);
}

// ── NESW button selection handler ──
function selectNESW(dir) {
  if (!_canvasData || !_canvasData.nesw_points) return;
  const pt = _canvasData.nesw_points[dir];
  if (!pt) return;

  _selectedDirection = dir;
  _selectedLanding = {lat: pt.lat, lon: pt.lon};

  // Highlight the selected button, reset others
  ['N','E','S','W'].forEach(d => {
    const btn = document.getElementById('btn-nesw-' + d);
    if (btn) {
      if (d === dir) {
        btn.style.background = '#4caf50';
        btn.style.boxShadow = '0 0 12px rgba(76, 175, 80, 0.5)';
      } else {
        btn.style.background = '#1565c0';
        btn.style.boxShadow = 'none';
      }
    }
  });

  // Update coordinate readout
  document.getElementById('landing-coords').textContent =
    dir + ': ' + pt.lat.toFixed(6) + ', ' + pt.lon.toFixed(6) +
    ' (' + STANDOFF_M.toFixed(1) + 'm ' + dir + ' of casualty)';

  // Enable confirm button
  document.getElementById('btn-confirm-landing').disabled = false;

  // Redraw both canvases with selection highlighted
  drawOverviewCanvas(_canvasData);
  drawLandingCanvas(_canvasData);
}

setInterval(poll, 500);
poll();
</script>
</body>
</html>"""
