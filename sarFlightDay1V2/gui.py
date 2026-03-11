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
    POST /api/guided_takeoff        Arm + takeoff to search alt (optional/SITL)
"""

from flask import Flask, jsonify, request, send_file
from config import FLIGHT_MODES
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
    """Arm in GUIDED and climb to search altitude (SITL / optional)."""
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
    <!-- Guided Takeoff (optional — shown when disarmed on ground) -->
    <div id="takeoff-section" style="margin-top:12px; display:flex; gap:10px; align-items:center; flex-wrap:wrap;">
      <button class="action-btn btn-amber" id="btn-takeoff" onclick="guidedTakeoff()">
        ↑ Guided Takeoff (SITL)
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
      ⚠ PLB has been activated. PLB flag stored. Future: REPLAN state.
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
      <span class="pf-label">Mode = LOITER or GUIDED</span>
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

      <input type="checkbox" id="pf-wp-cb" class="pf-check"
             onchange="toggleCheck('wp_verified')">
      <span class="pf-label" style="cursor:pointer;"
            onclick="document.getElementById('pf-wp-cb').click()">
        RC calibrated &amp; waypoints verified
      </span>
      <span class="pf-detail">operator</span>

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

  // PLB notice
  document.getElementById('plb-notice').style.display =
    idle.plb_triggered ? 'block' : 'none';

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
  // Manual checkboxes
  const wpCb    = document.getElementById('pf-wp-cb');
  const pilotCb = document.getElementById('pf-pilot-cb');
  if (checks.wp_verified)  wpCb.checked    = checks.wp_verified.pass;
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
  if (!confirm('Confirm PLB activation. The drone will switch to LOITER and return to IDLE. Continue?')) return;
  post('/api/plb_activated');
}

setInterval(poll, 500);
poll();
</script>
</body>
</html>"""
