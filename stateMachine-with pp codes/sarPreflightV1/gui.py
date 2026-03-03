"""
gui.py — Web dashboard for the SAR state machine.
==================================================
Run by main.py in the main thread.
Open in browser at http://<pi-ip>:5000

Endpoints:
    GET  /              → Dashboard page
    GET  /api/status    → JSON status (polled by browser)
    POST /api/command   → Send a mode-change command
"""

from flask import Flask, jsonify, request
from config import FLIGHT_MODES
from states import STATE_CLASSES

app = Flask(__name__)
shared_status = None  # set by main.py before starting


def create_app(shared):
    """Configure the Flask app with the shared status object."""
    global shared_status
    shared_status = shared
    return app


# ── API endpoints ──────────────────────────────────────────────

@app.route("/api/status")
def api_status():
    """Return current status as JSON (polled every 500ms by browser)."""
    status = shared_status.get_status()
    status["registered_states"] = list(STATE_CLASSES.keys())
    status["flight_modes"] = list(FLIGHT_MODES.keys())
    return jsonify(status)


@app.route("/api/command", methods=["POST"])
def api_command():
    """Receive a mode-change command from the GUI."""
    data = request.get_json()
    mode = data.get("mode", "").upper()

    if mode not in FLIGHT_MODES:
        return jsonify({"error": f"Unknown mode: {mode}"}), 400

    shared_status.send_command({"type": "mode_change", "mode": mode})
    return jsonify({"ok": True, "mode": mode})


@app.route("/api/timed_hold", methods=["POST"])
def api_timed_hold():
    """Receive a timed hold command from the GUI."""
    data = request.get_json()
    mode = data.get("mode", "").upper()
    duration = data.get("duration", 10)

    if mode not in FLIGHT_MODES:
        return jsonify({"error": f"Unknown mode: {mode}"}), 400

    shared_status.send_command({
        "type": "timed_hold",
        "mode": mode,
        "duration": int(duration),
    })
    return jsonify({"ok": True, "mode": mode, "duration": duration})


@app.route("/api/upload_mission", methods=["POST"])
def api_upload_mission():
    """Receive a list of waypoints to upload from the GUI."""
    data = request.get_json()
    waypoints = data.get("waypoints", [])

    if not waypoints:
        return jsonify({"error": "No waypoints provided"}), 400

    # Validate each waypoint
    for i, wp in enumerate(waypoints):
        try:
            lat = float(wp.get("lat"))
            lon = float(wp.get("lon"))
            alt = float(wp.get("alt"))
        except (TypeError, ValueError):
            return jsonify({"error": f"WP {i+1}: lat, lon, alt must be numbers"}), 400

        if not (-90 <= lat <= 90):
            return jsonify({"error": f"WP {i+1}: latitude {lat} out of range"}), 400
        if not (-180 <= lon <= 180):
            return jsonify({"error": f"WP {i+1}: longitude {lon} out of range"}), 400
        if not (0 <= alt <= 50):
            return jsonify({"error": f"WP {i+1}: altitude {alt}m outside 0-50m (R04)"}), 400

    shared_status.send_command({
        "type": "upload_mission",
        "waypoints": waypoints,
    })
    return jsonify({"ok": True, "count": len(waypoints)})


@app.route("/api/upload_fence", methods=["POST"])
def api_upload_fence():
    """Upload geofences from the KML file."""
    data = request.get_json() or {}
    kml_path = data.get("kml_path", None)

    cmd = {"type": "upload_fence"}
    if kml_path:
        cmd["kml_path"] = kml_path

    shared_status.send_command(cmd)
    return jsonify({"ok": True})


@app.route("/api/generate_pattern", methods=["POST"])
def api_generate_pattern():
    """Trigger search pattern generation from KML."""
    shared_status.send_command({"type": "generate_pattern"})
    return jsonify({"ok": True})


@app.route("/api/confirm_pattern", methods=["POST"])
def api_confirm_pattern():
    """Confirm the staged pattern and upload to autopilot."""
    shared_status.send_command({"type": "confirm_pattern"})
    return jsonify({"ok": True})


@app.route("/api/cancel_pattern", methods=["POST"])
def api_cancel_pattern():
    """Cancel the staged pattern."""
    shared_status.send_command({"type": "cancel_pattern"})
    return jsonify({"ok": True})


@app.route("/api/pattern_preview")
def api_pattern_preview():
    """Serve the pattern preview PNG image."""
    import os
    from flask import send_file
    from config import PATTERN_PREVIEW_PATH

    if os.path.exists(PATTERN_PREVIEW_PATH):
        return send_file(PATTERN_PREVIEW_PATH, mimetype="image/png")
    return jsonify({"error": "No preview available"}), 404


@app.route("/api/pattern_status")
def api_pattern_status():
    """Return current pattern generation status."""
    status = shared_status.get_status()
    return jsonify({
        "pattern_status": status.get("pattern_status", "idle"),
        "pending_pattern_info": status.get("pending_pattern_info"),
    })


@app.route("/api/preflight_toggle", methods=["POST"])
def api_preflight_toggle():
    """Toggle a manual preflight checkbox."""
    data = request.get_json() or {}
    field = data.get("field")
    if field not in ("wp_verified", "pilot_ready"):
        return jsonify({"error": f"Unknown field: {field}"}), 400
    shared_status.send_command({"type": "preflight_toggle", "field": field})
    return jsonify({"ok": True, "field": field})


@app.route("/api/start_search", methods=["POST"])
def api_start_search():
    """Start the SEARCH state (only if preflight checks pass)."""
    shared_status.send_command({"type": "start_search"})
    return jsonify({"ok": True})


# ── Dashboard page ─────────────────────────────────────────────

@app.route("/")
def dashboard():
    return DASHBOARD_HTML


# ── Inline HTML/CSS/JS ─────────────────────────────────────────

DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SAR Drone — Dashboard</title>
<style>
    * { margin: 0; padding: 0; box-sizing: border-box; }

    body {
        font-family: -apple-system, 'Segoe UI', Roboto, monospace;
        background: #1a1a2e;
        color: #e0e0e0;
        min-height: 100vh;
        padding: 24px;
    }

    h1 {
        font-size: 20px;
        font-weight: 600;
        margin-bottom: 8px;
        color: #fff;
    }

    .status-bar {
        display: flex;
        align-items: center;
        gap: 12px;
        margin-bottom: 24px;
        font-size: 13px;
        color: #888;
    }

    .status-dot {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        background: #555;
    }

    .status-dot.connected { background: #4caf50; }
    .status-dot.disconnected { background: #f44336; }

    .section {
        margin-bottom: 28px;
    }

    .section-label {
        font-size: 11px;
        font-weight: 600;
        text-transform: uppercase;
        letter-spacing: 1.5px;
        color: #666;
        margin-bottom: 10px;
    }

    .button-row {
        display: flex;
        flex-wrap: wrap;
        gap: 8px;
    }

    .btn {
        padding: 12px 22px;
        border: 2px solid #333;
        border-radius: 8px;
        background: #2a2a3e;
        color: #888;
        font-family: inherit;
        font-size: 13px;
        font-weight: 600;
        letter-spacing: 0.5px;
        cursor: pointer;
        transition: all 0.2s ease;
        user-select: none;
    }

    .btn:hover {
        border-color: #555;
        color: #bbb;
    }

    /* ── State colours ── */
    .btn.state-active {
        background: #1b5e20;
        border-color: #4caf50;
        color: #fff;
        box-shadow: 0 0 12px rgba(76, 175, 80, 0.3);
    }

    .btn.state-transitioning {
        background: #e65100;
        border-color: #ff9800;
        color: #fff;
        box-shadow: 0 0 12px rgba(255, 152, 0, 0.3);
        animation: pulse 1s ease-in-out infinite;
    }

    /* ── Mode colours ── */
    .btn.mode-active {
        background: #0d47a1;
        border-color: #42a5f5;
        color: #fff;
        box-shadow: 0 0 12px rgba(66, 165, 245, 0.3);
    }

    .btn.mode-btn:hover {
        border-color: #42a5f5;
        color: #42a5f5;
    }

    @keyframes pulse {
        0%, 100% { opacity: 1; }
        50% { opacity: 0.6; }
    }

    .message-box {
        margin-top: 24px;
        padding: 14px 18px;
        background: #16213e;
        border: 1px solid #333;
        border-radius: 8px;
        font-size: 13px;
        font-family: 'SF Mono', 'Fira Code', monospace;
        color: #aaa;
        min-height: 44px;
        display: flex;
        align-items: center;
    }

    .message-box .label {
        color: #555;
        margin-right: 10px;
    }

    /* ── Timed Hold panel ── */
    .timed-hold-panel {
        background: #16213e;
        border: 1px solid #333;
        border-radius: 8px;
        padding: 16px;
    }

    .th-row {
        display: flex;
        align-items: center;
        gap: 12px;
        margin-bottom: 10px;
    }

    .th-row:last-child { margin-bottom: 0; }

    .th-label {
        font-size: 12px;
        font-weight: 600;
        color: #888;
        min-width: 65px;
    }

    .th-slider {
        flex: 0 0 180px;
        accent-color: #ff9800;
        cursor: pointer;
    }

    .th-value {
        font-size: 13px;
        font-weight: 600;
        color: #ff9800;
        min-width: 36px;
    }

    .btn.th-btn {
        padding: 8px 16px;
        font-size: 12px;
        border-color: #4a3500;
        color: #999;
    }

    .btn.th-btn:hover {
        border-color: #ff9800;
        color: #ff9800;
    }

    .btn.th-btn.th-running {
        background: #e65100;
        border-color: #ff9800;
        color: #fff;
        box-shadow: 0 0 12px rgba(255, 152, 0, 0.3);
        animation: pulse 1s ease-in-out infinite;
    }

    /* ── Waypoint upload panel ── */
    .wp-panel {
        background: #16213e;
        border: 1px solid #333;
        border-radius: 8px;
        padding: 16px;
    }

    .wp-table {
        width: 100%;
        border-collapse: collapse;
        margin-bottom: 12px;
    }

    .wp-table th {
        font-size: 11px;
        font-weight: 600;
        text-transform: uppercase;
        letter-spacing: 1px;
        color: #666;
        text-align: left;
        padding: 4px 8px 8px 8px;
    }

    .wp-table td {
        padding: 3px 4px;
    }

    .wp-table .wp-num {
        font-size: 12px;
        font-weight: 600;
        color: #66bb6a;
        text-align: center;
        width: 30px;
    }

    .wp-input {
        width: 100%;
        padding: 7px 8px;
        background: #0d1b2a;
        border: 1px solid #333;
        border-radius: 5px;
        color: #e0e0e0;
        font-family: 'SF Mono', 'Fira Code', monospace;
        font-size: 13px;
    }

    .wp-input:focus {
        outline: none;
        border-color: #66bb6a;
    }

    .wp-input::placeholder {
        color: #444;
    }

    .wp-actions {
        display: flex;
        align-items: center;
        gap: 8px;
    }

    .btn.wp-btn {
        padding: 8px 18px;
        font-size: 12px;
        border-color: #1b5e20;
        color: #999;
    }

    .btn.wp-btn:hover {
        border-color: #66bb6a;
        color: #66bb6a;
    }

    .btn.wp-btn.wp-upload {
        padding: 10px 24px;
        font-size: 13px;
    }

    .btn.wp-btn.wp-uploading {
        background: #1b5e20;
        border-color: #66bb6a;
        color: #fff;
        box-shadow: 0 0 12px rgba(102, 187, 106, 0.3);
        animation: pulse 1s ease-in-out infinite;
    }

    .btn.wp-remove {
        padding: 4px 8px;
        font-size: 11px;
        border-color: #555;
        color: #666;
        min-width: 24px;
    }

    .btn.wp-remove:hover {
        border-color: #f44336;
        color: #f44336;
    }

    .wp-hint {
        font-size: 11px;
        color: #555;
        margin-top: 8px;
    }

    .wp-count {
        font-size: 12px;
        color: #888;
        margin-left: auto;
    }

    /* ── Preflight checklist ── */
    .pf-dot {
        width: 12px;
        height: 12px;
        border-radius: 50%;
        background: #333;
        display: inline-block;
        border: 1px solid #555;
    }

    .pf-dot.pass { background: #4caf50; border-color: #66bb6a; box-shadow: 0 0 6px rgba(76,175,80,0.4); }
    .pf-dot.fail { background: #f44336; border-color: #ef5350; box-shadow: 0 0 6px rgba(244,67,54,0.4); }

    .pf-label {
        font-size: 13px;
        color: #ccc;
    }

    .pf-detail {
        font-size: 11px;
        font-family: 'SF Mono', 'Fira Code', monospace;
        color: #666;
        text-align: right;
    }

    .pf-checkbox {
        width: 14px;
        height: 14px;
        accent-color: #4caf50;
        cursor: pointer;
    }
</style>
</head>
<body>

<h1>SAR Drone Dashboard</h1>
<div class="status-bar">
    <div class="status-dot" id="connDot"></div>
    <span id="connText">Connecting...</span>
</div>

<div class="section">
    <div class="section-label">State Machine</div>
    <div class="button-row" id="stateButtons"></div>
</div>

<div class="section">
    <div class="section-label">Flight Modes</div>
    <div class="button-row" id="modeButtons"></div>
</div>

<div class="section">
    <div class="section-label">Timed Hold Test</div>
    <div class="timed-hold-panel">
        <div class="th-row">
            <span class="th-label">Duration:</span>
            <input type="range" id="thDuration" min="5" max="60" value="10" class="th-slider">
            <span class="th-value" id="thDurationVal">10s</span>
        </div>
        <div class="th-row">
            <span class="th-label">Mode:</span>
            <div class="button-row" id="thModeButtons"></div>
        </div>
    </div>
</div>

<div class="section">
    <div class="section-label">Mission Upload</div>
    <div class="wp-panel">
        <table class="wp-table">
            <thead>
                <tr>
                    <th>#</th>
                    <th>Latitude</th>
                    <th>Longitude</th>
                    <th>Alt (m)</th>
                    <th></th>
                </tr>
            </thead>
            <tbody id="wpRows"></tbody>
        </table>
        <div class="wp-actions">
            <button class="btn wp-btn" onclick="addWpRow()">+ Add WP</button>
            <button class="btn wp-btn wp-upload" id="wpUploadBtn" onclick="sendMission()">Upload Mission</button>
            <span class="wp-count" id="wpCount">0 waypoints</span>
        </div>
        <div class="wp-hint">
            Trigger file: echo -e "MISSION 30\\n51.423,-2.671\\n51.424,-2.670" &gt; /tmp/sar_trigger.txt
        </div>
    </div>
</div>

<div class="section">
    <div class="section-label">Geofence Upload</div>
    <div class="timed-hold-panel">
        <div class="th-row">
            <span class="th-label" style="min-width:auto;">Upload Flight Area (inclusion) and SSSI (exclusion) fences from KML:</span>
        </div>
        <div class="th-row">
            <button class="btn wp-btn" id="fenceUploadBtn" onclick="sendFence()" style="background:#1b3a1b; border-color:#4caf50; color:#8bc34a;">
                Upload Fences from KML
            </button>
            <span class="wp-count" id="fenceStatus"></span>
        </div>
        <div class="wp-hint">
            Trigger file: echo "FENCE" &gt; /tmp/sar_trigger.txt
        </div>
    </div>
</div>

<div class="section">
    <div class="section-label">Search Pattern (Step 4)</div>
    <div class="timed-hold-panel" id="patternPanel">
        <div class="th-row">
            <span class="th-label" style="min-width:auto;">Generate lawnmower pattern from KML Survey Area:</span>
        </div>
        <div class="th-row">
            <button class="btn wp-btn" id="patternGenBtn" onclick="sendGeneratePattern()" style="background:#1b2a3b; border-color:#42a5f5; color:#90caf9;">
                Generate Search Pattern
            </button>
            <span class="wp-count" id="patternStatus"></span>
        </div>
        <div id="patternPreview" style="display:none; margin-top:12px;">
            <img id="patternImg" src="" style="width:100%; border-radius:6px; border:2px solid #333;" />
            <div style="display:flex; gap:8px; margin-top:10px;">
                <button class="btn" id="patternConfirmBtn" onclick="sendConfirmPattern()" style="background:#1b5e20; border-color:#4caf50; color:#fff; flex:1;">
                    ✓ Confirm &amp; Upload
                </button>
                <button class="btn" id="patternCancelBtn" onclick="sendCancelPattern()" style="background:#4a1a1a; border-color:#f44336; color:#ef9a9a; flex:1;">
                    ✕ Cancel
                </button>
            </div>
        </div>
        <div class="wp-hint">
            Trigger file: echo "PATTERN" &gt; /tmp/sar_trigger.txt
        </div>
    </div>
</div>

<div class="section">
    <div class="section-label">Preflight Checklist (Step 5)</div>
    <div class="timed-hold-panel" id="preflightPanel" style="border-color:#42a5f5;">
        <div style="display:grid; grid-template-columns:auto 1fr auto; gap:4px 12px; align-items:center; font-size:13px;">

            <span id="pf-hb-dot" class="pf-dot"></span>
            <span class="pf-label">Heartbeat</span>
            <span id="pf-hb-detail" class="pf-detail">—</span>

            <span id="pf-gps-dot" class="pf-dot"></span>
            <span class="pf-label">GPS Fix</span>
            <span id="pf-gps-detail" class="pf-detail">—</span>

            <span id="pf-armed-dot" class="pf-dot"></span>
            <span class="pf-label">Armed</span>
            <span id="pf-armed-detail" class="pf-detail">—</span>

            <span id="pf-mode-dot" class="pf-dot"></span>
            <span class="pf-label">Mode (GUIDED/LOITER)</span>
            <span id="pf-mode-detail" class="pf-detail">—</span>

            <span id="pf-alt-dot" class="pf-dot"></span>
            <span class="pf-label">Altitude in range</span>
            <span id="pf-alt-detail" class="pf-detail">—</span>

            <span id="pf-mission-dot" class="pf-dot"></span>
            <span class="pf-label">Mission uploaded</span>
            <span id="pf-mission-detail" class="pf-detail">—</span>

            <span id="pf-fence-dot" class="pf-dot"></span>
            <span class="pf-label">Fences uploaded</span>
            <span id="pf-fence-detail" class="pf-detail">—</span>

            <span id="pf-bat-dot" class="pf-dot"></span>
            <span class="pf-label">Battery</span>
            <span id="pf-bat-detail" class="pf-detail">—</span>

            <input type="checkbox" id="pf-wp-cb" class="pf-checkbox" onchange="togglePreflight('wp_verified')">
            <span class="pf-label" style="cursor:pointer;" onclick="document.getElementById('pf-wp-cb').click()">Waypoints verified (inside Flight Area, outside SSSI)</span>
            <span id="pf-wp-detail" class="pf-detail">operator</span>

            <input type="checkbox" id="pf-pilot-cb" class="pf-checkbox" onchange="togglePreflight('pilot_ready')">
            <span class="pf-label" style="cursor:pointer;" onclick="document.getElementById('pf-pilot-cb').click()">Safety pilot ready</span>
            <span id="pf-pilot-detail" class="pf-detail">operator</span>

        </div>

        <div style="margin-top:14px; display:flex; align-items:center; gap:12px;">
            <button class="btn" id="startSearchBtn" onclick="sendStartSearch()" disabled
                    style="padding:12px 28px; font-size:14px; font-weight:700; letter-spacing:1px;
                           background:#1a1a2e; border-color:#555; color:#555; flex:0 0 auto;">
                START SEARCH
            </button>
            <span id="readinessLabel" style="font-size:13px; font-weight:600; color:#555;">
                NOT READY
            </span>
        </div>
    </div>
</div>

<div class="message-box">
    <span class="label">LOG:</span>
    <span id="lastMessage">Waiting for connection...</span>
</div>

<script>
    let initialized = false;

    // ── Duration slider ──
    const slider = document.getElementById('thDuration');
    const sliderVal = document.getElementById('thDurationVal');
    slider.addEventListener('input', () => {
        sliderVal.textContent = slider.value + 's';
    });

    async function poll() {
        try {
            const res = await fetch('/api/status');
            const s = await res.json();
            update(s);
        } catch (e) {
            document.getElementById('connDot').className = 'status-dot disconnected';
            document.getElementById('connText').textContent = 'GUI disconnected from Pi';
        }
    }

    function update(s) {
        // ── Connection indicator ──
        const dot = document.getElementById('connDot');
        const txt = document.getElementById('connText');
        if (s.connected) {
            dot.className = 'status-dot connected';
            txt.textContent = 'Connected — ' + s.current_mode;
        } else {
            dot.className = 'status-dot disconnected';
            txt.textContent = 'Waiting for heartbeat...';
        }

        // ── Build buttons once ──
        if (!initialized) {
            buildStateButtons(s.registered_states);
            buildModeButtons(s.flight_modes);
            buildTimedHoldButtons(s.flight_modes);
            initialized = true;
        }

        // ── Update state buttons ──
        s.registered_states.forEach(name => {
            const btn = document.getElementById('state-' + name);
            if (!btn) return;
            btn.className = 'btn';
            if (name === s.current_state) {
                btn.classList.add(s.transitioning ? 'state-transitioning' : 'state-active');
            }
        });

        // ── Update mode buttons ──
        s.flight_modes.forEach(name => {
            const btn = document.getElementById('mode-' + name);
            if (!btn) return;
            btn.className = 'btn mode-btn';
            if (name === s.current_mode) {
                btn.classList.add('mode-active');
            }
        });

        // ── Update timed hold buttons ──
        const isHolding = s.current_state === 'TIMED_HOLD';
        s.flight_modes.forEach(name => {
            const btn = document.getElementById('th-' + name);
            if (!btn) return;
            btn.className = 'btn th-btn';
            if (isHolding) {
                btn.classList.add('th-running');
            }
        });

        // ── Update waypoint upload button ──
        const wpBtn = document.getElementById('wpUploadBtn');
        if (wpBtn) {
            wpBtn.className = 'btn wp-btn';
            if (s.current_state === 'UPLOAD_MISSION') {
                wpBtn.classList.add('wp-uploading');
                wpBtn.textContent = 'Uploading...';
            } else {
                wpBtn.textContent = 'Upload';
            }
        }

        // ── Update fence upload button ──
        const fenceBtn = document.getElementById('fenceUploadBtn');
        if (fenceBtn) {
            if (s.current_state === 'UPLOAD_FENCE') {
                fenceBtn.style.borderColor = '#ff9800';
                fenceBtn.style.color = '#ff9800';
                fenceBtn.textContent = 'Uploading Fences...';
            } else {
                fenceBtn.style.borderColor = '#4caf50';
                fenceBtn.style.color = '#8bc34a';
                fenceBtn.textContent = 'Upload Fences from KML';
            }
        }

        // ── Log message ──
        if (s.last_message) {
            document.getElementById('lastMessage').textContent = s.last_message;
        }

        // ── Update pattern panel ──
        updatePatternPanel(s);

        // ── Update preflight checklist ──
        updatePreflightPanel(s);
    }

    function buildStateButtons(states) {
        const row = document.getElementById('stateButtons');
        row.innerHTML = '';
        states.forEach(name => {
            const btn = document.createElement('button');
            btn.className = 'btn';
            btn.id = 'state-' + name;
            btn.textContent = name;
            row.appendChild(btn);
        });
    }

    function buildModeButtons(modes) {
        const row = document.getElementById('modeButtons');
        row.innerHTML = '';
        modes.forEach(name => {
            const btn = document.createElement('button');
            btn.className = 'btn mode-btn';
            btn.id = 'mode-' + name;
            btn.textContent = name;
            btn.onclick = () => sendCommand(name);
            row.appendChild(btn);
        });
    }

    function buildTimedHoldButtons(modes) {
        const row = document.getElementById('thModeButtons');
        row.innerHTML = '';
        modes.forEach(name => {
            const btn = document.createElement('button');
            btn.className = 'btn th-btn';
            btn.id = 'th-' + name;
            btn.textContent = name;
            btn.onclick = () => sendTimedHold(name);
            row.appendChild(btn);
        });
    }

    async function sendCommand(mode) {
        try {
            await fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ mode: mode })
            });
        } catch (e) {
            console.error('Command failed:', e);
        }
    }

    async function sendTimedHold(mode) {
        const duration = parseInt(slider.value);
        try {
            await fetch('/api/timed_hold', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ mode: mode, duration: duration })
            });
        } catch (e) {
            console.error('Timed hold failed:', e);
        }
    }

    // ── Waypoint table management ──
    let wpRowCount = 0;

    function addWpRow(lat, lon, alt) {
        wpRowCount++;
        const tbody = document.getElementById('wpRows');
        const tr = document.createElement('tr');
        tr.id = 'wp-row-' + wpRowCount;
        tr.innerHTML =
            '<td class="wp-num">' + wpRowCount + '</td>' +
            '<td><input type="text" class="wp-input wp-lat" placeholder="51.4230" value="' + (lat || '') + '"></td>' +
            '<td><input type="text" class="wp-input wp-lon" placeholder="-2.6710" value="' + (lon || '') + '"></td>' +
            '<td><input type="text" class="wp-input wp-alt" placeholder="30" value="' + (alt || '') + '" style="width:80px;"></td>' +
            '<td><button class="btn wp-remove" onclick="removeWpRow(this)">x</button></td>';
        tbody.appendChild(tr);
        updateWpCount();
    }

    function removeWpRow(btn) {
        btn.closest('tr').remove();
        renumberWpRows();
        updateWpCount();
    }

    function renumberWpRows() {
        const rows = document.querySelectorAll('#wpRows tr');
        rows.forEach((row, i) => {
            row.querySelector('.wp-num').textContent = i + 1;
        });
    }

    function updateWpCount() {
        const n = document.querySelectorAll('#wpRows tr').length;
        document.getElementById('wpCount').textContent = n + ' waypoint' + (n !== 1 ? 's' : '');
    }

    function getWaypoints() {
        const rows = document.querySelectorAll('#wpRows tr');
        const wps = [];
        rows.forEach(row => {
            const lat = row.querySelector('.wp-lat').value.trim();
            const lon = row.querySelector('.wp-lon').value.trim();
            const alt = row.querySelector('.wp-alt').value.trim();
            if (lat && lon && alt) {
                wps.push({ lat: parseFloat(lat), lon: parseFloat(lon), alt: parseFloat(alt) });
            }
        });
        return wps;
    }

    async function sendMission() {
        const wps = getWaypoints();
        if (wps.length === 0) {
            alert('Add at least one waypoint with lat, lon, and alt.');
            return;
        }
        try {
            const res = await fetch('/api/upload_mission', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ waypoints: wps })
            });
            const data = await res.json();
            if (!res.ok) {
                alert('Error: ' + (data.error || 'Upload failed'));
            }
        } catch (e) {
            console.error('Mission upload failed:', e);
        }
    }

    async function sendFence() {
        const btn = document.getElementById('fenceUploadBtn');
        const status = document.getElementById('fenceStatus');
        btn.textContent = 'Uploading...';
        btn.style.borderColor = '#ff9800';
        btn.style.color = '#ff9800';
        status.textContent = '';
        try {
            const res = await fetch('/api/upload_fence', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({})
            });
            const data = await res.json();
            if (res.ok) {
                status.textContent = 'Command sent — check log';
            } else {
                status.textContent = 'Error: ' + (data.error || 'Failed');
            }
        } catch (e) {
            console.error('Fence upload failed:', e);
            status.textContent = 'Connection error';
        }
        btn.textContent = 'Upload Fences from KML';
        btn.style.borderColor = '#4caf50';
        btn.style.color = '#8bc34a';
    }

    // ── Pattern generation ──

    async function sendGeneratePattern() {
        const btn = document.getElementById('patternGenBtn');
        const status = document.getElementById('patternStatus');
        btn.textContent = 'Generating...';
        btn.style.borderColor = '#ff9800';
        btn.style.color = '#ff9800';
        status.textContent = '';
        document.getElementById('patternPreview').style.display = 'none';
        try {
            await fetch('/api/generate_pattern', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({})
            });
            status.textContent = 'Generating...';
        } catch (e) {
            console.error('Pattern generation failed:', e);
            status.textContent = 'Connection error';
        }
    }

    async function sendConfirmPattern() {
        const status = document.getElementById('patternStatus');
        status.textContent = 'Uploading to autopilot...';
        try {
            await fetch('/api/confirm_pattern', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({})
            });
        } catch (e) {
            console.error('Pattern confirm failed:', e);
            status.textContent = 'Connection error';
        }
    }

    async function sendCancelPattern() {
        const status = document.getElementById('patternStatus');
        document.getElementById('patternPreview').style.display = 'none';
        try {
            await fetch('/api/cancel_pattern', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({})
            });
            status.textContent = 'Cancelled';
        } catch (e) {
            console.error('Pattern cancel failed:', e);
        }
    }

    // ── Preflight checklist ──

    async function togglePreflight(field) {
        try {
            await fetch('/api/preflight_toggle', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ field: field })
            });
        } catch (e) {
            console.error('Preflight toggle failed:', e);
        }
    }

    async function sendStartSearch() {
        try {
            await fetch('/api/start_search', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({})
            });
        } catch (e) {
            console.error('Start search failed:', e);
        }
    }

    function updatePreflightPanel(s) {
        const checks = s.preflight_checks;
        if (!checks) return;

        const items = [
            { id: 'hb',      key: 'heartbeat' },
            { id: 'gps',     key: 'gps' },
            { id: 'armed',   key: 'armed' },
            { id: 'mode',    key: 'mode' },
            { id: 'alt',     key: 'altitude' },
            { id: 'mission', key: 'mission' },
            { id: 'fence',   key: 'fences' },
            { id: 'bat',     key: 'battery' },
        ];

        items.forEach(item => {
            const dot = document.getElementById('pf-' + item.id + '-dot');
            const detail = document.getElementById('pf-' + item.id + '-detail');
            const chk = checks[item.key];
            if (dot && chk) {
                dot.className = 'pf-dot ' + (chk.pass ? 'pass' : 'fail');
            }
            if (detail && chk) {
                detail.textContent = chk.detail || '';
            }
        });

        // Manual checkboxes — sync with server state
        const wpCb = document.getElementById('pf-wp-cb');
        const pilotCb = document.getElementById('pf-pilot-cb');
        if (wpCb && checks.wp_verified) {
            wpCb.checked = checks.wp_verified.pass;
        }
        if (pilotCb && checks.pilot_ready) {
            pilotCb.checked = checks.pilot_ready.pass;
        }

        // Start Search button
        const btn = document.getElementById('startSearchBtn');
        const label = document.getElementById('readinessLabel');
        const ready = checks.mission_ready;

        if (ready) {
            btn.disabled = false;
            btn.style.background = '#1b5e20';
            btn.style.borderColor = '#4caf50';
            btn.style.color = '#fff';
            btn.style.boxShadow = '0 0 12px rgba(76,175,80,0.4)';
            label.textContent = 'ALL CHECKS PASSED — READY';
            label.style.color = '#4caf50';
        } else {
            btn.disabled = true;
            btn.style.background = '#1a1a2e';
            btn.style.borderColor = '#555';
            btn.style.color = '#555';
            btn.style.boxShadow = 'none';
            label.textContent = 'NOT READY';
            label.style.color = '#555';
        }
    }

    function updatePatternPanel(s) {
        const btn = document.getElementById('patternGenBtn');
        const status = document.getElementById('patternStatus');
        const preview = document.getElementById('patternPreview');
        const pStatus = s.pattern_status || 'idle';

        if (pStatus === 'running') {
            btn.textContent = 'Generating...';
            btn.style.borderColor = '#ff9800';
            btn.style.color = '#ff9800';
            status.textContent = 'Generating pattern...';
            preview.style.display = 'none';
        } else if (pStatus === 'awaiting_confirmation') {
            btn.textContent = 'Generate Search Pattern';
            btn.style.borderColor = '#42a5f5';
            btn.style.color = '#90caf9';
            const info = s.pending_pattern_info;
            let msg = 'Pattern ready — review below';
            if (info) {
                msg = info.wp_count + ' waypoints';
                if (info.oob_count > 0) {
                    msg += ' — ⚠ ' + info.oob_count + ' OUT OF BOUNDS';
                }
            }
            status.textContent = msg;
            preview.style.display = 'block';
            // Reload preview image (cache-bust)
            document.getElementById('patternImg').src =
                '/api/pattern_preview?t=' + Date.now();
        } else if (pStatus === 'confirmed') {
            btn.textContent = 'Generate Search Pattern';
            btn.style.borderColor = '#42a5f5';
            btn.style.color = '#90caf9';
            status.textContent = 'Pattern confirmed — uploading';
            preview.style.display = 'none';
        } else if (pStatus === 'cancelled') {
            btn.textContent = 'Generate Search Pattern';
            btn.style.borderColor = '#42a5f5';
            btn.style.color = '#90caf9';
            status.textContent = 'Cancelled';
            preview.style.display = 'none';
        } else if (pStatus.startsWith && pStatus.startsWith('error:')) {
            btn.textContent = 'Generate Search Pattern';
            btn.style.borderColor = '#f44336';
            btn.style.color = '#ef9a9a';
            status.textContent = pStatus;
            preview.style.display = 'none';
        } else {
            // idle
            btn.textContent = 'Generate Search Pattern';
            btn.style.borderColor = '#42a5f5';
            btn.style.color = '#90caf9';
        }
    }

    // Start with one empty row
    addWpRow();

    // Poll every 500ms
    setInterval(poll, 500);
    poll();
</script>

</body>
</html>
"""