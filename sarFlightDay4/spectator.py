"""
spectator.py — Read-only spectator display for demo day.
=========================================================
Completely separate process from the operator system.
Polls the operator GUI's /api/status endpoint over HTTP
and renders a non-interactive live mission display.

Usage:
    python3 spectator.py                           # defaults (localhost)
    python3 spectator.py --operator 192.168.1.42   # Pi IP
    python3 spectator.py --port 5001               # spectator port

Architecture:
    spectator browser  ──>  spectator Flask (port 5001)
                                  │
                                  │  server-side HTTP fetch
                                  ▼
                            operator Flask (port 5000) /api/status

    The spectator's backend proxies the status request so the
    spectator browser never talks to the operator directly.
    This means zero CORS changes to the operator code.

    If anything in this file crashes, the operator system and
    state machine are completely unaffected — they don't know
    this process exists.
"""

import argparse
import json
import urllib.request
import urllib.error
from flask import Flask, jsonify, Response

# ── Configuration ─────────────────────────────────────────────

DEFAULT_OPERATOR_HOST = "127.0.0.1"
DEFAULT_OPERATOR_PORT = 5000
DEFAULT_SPECTATOR_PORT = 5001

# ── Flask app ─────────────────────────────────────────────────

app = Flask(__name__)
_operator_url = None  # set in main()


@app.route("/")
def index():
    from flask import make_response
    resp = make_response(SPECTATOR_HTML)
    resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp


@app.route("/api/status")
def proxy_status():
    """
    Fetch status from the operator GUI and relay it.
    If the operator is unreachable, return an error payload
    so the spectator JS can show a 'waiting' state.
    """
    try:
        req = urllib.request.Request(_operator_url, method="GET")
        with urllib.request.urlopen(req, timeout=4) as resp:
            data = json.loads(resp.read().decode())
        return jsonify(data)
    except (urllib.error.URLError, OSError, json.JSONDecodeError, Exception):
        return jsonify({"_proxy_error": True, "connected": False})


@app.route("/api/satellite_bg")
def proxy_satellite():
    """Proxy the satellite background image from the operator."""
    try:
        base = _operator_url.rsplit("/api/status", 1)[0]
        url = base + "/api/satellite_bg"
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = resp.read()
            content_type = resp.headers.get("Content-Type", "image/png")
        return Response(data, mimetype=content_type)
    except Exception:
        # Return a 1x1 transparent PNG so the canvas doesn't break
        return Response(b"", status=404)


# ── Spectator HTML ────────────────────────────────────────────

SPECTATOR_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SAR Mission — Live</title>
<style>
/* ═══════════════════════════════════════════════════════════════
   RESET & BASE
   ═══════════════════════════════════════════════════════════════ */
*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

:root {
  --bg:          #080c14;
  --panel:       #0f1520;
  --panel-border:#1a2435;
  --text:        #c8d6e0;
  --text-dim:    #5a7080;
  --text-muted:  #3a4a58;
  --accent:      #3b9eff;
  --green:       #34d399;
  --orange:      #f59e0b;
  --red:         #ef4444;
  --purple:      #a78bfa;
}

body {
  background: var(--bg);
  color: var(--text);
  font-family: 'Inter', 'Segoe UI', system-ui, -apple-system, sans-serif;
  height: 100vh;
  overflow: hidden;
  display: grid;
  grid-template-rows: auto 1fr auto;
  grid-template-columns: 1fr;
}

/* ═══════════════════════════════════════════════════════════════
   TOP BAR
   ═══════════════════════════════════════════════════════════════ */
.topbar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0.7rem 1.5rem;
  background: var(--panel);
  border-bottom: 1px solid var(--panel-border);
  z-index: 10;
}
.topbar-left {
  display: flex;
  align-items: center;
  gap: 1rem;
}
.topbar-title {
  font-size: 1rem;
  font-weight: 700;
  letter-spacing: 0.06em;
  text-transform: uppercase;
  color: var(--accent);
}
.topbar-team {
  font-size: 0.75rem;
  color: var(--text-dim);
  padding-left: 1rem;
  border-left: 1px solid var(--panel-border);
}
.conn-pill {
  display: flex;
  align-items: center;
  gap: 0.45rem;
  font-size: 0.75rem;
  color: var(--text-dim);
}
.conn-pill .dot {
  width: 8px; height: 8px;
  border-radius: 50%;
  background: var(--red);
  transition: background 0.3s;
}
.conn-pill .dot.on { background: var(--green); box-shadow: 0 0 6px rgba(52,211,153,.5); }
.topbar-right {
  display: flex;
  align-items: center;
  gap: 1.5rem;
}
.clock {
  font-size: 0.8rem;
  font-variant-numeric: tabular-nums;
  color: var(--text-dim);
  font-family: 'Consolas', 'Fira Code', monospace;
}

/* ═══════════════════════════════════════════════════════════════
   MAIN AREA — map left, panels right
   ═══════════════════════════════════════════════════════════════ */
.main {
  display: grid;
  grid-template-columns: 1fr 340px;
  gap: 0;
  overflow: hidden;
}

/* ── Map ─────────────────────────────────────────────────────── */
.map-wrap {
  position: relative;
  background: #060a10;
  border-right: 1px solid var(--panel-border);
}
.map-wrap canvas {
  width: 100%;
  height: 100%;
  display: block;
}

/* Map overlays */
.map-state-badge {
  position: absolute;
  top: 1rem;
  left: 1rem;
  background: rgba(15,21,32,.85);
  border: 1px solid var(--panel-border);
  border-radius: 8px;
  padding: 0.6rem 1rem;
  backdrop-filter: blur(8px);
}
.map-state-badge .state-name {
  font-size: 1.8rem;
  font-weight: 800;
  letter-spacing: 0.02em;
  line-height: 1;
}
.map-state-badge .state-desc {
  font-size: 0.75rem;
  color: var(--text-dim);
  margin-top: 0.25rem;
}

.map-legend {
  position: absolute;
  bottom: 1rem;
  left: 1rem;
  background: rgba(15,21,32,.85);
  border: 1px solid var(--panel-border);
  border-radius: 6px;
  padding: 0.5rem 0.8rem;
  font-size: 0.65rem;
  color: var(--text-dim);
  display: flex;
  gap: 0.8rem;
  backdrop-filter: blur(8px);
}
.map-legend .leg { display: flex; align-items: center; gap: 0.3rem; }
.map-legend .swatch {
  width: 10px; height: 3px; border-radius: 1px;
}

/* ── Right panels ────────────────────────────────────────────── */
.sidebar {
  display: flex;
  flex-direction: column;
  background: var(--panel);
  overflow-y: auto;
  padding: 0;
}

.panel {
  padding: 1rem 1.2rem;
  border-bottom: 1px solid var(--panel-border);
}
.panel-label {
  font-size: 0.65rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.1em;
  color: var(--text-muted);
  margin-bottom: 0.6rem;
}

/* Timeline */
.timeline {
  display: flex;
  gap: 2px;
}
.tl-step {
  flex: 1;
  text-align: center;
  padding: 0.35rem 0;
  font-size: 0.55rem;
  font-weight: 600;
  letter-spacing: 0.04em;
  text-transform: uppercase;
  color: var(--text-muted);
  background: rgba(26,36,53,.5);
  border-radius: 3px;
  transition: all 0.4s;
}
.tl-step.done { background: rgba(52,211,153,.15); color: var(--green); }
.tl-step.active { background: rgba(59,158,255,.2); color: var(--accent); box-shadow: 0 0 8px rgba(59,158,255,.15); }
.tl-step.complete { background: rgba(52,211,153,.25); color: var(--green); box-shadow: 0 0 8px rgba(52,211,153,.2); }

/* Telemetry grid */
.telem-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 0.5rem;
}
.telem-cell {
  background: rgba(26,36,53,.4);
  border-radius: 8px;
  padding: 0.6rem 0.7rem;
}
.telem-cell .label {
  font-size: 0.6rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.08em;
  color: var(--text-muted);
}
.telem-cell .val {
  font-size: 1.5rem;
  font-weight: 700;
  font-variant-numeric: tabular-nums;
  color: var(--text);
  line-height: 1.2;
}
.telem-cell .val .unit {
  font-size: 0.7rem;
  font-weight: 400;
  color: var(--text-dim);
}

/* Battery */
.bat-row {
  display: flex;
  align-items: center;
  gap: 0.6rem;
}
.bat-track {
  flex: 1;
  height: 12px;
  background: rgba(26,36,53,.6);
  border-radius: 6px;
  overflow: hidden;
}
.bat-fill {
  height: 100%;
  border-radius: 6px;
  background: var(--green);
  transition: width 0.8s ease, background 0.5s;
}
.bat-fill.mid { background: var(--orange); }
.bat-fill.low { background: var(--red); }
.bat-pct {
  font-size: 1.1rem;
  font-weight: 700;
  font-variant-numeric: tabular-nums;
  min-width: 3rem;
  text-align: right;
}

/* Progress */
.prog-row { margin-top: 0.4rem; }
.prog-track {
  height: 6px;
  background: rgba(26,36,53,.6);
  border-radius: 3px;
  overflow: hidden;
}
.prog-fill {
  height: 100%;
  border-radius: 3px;
  background: var(--accent);
  transition: width 0.8s ease;
}
.prog-fill.focus { background: var(--purple); }
.prog-text {
  font-size: 0.7rem;
  color: var(--text-dim);
  margin-top: 0.2rem;
}

/* Log */
.log-box {
  font-family: 'Consolas', 'Fira Code', monospace;
  font-size: 0.7rem;
  color: var(--text-dim);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  padding: 0.5rem 0;
}

/* ═══════════════════════════════════════════════════════════════
   DETECTION ALERT OVERLAY
   ═══════════════════════════════════════════════════════════════ */
.detection-overlay {
  display: none;
  position: fixed;
  bottom: 0; left: 0; right: 0;
  z-index: 100;
  padding: 1rem 2rem;
  background: linear-gradient(180deg, rgba(8,12,20,0) 0%, rgba(245,158,11,.12) 100%);
  border-top: 2px solid var(--orange);
  backdrop-filter: blur(8px);
  text-align: center;
  animation: slideUp 0.5s ease-out;
}
.detection-overlay.show { display: block; }
.detection-overlay h3 {
  font-size: 1.3rem;
  font-weight: 800;
  color: var(--orange);
  letter-spacing: 0.1em;
  text-transform: uppercase;
  animation: pulse-text 1.5s ease-in-out infinite;
}
.detection-overlay .det-coords {
  font-family: monospace;
  font-size: 0.85rem;
  color: var(--text-dim);
  margin-top: 0.2rem;
}
@keyframes slideUp {
  from { transform: translateY(100%); opacity: 0; }
  to   { transform: translateY(0);    opacity: 1; }
}
@keyframes pulse-text {
  0%, 100% { opacity: 1; }
  50%      { opacity: 0.6; }
}

/* ═══════════════════════════════════════════════════════════════
   PAYLOAD DEPLOYED OVERLAY
   ═══════════════════════════════════════════════════════════════ */
.deploy-overlay {
  display: none;
  position: fixed;
  bottom: 0; left: 0; right: 0;
  z-index: 100;
  padding: 1rem 2rem;
  background: linear-gradient(180deg, rgba(8,12,20,0) 0%, rgba(52,211,153,.12) 100%);
  border-top: 2px solid var(--green);
  backdrop-filter: blur(8px);
  text-align: center;
  animation: slideUp 0.5s ease-out;
}
.deploy-overlay.show { display: block; }
.deploy-overlay h3 {
  font-size: 1.3rem;
  font-weight: 800;
  color: var(--green);
  letter-spacing: 0.1em;
  text-transform: uppercase;
  animation: pulse-text 1.5s ease-in-out infinite;
}
.deploy-overlay .deploy-detail {
  font-family: monospace;
  font-size: 0.85rem;
  color: var(--text-dim);
  margin-top: 0.2rem;
}

/* ═══════════════════════════════════════════════════════════════
   RETURNING HOME OVERLAY
   ═══════════════════════════════════════════════════════════════ */
.rth-overlay {
  display: none;
  position: fixed;
  bottom: 0; left: 0; right: 0;
  z-index: 100;
  padding: 1rem 2rem;
  background: linear-gradient(180deg, rgba(8,12,20,0) 0%, rgba(59,158,255,.12) 100%);
  border-top: 2px solid var(--accent);
  backdrop-filter: blur(8px);
  text-align: center;
  animation: slideUp 0.5s ease-out;
}
.rth-overlay.show { display: block; }
.rth-overlay h3 {
  font-size: 1.3rem;
  font-weight: 800;
  color: var(--accent);
  letter-spacing: 0.1em;
  text-transform: uppercase;
  animation: pulse-text 1.5s ease-in-out infinite;
}
.rth-overlay .rth-detail {
  font-family: monospace;
  font-size: 0.85rem;
  color: var(--text-dim);
  margin-top: 0.2rem;
}

/* ═══════════════════════════════════════════════════════════════
   MISSION COMPLETE OVERLAY
   ═══════════════════════════════════════════════════════════════ */
.complete-overlay {
  display: none;
  position: fixed;
  inset: 0;
  z-index: 200;
  background: rgba(8,12,20,.92);
  backdrop-filter: blur(12px);
  justify-content: center;
  align-items: center;
  flex-direction: column;
  animation: fadeIn 0.8s ease-out;
}
.complete-overlay.show { display: flex; }
.complete-overlay h2 {
  font-size: 3rem;
  font-weight: 800;
  color: var(--green);
  letter-spacing: 0.08em;
  text-transform: uppercase;
  margin-bottom: 0.5rem;
}
.complete-overlay .complete-sub {
  font-size: 1rem;
  color: var(--text-dim);
  margin-bottom: 2.5rem;
}
.summary-grid {
  display: grid;
  grid-template-columns: repeat(3, 180px);
  gap: 1rem;
}
.summary-card {
  background: rgba(26,36,53,.6);
  border: 1px solid var(--panel-border);
  border-radius: 10px;
  padding: 1rem;
  text-align: center;
}
.summary-card .s-label {
  font-size: 0.65rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.08em;
  color: var(--text-muted);
  margin-bottom: 0.3rem;
}
.summary-card .s-val {
  font-size: 1.8rem;
  font-weight: 700;
  color: var(--text);
  font-variant-numeric: tabular-nums;
}
.summary-card .s-unit {
  font-size: 0.75rem;
  color: var(--text-dim);
}

/* State log in summary */
.state-log {
  margin-top: 2rem;
  display: flex;
  gap: 0.4rem;
  align-items: center;
}
.state-log .sl-item {
  display: flex;
  align-items: center;
  gap: 0.3rem;
  font-size: 0.7rem;
  color: var(--text-dim);
}
.state-log .sl-dot {
  width: 6px; height: 6px;
  border-radius: 50%;
  background: var(--green);
}
.state-log .sl-arrow {
  color: var(--text-muted);
  font-size: 0.6rem;
  margin: 0 0.1rem;
}

/* ═══════════════════════════════════════════════════════════════
   BOTTOM BAR
   ═══════════════════════════════════════════════════════════════ */
.bottombar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0.4rem 1.5rem;
  background: var(--panel);
  border-top: 1px solid var(--panel-border);
  font-size: 0.7rem;
  color: var(--text-muted);
  z-index: 10;
}
.bottombar .log-msg {
  font-family: 'Consolas', 'Fira Code', monospace;
  color: var(--text-dim);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  flex: 1;
  margin-right: 1rem;
}

/* ═══════════════════════════════════════════════════════════════
   ANIMATIONS
   ═══════════════════════════════════════════════════════════════ */
@keyframes fadeIn {
  from { opacity: 0; }
  to   { opacity: 1; }
}
@keyframes drone-pulse {
  0%, 100% { r: 5; opacity: 1; }
  50%      { r: 8; opacity: 0.6; }
}
</style>
</head>
<body>

<!-- ═══════════ TOP BAR ═══════════ -->
<div class="topbar">
  <div class="topbar-left">
    <div class="topbar-title">SAR Quadrotor Mission</div>
    <div class="topbar-team">Blue Co</div>
  </div>
  <div class="topbar-right">
    <div class="conn-pill">
      <div class="dot" id="conn-dot"></div>
      <span id="conn-text">Connecting...</span>
    </div>
    <div class="clock" id="clock">--:--:--</div>
  </div>
</div>

<!-- ═══════════ MAIN ═══════════ -->
<div class="main">

  <!-- ── Map ──────────────────────── -->
  <div class="map-wrap">
    <canvas id="map-canvas"></canvas>
    <div class="map-state-badge">
      <div class="state-name" id="state-name">STANDBY</div>
      <div class="state-desc" id="state-desc">Waiting for mission data</div>
    </div>
    <div class="map-legend">
      <div class="leg"><div class="swatch" style="background:#3b9eff"></div> Search pattern</div>
      <div class="leg"><div class="swatch" style="background:#5a7080"></div> SSSI corridor</div>
      <div class="leg"><div class="swatch" style="background:#a78bfa"></div> Focus area</div>
      <div class="leg"><div class="swatch" style="background:var(--orange)"></div> Detection</div>
      <div class="leg"><div class="swatch" style="background:var(--green)"></div> Landing</div>
    </div>
  </div>

  <!-- ── Sidebar ──────────────────── -->
  <div class="sidebar">

    <!-- Timeline -->
    <div class="panel">
      <div class="panel-label">Mission Phase</div>
      <div class="timeline" id="timeline">
        <div class="tl-step" data-states="IDLE,PRE_AUTO_CHECK,GUIDED_TAKEOFF,UPLOAD_FENCE,GENERATE_PATTERN,UPLOAD_MISSION,CHANGE_MODE">Pre</div>
        <div class="tl-step" data-states="SEARCH">Search</div>
        <div class="tl-step" data-states="REPLAN">Replan</div>
        <div class="tl-step" data-states="FOCUS">Focus</div>
        <div class="tl-step" data-states="DELIVER">Deliver</div>
        <div class="tl-step" data-states="SAFE_RTL">RTH</div>
        <div class="tl-step" id="tl-complete" data-states="_COMPLETE">Complete</div>
      </div>
    </div>

    <!-- Telemetry -->
    <div class="panel">
      <div class="panel-label">Telemetry</div>
      <div class="telem-grid">
        <div class="telem-cell">
          <div class="label">Altitude</div>
          <div class="val"><span id="v-alt">--</span> <span class="unit">m</span></div>
        </div>
        <div class="telem-cell">
          <div class="label">Mode</div>
          <div class="val" id="v-mode" style="font-size:1rem">--</div>
        </div>
        <div class="telem-cell">
          <div class="label">Waypoint</div>
          <div class="val" id="v-wp">--</div>
        </div>
        <div class="telem-cell">
          <div class="label">Elapsed</div>
          <div class="val" id="v-elapsed" style="font-size:1.1rem">--</div>
        </div>
        <div class="telem-cell">
          <div class="label">GPS Sats</div>
          <div class="val" id="v-sats">--</div>
        </div>
        <div class="telem-cell">
          <div class="label">Position</div>
          <div class="val" id="v-pos" style="font-size:0.7rem;font-weight:400">--</div>
        </div>
      </div>
    </div>

    <!-- Battery -->
    <div class="panel">
      <div class="panel-label">Battery</div>
      <div class="bat-row">
        <div class="bat-track">
          <div class="bat-fill" id="bat-fill" style="width:0%"></div>
        </div>
        <div class="bat-pct" id="bat-pct">--%</div>
      </div>
    </div>

    <!-- Progress -->
    <div class="panel" id="progress-panel" style="display:none">
      <div class="panel-label" id="prog-label">Search Progress</div>
      <div class="prog-row">
        <div class="prog-track">
          <div class="prog-fill" id="prog-fill" style="width:0%"></div>
        </div>
        <div class="prog-text" id="prog-text">0%</div>
      </div>
    </div>

    <!-- Detection info (appears when found) -->
    <div class="panel" id="det-panel" style="display:none">
      <div class="panel-label">Detection</div>
      <div style="font-size:0.85rem;color:var(--orange);font-weight:600">Casualty Located</div>
      <div id="det-info" style="font-size:0.75rem;color:var(--text-dim);font-family:monospace;margin-top:0.2rem"></div>
    </div>

    <!-- Spacer to push log down -->
    <div style="flex:1"></div>

    <!-- Log in sidebar -->
    <div class="panel" style="border-bottom:none;border-top:1px solid var(--panel-border)">
      <div class="panel-label">System Log</div>
      <div class="log-box" id="log-msg">Waiting for operator system...</div>
    </div>
  </div>
</div>

<!-- ═══════════ BOTTOM BAR ═══════════ -->
<div class="bottombar">
  <div class="log-msg" id="bottom-mode">Mode: --</div>
  <div id="bottom-status">Spectator Display</div>
</div>

<!-- ═══════════ DETECTION OVERLAY ═══════════ -->
<div class="detection-overlay" id="det-overlay">
  <h3>CASUALTY DETECTED</h3>
  <div class="det-coords" id="det-coords"></div>
</div>

<!-- ═══════════ DEPLOY OVERLAY ═══════════ -->
<div class="deploy-overlay" id="deploy-overlay">
  <h3>PAYLOAD DEPLOYED</h3>
  <div class="deploy-detail">First aid kit delivered to casualty</div>
</div>

<!-- ═══════════ RETURNING HOME OVERLAY ═══════════ -->
<div class="rth-overlay" id="rth-overlay">
  <h3>RETURNING TO HOME</h3>
  <div class="rth-detail">Flying SSSI-safe corridor back to launch site</div>
</div>

<!-- ═══════════ MISSION COMPLETE OVERLAY ═══════════ -->
<div class="complete-overlay" id="complete-overlay">
  <h2>MISSION COMPLETE</h2>
  <div class="complete-sub">Drone has returned to home and landed safely</div>
  <div class="summary-grid" id="summary-grid"></div>
  <div class="state-log" id="state-log"></div>
</div>


<script>
/* ═══════════════════════════════════════════════════════════════
   FENSWOOD FARM COORDINATES (from config.py — hardcoded)
   ═══════════════════════════════════════════════════════════════ */
// Full flight-area bounds — tightened to match satellite coverage
// with a small margin so the image fills the canvas well.
const MAP_BOUNDS = {
  north: 51.4258,
  south: 51.4200,
  west:  -2.6762,
  east:  -2.6612,
};

// Satellite background image bounds (from config.py SATELLITE_BOUNDS_*)
const SAT_BOUNDS = {
  north: 51.42510754,
  south: 51.42090293,
  west:  -2.67524776,
  east:  -2.66256298,
};

const SEARCH_WPS = [
  [51.42413284,-2.66885908],[51.42351517,-2.67112058],[51.42331513,-2.67087753],
  [51.42294165,-2.67003302],[51.42339851,-2.66830482],[51.42401931,-2.66877339],
  [51.42346411,-2.67080618],[51.42340627,-2.67073591],[51.42308444,-2.67000820],
  [51.42346309,-2.66857588],[51.42385545,-2.66887202],[51.42342795,-2.67043723],
  [51.42322724,-2.66998338],[51.42352767,-2.66884693],[51.42369158,-2.66897064],
  [51.42340200,-2.67003087],
];

const SSSI_OUT = [
  [51.4234034,-2.6713890],[51.4220921,-2.6697958],
  [51.4226507,-2.6675588],[51.4234937,-2.6681381],
];

const SSSI_HOME = [
  [51.4234937,-2.6681381],[51.4226507,-2.6675588],
  [51.4220921,-2.6697958],[51.4234034,-2.6713890],
];

const HOME_POS = [51.4234034, -2.6713890];  // NAV1 ≈ takeoff point

/* ═══════════════════════════════════════════════════════════════
   STATE
   ═══════════════════════════════════════════════════════════════ */
const STATE_DESC = {
  IDLE:             'System idle — awaiting mission setup',
  UPLOAD_FENCE:     'Uploading geofence boundaries',
  GENERATE_PATTERN: 'Generating search pattern',
  UPLOAD_MISSION:   'Uploading waypoints to autopilot',
  PRE_AUTO_CHECK:   'Running pre-flight safety checks',
  GUIDED_TAKEOFF:   'Taking off to search altitude',
  SEARCH:           'Autonomous search of the designated area',
  REPLAN:           'PLB signal received — replanning to focus area',
  FOCUS:            'Focused camera search for the casualty',
  DELIVER:          'Casualty located — delivering aid package',
  SAFE_RTL:         'Returning to home via safe corridor',
  CHANGE_MODE:      'Transitioning flight mode',
  STARTING:         'System starting up',
};

const FRIENDLY_STATE = {
  IDLE: 'STANDBY', PRE_AUTO_CHECK: 'PREFLIGHT', GUIDED_TAKEOFF: 'TAKEOFF',
  SEARCH: 'SEARCHING', REPLAN: 'REPLANNING', FOCUS: 'FOCUSED SEARCH',
  DELIVER: 'DELIVERING AID', SAFE_RTL: 'RETURNING HOME',
  UPLOAD_FENCE: 'SETUP', GENERATE_PATTERN: 'SETUP', UPLOAD_MISSION: 'SETUP',
  CHANGE_MODE: 'TRANSITIONING', STARTING: 'STARTING',
};

const STATE_COLOR = {
  IDLE: '#c8d6e0', PRE_AUTO_CHECK: '#c8d6e0', GUIDED_TAKEOFF: '#3b9eff',
  SEARCH: '#34d399', REPLAN: '#f59e0b', FOCUS: '#a78bfa',
  DELIVER: '#f59e0b', SAFE_RTL: '#3b9eff', UPLOAD_FENCE: '#c8d6e0',
  GENERATE_PATTERN: '#c8d6e0', UPLOAD_MISSION: '#c8d6e0',
  CHANGE_MODE: '#c8d6e0', STARTING: '#5a7080',
};

let reachedStates = new Set();
let droneTrail = [];          // [{lat, lon, t}] — last N positions
const TRAIL_MAX = 200;
let lastKnownLat = null;     // persists across state transitions
let lastKnownLon = null;
let proxyFailCount = 0;      // don't flash 'No connection' on 1 missed poll
let landingTargetLat = null;  // selected landing point
let landingTargetLon = null;
let detectionShown = false;
let deployShown = false;
let rthShown = false;
let missionComplete = false;

// ── Dynamic view bounds ──
// During FOCUS/DELIVER the map zooms to the focus polygon area
// so markers are large and clear.  Otherwise shows full area.
let currentView = { ...MAP_BOUNDS };  // start with full area
let focusViewComputed = false;
let focusView = null;  // tight bounds around focus polygon

// Satellite background image
let satImg = null;
let satImgLoaded = false;
(function loadSatellite() {
  const img = new Image();
  img.onload = () => { satImg = img; satImgLoaded = true; };
  img.onerror = () => { console.log('Satellite image not available — using grid'); };
  img.src = '/api/satellite_bg';
})();

// Mission tracking for summary
let missionTracker = {
  startBattery: null,
  endBattery: null,
  stateLog: [],              // [{state, time}]
  lastState: null,
  missionStartTime: null,
  detectionLat: null,
  detectionLon: null,
  totalWps: 0,
};

/* ═══════════════════════════════════════════════════════════════
   MAP RENDERING
   ═══════════════════════════════════════════════════════════════ */
const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
  const wrap = canvas.parentElement;
  canvas.width = wrap.clientWidth * devicePixelRatio;
  canvas.height = wrap.clientHeight * devicePixelRatio;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Padding around the map content (pixels)
const MAP_PAD = 30;

function geoToCanvas(lat, lon) {
  // Aspect-ratio-correct projection.
  // At this latitude, 1° longitude is cos(lat) times shorter than 1° latitude.
  // We compute a uniform metres-per-pixel scale and center in the canvas.
  const w = canvas.width / devicePixelRatio;
  const h = canvas.height / devicePixelRatio;
  const drawW = w - MAP_PAD * 2;
  const drawH = h - MAP_PAD * 2;

  const v = currentView;
  const midLat = (v.north + v.south) / 2;
  const cosLat = Math.cos(midLat * Math.PI / 180);

  // Real-world proportional span (in "equatorial-degree" units)
  const realW = (v.east - v.west) * cosLat;
  const realH = v.north - v.south;

  // Uniform scale: fit the geo-bounds into the canvas without stretching
  const scaleW = drawW / realW;
  const scaleH = drawH / realH;
  const scale = Math.min(scaleW, scaleH);

  // Centre the map in the available space
  const usedW = realW * scale;
  const usedH = realH * scale;
  const offX = MAP_PAD + (drawW - usedW) / 2;
  const offY = MAP_PAD + (drawH - usedH) / 2;

  const x = offX + (lon - v.west) * cosLat * scale;
  const y = offY + (v.north - lat) * scale;
  return [x, y];
}

function computeFocusView(focusPoly, detLat, detLon) {
  // Build tight bounds around the focus polygon, detection point,
  // and home position, with generous padding.
  let lats = [], lons = [];
  if (focusPoly && focusPoly.length > 0) {
    focusPoly.forEach(([lat, lon]) => { lats.push(lat); lons.push(lon); });
  }
  if (detLat && detLon) { lats.push(detLat); lons.push(detLon); }
  // Include home so the spectator can see relative position
  lats.push(HOME_POS[0]); lons.push(HOME_POS[1]);

  if (lats.length < 2) return null;

  let minLat = Math.min(...lats), maxLat = Math.max(...lats);
  let minLon = Math.min(...lons), maxLon = Math.max(...lons);

  // Add ~30% padding so markers aren't on the edges
  const latPad = (maxLat - minLat) * 0.3 || 0.0005;
  const lonPad = (maxLon - minLon) * 0.3 || 0.0005;

  return {
    north: maxLat + latPad,
    south: minLat - latPad,
    east:  maxLon + lonPad,
    west:  minLon - lonPad,
  };
}

function updateView(state, s) {
  // Compute the focus view once when the focus polygon first appears
  if (!focusViewComputed) {
    const focusPoly = (s && s.focus_polygon_coords) ||
                      (s && s.deliver_status && s.deliver_status.focus_polygon) ||
                      null;
    const _ds = (typeof s.deliver_status === 'object') ? s.deliver_status : {};
    const _fp = s.focus_progress || {};
    const detLat = _fp.detection_lat ?? _ds.detection_lat ?? null;
    const detLon = _fp.detection_lon ?? _ds.detection_lon ?? null;

    if (focusPoly && focusPoly.length > 2) {
      focusView = computeFocusView(focusPoly, detLat, detLon);
      focusViewComputed = true;
    }
  }

  // Switch view based on mission phase
  const zoomedStates = ['FOCUS', 'DELIVER'];
  if (focusView && zoomedStates.includes(state)) {
    currentView = focusView;
  } else if (focusView && state === 'SAFE_RTL') {
    // Zoom back out for RTH so the full corridor is visible
    currentView = { ...MAP_BOUNDS };
  } else {
    currentView = { ...MAP_BOUNDS };
  }
}

function drawMap(s) {
  const w = canvas.width / devicePixelRatio;
  const h = canvas.height / devicePixelRatio;
  ctx.clearRect(0, 0, w, h);

  // Background — satellite image or grid fallback
  if (satImgLoaded && satImg) {
    // Map the satellite image geo-bounds to canvas coordinates
    const [sx0, sy0] = geoToCanvas(SAT_BOUNDS.north, SAT_BOUNDS.west);
    const [sx1, sy1] = geoToCanvas(SAT_BOUNDS.south, SAT_BOUNDS.east);
    ctx.globalAlpha = 0.65;  // dim slightly so overlays pop
    ctx.drawImage(satImg, sx0, sy0, sx1 - sx0, sy1 - sy0);
    ctx.globalAlpha = 1.0;
    // Light grid overlay on top of satellite
    ctx.strokeStyle = 'rgba(26,36,53,.25)';
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= 10; i++) {
      const x = MAP_PAD + (w - MAP_PAD * 2) / 10 * i;
      ctx.beginPath(); ctx.moveTo(x, MAP_PAD); ctx.lineTo(x, h - MAP_PAD); ctx.stroke();
      const y = MAP_PAD + (h - MAP_PAD * 2) / 10 * i;
      ctx.beginPath(); ctx.moveTo(MAP_PAD, y); ctx.lineTo(w - MAP_PAD, y); ctx.stroke();
    }
  } else {
    // Fallback grid
    ctx.strokeStyle = 'rgba(26,36,53,.4)';
    ctx.lineWidth = 0.5;
    for (let i = 0; i < 20; i++) {
      const x = (w / 20) * i;
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
      const y = (h / 20) * i;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }
  }

  // ── SSSI corridor (outbound) ──
  drawPath(SSSI_OUT, 'rgba(90,112,128,.3)', 2, [6, 4]);

  // ── SSSI corridor (return) ──
  drawPath(SSSI_HOME, 'rgba(90,112,128,.3)', 2, [6, 4]);

  // ── Search pattern (original — dimmed if focus pattern is active) ──
  const hasFocusWPs = s && s.focus_waypoints && s.focus_waypoints.length > 1;
  const searchAlpha = hasFocusWPs ? 0.12 : 0.35;
  drawPath(SEARCH_WPS, `rgba(59,158,255,${searchAlpha})`, 1.5, []);
  SEARCH_WPS.forEach(([lat, lon]) => {
    const [x, y] = geoToCanvas(lat, lon);
    ctx.beginPath();
    ctx.arc(x, y, 2.5, 0, Math.PI * 2);
    ctx.fillStyle = `rgba(59,158,255,${searchAlpha + 0.05})`;
    ctx.fill();
  });

  // ── Focus waypoints (replanned pattern — purple) ──
  if (hasFocusWPs) {
    drawPath(s.focus_waypoints, 'rgba(167,139,250,.5)', 1.5, []);
    s.focus_waypoints.forEach(([lat, lon]) => {
      const [x, y] = geoToCanvas(lat, lon);
      ctx.beginPath();
      ctx.arc(x, y, 2.5, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(167,139,250,.6)';
      ctx.fill();
    });
  }

  // ── Focus area polygon ──
  // Try focus_polygon_coords (set by REPLAN), fall back to deliver_status.focus_polygon
  const focusPoly = (s && s.focus_polygon_coords) ||
                    (s && s.deliver_status && s.deliver_status.focus_polygon) ||
                    null;
  if (focusPoly && focusPoly.length > 2) {
    ctx.beginPath();
    focusPoly.forEach(([lat, lon], i) => {
      const [x, y] = geoToCanvas(lat, lon);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.fillStyle = 'rgba(167,139,250,.08)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(167,139,250,.4)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([]);
    ctx.stroke();
  }

  // ── Home marker ──
  const [hx, hy] = geoToCanvas(HOME_POS[0], HOME_POS[1]);
  ctx.beginPath();
  ctx.arc(hx, hy, 5, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(52,211,153,.6)';
  ctx.fill();
  ctx.strokeStyle = 'rgba(52,211,153,.9)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([]);
  ctx.stroke();
  ctx.fillStyle = 'rgba(52,211,153,.7)';
  ctx.font = '600 9px Inter, system-ui';
  ctx.fillText('HOME', hx + 8, hy + 3);

  // ── Detection marker ──
  if (missionTracker.detectionLat && missionTracker.detectionLon) {
    const [dx, dy] = geoToCanvas(missionTracker.detectionLat, missionTracker.detectionLon);
    // Pulsing ring
    const pulse = (Math.sin(Date.now() / 300) + 1) / 2;
    ctx.beginPath();
    ctx.arc(dx, dy, 8 + pulse * 6, 0, Math.PI * 2);
    ctx.strokeStyle = `rgba(245,158,11,${0.2 + pulse * 0.2})`;
    ctx.lineWidth = 2;
    ctx.stroke();
    // Core dot
    ctx.beginPath();
    ctx.arc(dx, dy, 5, 0, Math.PI * 2);
    ctx.fillStyle = '#f59e0b';
    ctx.fill();
    // Cross
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(dx - 10, dy); ctx.lineTo(dx + 10, dy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(dx, dy - 10); ctx.lineTo(dx, dy + 10); ctx.stroke();
    // Label
    ctx.fillStyle = '#f59e0b';
    ctx.font = '700 10px Inter, system-ui';
    ctx.fillText('DETECTED', dx + 12, dy - 2);
  }

  // ── Landing target marker ──
  if (landingTargetLat && landingTargetLon) {
    const [lx, ly] = geoToCanvas(landingTargetLat, landingTargetLon);
    // Diamond shape
    ctx.beginPath();
    ctx.moveTo(lx, ly - 7);
    ctx.lineTo(lx + 5, ly);
    ctx.lineTo(lx, ly + 7);
    ctx.lineTo(lx - 5, ly);
    ctx.closePath();
    ctx.fillStyle = 'rgba(52,211,153,.3)';
    ctx.fill();
    ctx.strokeStyle = '#34d399';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([]);
    ctx.stroke();
    // Label
    ctx.fillStyle = '#34d399';
    ctx.font = '600 9px Inter, system-ui';
    ctx.fillText('LANDING', lx + 8, ly + 3);

    // Line from detection to landing point (if both exist)
    if (missionTracker.detectionLat && missionTracker.detectionLon) {
      const [ddx, ddy] = geoToCanvas(missionTracker.detectionLat, missionTracker.detectionLon);
      ctx.beginPath();
      ctx.moveTo(ddx, ddy);
      ctx.lineTo(lx, ly);
      ctx.strokeStyle = 'rgba(52,211,153,.25)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  // ── Drone trail ──
  if (droneTrail.length > 1) {
    ctx.beginPath();
    droneTrail.forEach(({lat, lon}, i) => {
      const [x, y] = geoToCanvas(lat, lon);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.strokeStyle = 'rgba(255,255,255,.15)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([]);
    ctx.stroke();
  }

  // ── Drone position ──
  const [droneLat, droneLon] = getDronePos(s);
  if (droneLat && droneLon) {
    const [dx, dy] = geoToCanvas(droneLat, droneLon);

    // Outer pulse ring
    const t = Date.now() / 600;
    const r = 12 + Math.sin(t) * 4;
    ctx.beginPath();
    ctx.arc(dx, dy, r, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255,255,255,.06)';
    ctx.fill();

    // Inner ring
    ctx.beginPath();
    ctx.arc(dx, dy, 6, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255,255,255,.15)';
    ctx.fill();

    // Core
    ctx.beginPath();
    ctx.arc(dx, dy, 3.5, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();
    ctx.shadowColor = '#ffffff';
    ctx.shadowBlur = 8;
    ctx.fill();
    ctx.shadowBlur = 0;
  }
}

function drawPath(coords, color, width, dash) {
  if (coords.length < 2) return;
  ctx.beginPath();
  coords.forEach(([lat, lon], i) => {
    const [x, y] = geoToCanvas(lat, lon);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.stroke();
  ctx.setLineDash([]);
}

function getDronePos(s) {
  // Returns [lat, lon] from whichever state is currently active.
  // Falls back to last known position during transitions.
  //
  // Key insight: extras persist across states (set_extra doesn't clear
  // old keys). So idle_status might still have stale ground coordinates
  // from before takeoff. We MUST check the active state's data first,
  // and only use idle_status if we're actually in IDLE.
  if (!s) return [lastKnownLat, lastKnownLon];

  const state = s.current_state || '';

  // Map each state to its data source
  const stateSourceMap = {
    'SEARCH':         s.search_progress,
    'FOCUS':          s.focus_progress,
    'DELIVER':        (typeof s.deliver_status === 'object') ? s.deliver_status : null,
    'SAFE_RTL':       (typeof s.safe_rtl_status === 'object') ? s.safe_rtl_status : null,
    'IDLE':           s.idle_status,
    'PRE_AUTO_CHECK': s.idle_status,
    'GUIDED_TAKEOFF': s.idle_status,
  };

  // Try the active state's source first
  const primary = stateSourceMap[state];
  if (primary && primary.lat != null && primary.lon != null &&
      Math.abs(primary.lat) > 0.001 && Math.abs(primary.lon) > 0.001) {
    lastKnownLat = primary.lat;
    lastKnownLon = primary.lon;
    return [primary.lat, primary.lon];
  }

  // For transitional states (REPLAN, CHANGE_MODE, UPLOAD_*, GENERATE_*),
  // try all active sources in priority order (skip stale idle_status)
  const activeSources = [
    s.search_progress,
    s.focus_progress,
    (typeof s.deliver_status === 'object') ? s.deliver_status : null,
    (typeof s.safe_rtl_status === 'object') ? s.safe_rtl_status : null,
  ];

  for (const src of activeSources) {
    if (src && src.lat != null && src.lon != null &&
        Math.abs(src.lat) > 0.001 && Math.abs(src.lon) > 0.001) {
      lastKnownLat = src.lat;
      lastKnownLon = src.lon;
      return [src.lat, src.lon];
    }
  }

  // No active source — use last known (persists through REPLAN, etc.)
  return [lastKnownLat, lastKnownLon];
}

// Also track the landing target from deliver_status
function updateLandingTarget(s) {
  const ds = (typeof s.deliver_status === 'object') ? s.deliver_status : null;
  if (ds && ds.deliver_target_lat && ds.deliver_target_lon) {
    landingTargetLat = ds.deliver_target_lat;
    landingTargetLon = ds.deliver_target_lon;
  }
}

/* ═══════════════════════════════════════════════════════════════
   CLOCK
   ═══════════════════════════════════════════════════════════════ */
function updateClock() {
  const now = new Date();
  document.getElementById('clock').textContent =
    now.toLocaleTimeString('en-GB', { hour12: false });
}
setInterval(updateClock, 1000);
updateClock();

/* ═══════════════════════════════════════════════════════════════
   POLLING & RENDER
   ═══════════════════════════════════════════════════════════════ */
let lastStatus = null;

async function poll() {
  try {
    const r = await fetch('/api/status');
    const s = await r.json();
    if (s._proxy_error) {
      proxyFailCount++;
      if (proxyFailCount >= 3) showDisconnected();
      return;
    }
    proxyFailCount = 0;
    lastStatus = s;
    render(s);
  } catch(e) {
    proxyFailCount++;
    if (proxyFailCount >= 3) showDisconnected();
  }
}

function showDisconnected() {
  document.getElementById('conn-dot').className = 'dot';
  document.getElementById('conn-text').textContent = 'No connection';
}

// ── Telemetry source selection ───────────────────────────────
// CRITICAL FIX: SharedStatus extras are cumulative — old keys persist
// even after a state exits (e.g. search_progress sticks around when
// PLB triggers REPLAN → FOCUS because SEARCH.exit() is pass).
//
// This function returns THE ONE correct telemetry source for the
// current state.  It never falls back to a previous phase's data.
// Better to show '--' than wrong numbers from a stale source.

function getTelemetrySource(state, s) {
  // Direct mapping: each major state owns exactly one data source
  const directMap = {
    'SEARCH':         s.search_progress,
    'FOCUS':          s.focus_progress,
    'DELIVER':        (typeof s.deliver_status === 'object') ? s.deliver_status : null,
    'SAFE_RTL':       (typeof s.safe_rtl_status === 'object') ? s.safe_rtl_status : null,
    'IDLE':           s.idle_status,
    'PRE_AUTO_CHECK': s.idle_status,
    'GUIDED_TAKEOFF': s.idle_status,
  };

  if (directMap[state] !== undefined) {
    return directMap[state] || {};
  }

  // Transitional states (REPLAN, CHANGE_MODE, UPLOAD_*, GENERATE_*):
  // Walk the mission phases BACKWARDS.  The latest phase that actually
  // has data with a non-null alt is the one currently pushing telemetry.
  // This correctly picks focus_progress after SEARCH→REPLAN→FOCUS,
  // and ignores stale search_progress.
  const phaseSources = [
    (typeof s.safe_rtl_status === 'object') ? s.safe_rtl_status : null,
    (typeof s.deliver_status === 'object') ? s.deliver_status : null,
    s.focus_progress,
    // search_progress is INTENTIONALLY LAST — if any later phase has
    // data, search is stale and must be skipped.
    s.search_progress,
  ];

  for (const src of phaseSources) {
    if (src && typeof src === 'object' && src.alt != null) {
      return src;
    }
  }
  return {};
}

function render(s) {
  if (missionComplete) return;  // freeze on completion screen

  const state = s.current_state || 'STARTING';

  // ── Dynamic zoom ──
  updateView(state, s);

  // ── Connection ──
  const dot = document.getElementById('conn-dot');
  const txt = document.getElementById('conn-text');
  if (s.connected) {
    dot.className = 'dot on';
    txt.textContent = s.current_mode || 'Connected';
  } else {
    dot.className = 'dot';
    txt.textContent = 'No heartbeat';
  }

  // ── State badge on map ──
  const nameEl = document.getElementById('state-name');
  const descEl = document.getElementById('state-desc');
  nameEl.textContent = FRIENDLY_STATE[state] || state;
  nameEl.style.color = STATE_COLOR[state] || '#c8d6e0';
  descEl.textContent = STATE_DESC[state] || '';

  // ── Timeline ──
  reachedStates.add(state);
  updateTimeline(state);

  // ── Pull telemetry from active state ──
  // Uses getTelemetrySource which returns ONLY the correct source
  // for this state.  No ?? chains across sources = no stale data.
  const telem = getTelemetrySource(state, s);

  const alt = telem.alt ?? null;
  const bat = telem.battery_pct ?? -1;
  const sats = telem.satellites ?? null;
  const [lat, lon] = getDronePos(s);
  updateLandingTarget(s);
  const wp = telem.current_wp ?? null;
  const totalWps = telem.total_wps ?? null;
  const elapsed = telem.elapsed ?? null;

  // Altitude
  document.getElementById('v-alt').textContent =
    alt != null ? Number(alt).toFixed(0) : '--';

  // Mode
  document.getElementById('v-mode').textContent = s.current_mode || '--';
  document.getElementById('bottom-mode').textContent = 'Mode: ' + (s.current_mode || '--');

  // Waypoint
  document.getElementById('v-wp').textContent =
    (wp != null && totalWps) ? wp + '/' + totalWps : '--';

  // Elapsed
  document.getElementById('v-elapsed').textContent = elapsed || '--';

  // Satellites
  document.getElementById('v-sats').textContent = sats != null ? sats : '--';

  // Position
  document.getElementById('v-pos').textContent =
    (lat != null && lon != null) ? Number(lat).toFixed(5) + ', ' + Number(lon).toFixed(5) : '--';

  // ── Battery ──
  const batEl = document.getElementById('bat-pct');
  const batFill = document.getElementById('bat-fill');
  if (bat >= 0) {
    batEl.textContent = bat + '%';
    batFill.style.width = bat + '%';
    batFill.className = 'bat-fill' + (bat < 20 ? ' low' : bat < 40 ? ' mid' : '');
  } else {
    batEl.textContent = '--%';
    batFill.style.width = '0%';
  }

  // ── Progress ──
  const progPanel = document.getElementById('progress-panel');
  const pct = telem.progress_pct ?? null;
  if (pct != null && ['SEARCH','FOCUS','SAFE_RTL'].includes(state)) {
    progPanel.style.display = 'block';
    const isRtl = state === 'SAFE_RTL';
    const isFocus = state === 'FOCUS';
    document.getElementById('prog-label').textContent =
      isRtl ? 'Return Progress' : isFocus ? 'Focus Search Progress' : 'Search Progress';
    document.getElementById('prog-fill').style.width = pct + '%';
    document.getElementById('prog-fill').className =
      'prog-fill' + (isFocus ? ' focus' : '');
    const pass2 = (s.search_progress && s.search_progress.second_pass && state === 'SEARCH') ? ' (Pass 2)' : '';
    document.getElementById('prog-text').textContent = pct + '% complete' + pass2;
  } else {
    progPanel.style.display = 'none';
  }

  // ── Detection ──
  // Detection coords come from focus_progress or deliver_status — check both
  // directly (not via telem, because detection persists across states)
  const _fp = s.focus_progress || {};
  const _ds = (typeof s.deliver_status === 'object') ? s.deliver_status : {};
  const detLat = _fp.detection_lat ?? _ds.detection_lat ?? null;
  const detLon = _fp.detection_lon ?? _ds.detection_lon ?? null;
  if (detLat && detLon) {
    missionTracker.detectionLat = detLat;
    missionTracker.detectionLon = detLon;
    document.getElementById('det-panel').style.display = 'block';
    document.getElementById('det-info').textContent =
      Number(detLat).toFixed(6) + ', ' + Number(detLon).toFixed(6);

    // Show overlay on first detection
    if (!detectionShown) {
      detectionShown = true;
      const overlay = document.getElementById('det-overlay');
      overlay.className = 'detection-overlay show';
      document.getElementById('det-coords').textContent =
        Number(detLat).toFixed(6) + ', ' + Number(detLon).toFixed(6);
      // Auto-hide after 8 seconds
      setTimeout(() => { overlay.className = 'detection-overlay'; }, 8000);
    }
  }

  // ── Payload deployed ──
  // Use the persistent payload_deployed flag (set on SharedStatus extras,
  // survives state transitions) rather than the transient deliver_status
  // which gets cleared when DELIVER exits to SAFE_RTL.
  if (!deployShown && s.payload_deployed) {
    deployShown = true;
    const dOverlay = document.getElementById('deploy-overlay');
    dOverlay.className = 'deploy-overlay show';
    setTimeout(() => { dOverlay.className = 'deploy-overlay'; }, 8000);
  }

  // ── Returning home overlay ──
  if (!rthShown && state === 'SAFE_RTL') {
    const _rtlSt = (typeof s.safe_rtl_status === 'object') ? (s.safe_rtl_status.status || '') : '';
    if (_rtlSt.includes('RETURNING')) {
      rthShown = true;
      const rOverlay = document.getElementById('rth-overlay');
      rOverlay.className = 'rth-overlay show';
      setTimeout(() => { rOverlay.className = 'rth-overlay'; }, 8000);
    }
  }

  // ── Drone trail ──
  if (lat && lon) {
    const last = droneTrail.length > 0 ? droneTrail[droneTrail.length - 1] : null;
    if (!last || Math.abs(last.lat - lat) > 0.000005 || Math.abs(last.lon - lon) > 0.000005) {
      droneTrail.push({ lat, lon, t: Date.now() });
      if (droneTrail.length > TRAIL_MAX) droneTrail.shift();
    }
  }

  // ── Log ──
  if (s.last_message) {
    document.getElementById('log-msg').textContent = s.last_message;
  }

  // ── Mission tracking ──
  if (state === 'SEARCH' && missionTracker.startBattery === null && bat >= 0) {
    missionTracker.startBattery = bat;
    missionTracker.missionStartTime = Date.now();
  }
  if (bat >= 0) missionTracker.endBattery = bat;
  if (totalWps) missionTracker.totalWps = Math.max(missionTracker.totalWps, totalWps);

  if (state !== missionTracker.lastState) {
    missionTracker.stateLog.push({ state, time: new Date().toLocaleTimeString('en-GB', {hour12:false}) });
    missionTracker.lastState = state;
  }

  // ── Mission complete ──
  const rtlStatus = (typeof s.safe_rtl_status === 'object') ? (s.safe_rtl_status.status || '') : (s.safe_rtl_status || '');
  if ((rtlStatus.toUpperCase().includes('MISSION COMPLETE') || s.mission_complete) && !missionComplete) {
    missionComplete = true;
    // Mark all timeline steps as done, light up Complete
    const steps = document.querySelectorAll('.tl-step');
    steps.forEach(step => {
      if (step.id === 'tl-complete') {
        step.className = 'tl-step complete';
      } else {
        step.className = 'tl-step done';
      }
    });
    showMissionComplete();
  }

  // ── Draw map ──
  drawMap(s);
}

function updateTimeline(currentState) {
  const steps = document.querySelectorAll('.tl-step');
  // Build ordered list of which phases are done
  const phaseOrder = ['IDLE','PRE_AUTO_CHECK','GUIDED_TAKEOFF','UPLOAD_FENCE',
    'GENERATE_PATTERN','UPLOAD_MISSION','CHANGE_MODE','SEARCH','REPLAN','FOCUS','DELIVER','SAFE_RTL'];
  const currentIdx = phaseOrder.indexOf(currentState);

  steps.forEach(step => {
    const states = step.dataset.states.split(',');
    const isCurrent = states.includes(currentState);
    const visited = states.some(s => reachedStates.has(s));

    // Determine highest index in this step
    const maxIdx = Math.max(...states.map(s => phaseOrder.indexOf(s)));

    step.className = 'tl-step';
    if (isCurrent) {
      step.classList.add('active');
    } else if (visited && maxIdx < currentIdx) {
      step.classList.add('done');
    }
  });
}

/* ═══════════════════════════════════════════════════════════════
   MISSION COMPLETE SCREEN
   ═══════════════════════════════════════════════════════════════ */
function showMissionComplete() {
  const grid = document.getElementById('summary-grid');
  const mt = missionTracker;

  // Calculate mission duration
  let duration = '--';
  if (mt.missionStartTime) {
    const secs = Math.round((Date.now() - mt.missionStartTime) / 1000);
    const m = Math.floor(secs / 60);
    const s = secs % 60;
    duration = m + ':' + String(s).padStart(2, '0');
  }

  const batUsed = (mt.startBattery !== null && mt.endBattery !== null)
    ? (mt.startBattery - mt.endBattery) + '%'
    : '--';

  const cards = [
    { label: 'Mission Time', val: duration, unit: 'min:sec' },
    { label: 'Battery Used', val: batUsed, unit: mt.startBattery ? mt.startBattery + '% → ' + mt.endBattery + '%' : '' },
    { label: 'Waypoints', val: mt.totalWps, unit: 'completed' },
    { label: 'Detection', val: mt.detectionLat ? 'YES' : 'NO', unit: mt.detectionLat ? Number(mt.detectionLat).toFixed(5) + ', ' + Number(mt.detectionLon).toFixed(5) : '' },
    { label: 'States Visited', val: new Set(mt.stateLog.map(s => s.state)).size, unit: 'phases' },
    { label: 'Final Battery', val: mt.endBattery != null ? mt.endBattery + '%' : '--', unit: 'remaining' },
  ];

  grid.innerHTML = cards.map(c => `
    <div class="summary-card">
      <div class="s-label">${c.label}</div>
      <div class="s-val">${c.val}</div>
      <div class="s-unit">${c.unit}</div>
    </div>
  `).join('');

  // State timeline
  const log = document.getElementById('state-log');
  const mainStates = mt.stateLog.filter(s =>
    ['IDLE','SEARCH','REPLAN','FOCUS','DELIVER','SAFE_RTL'].includes(s.state)
  );
  log.innerHTML = mainStates.map((s, i) => {
    const arrow = i < mainStates.length - 1 ? '<span class="sl-arrow">→</span>' : '';
    return `<div class="sl-item"><span class="sl-dot"></span>${FRIENDLY_STATE[s.state] || s.state} <span style="color:var(--text-muted)">${s.time}</span></div>${arrow}`;
  }).join('');

  document.getElementById('complete-overlay').className = 'complete-overlay show';
}

/* ═══════════════════════════════════════════════════════════════
   MAP ANIMATION LOOP (redraws for pulsing effects)
   ═══════════════════════════════════════════════════════════════ */
function animateMap() {
  if (lastStatus && !missionComplete) drawMap(lastStatus);
  requestAnimationFrame(animateMap);
}
requestAnimationFrame(animateMap);

/* ═══════════════════════════════════════════════════════════════
   START
   ═══════════════════════════════════════════════════════════════ */
setInterval(poll, 800);
poll();
// Initial map draw (empty)
drawMap(null);
</script>
</body>
</html>"""


# ── Entry point ───────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="SAR Spectator Display (read-only, fully isolated)"
    )
    parser.add_argument(
        "--operator", default=DEFAULT_OPERATOR_HOST,
        help=f"Operator GUI IP/hostname (default: {DEFAULT_OPERATOR_HOST})"
    )
    parser.add_argument(
        "--operator-port", type=int, default=DEFAULT_OPERATOR_PORT,
        help=f"Operator GUI port (default: {DEFAULT_OPERATOR_PORT})"
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_SPECTATOR_PORT,
        help=f"Spectator display port (default: {DEFAULT_SPECTATOR_PORT})"
    )
    args = parser.parse_args()

    global _operator_url
    _operator_url = f"http://{args.operator}:{args.operator_port}/api/status"

    print(f"[SPECTATOR] Proxying status from {_operator_url}")
    print(f"[SPECTATOR] Open http://<this-machine>:{args.port} on the demo monitor")
    print(f"[SPECTATOR] This process is fully independent — kill it any time.\n")

    app.run(host="0.0.0.0", port=args.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
