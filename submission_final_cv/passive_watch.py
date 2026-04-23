#!/usr/bin/env python3
# passive_watch.py — read-only camera observer, AI detection, MJPEG stream + snapshots.
# Runs on the Pi; connect from a laptop browser at http://<PI_IP>:8090/
# No commands are ever sent. GPS (if present) is read from mavproxy for geotagging only.
import sys, os, time, signal, threading, argparse, subprocess
import csv, json, math, queue
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

# field_tools/ is one level below root, so push that on sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
signal.signal(signal.SIGINT, signal.SIG_DFL)  # ctrl+c

# headless OpenCV
if not os.environ.get('DISPLAY'):
    os.environ.pop('QT_QPA_PLATFORM', None)

import cv2
import numpy as np
import config
from vision import VisionSystem

# streaming globals
latest_jpeg = None
latest_det_jpeg = None
latest_bullseye = None
latest_detection_jpeg = None   # latest detection snapshot
latest_smart_grid_jpeg = None  # 5x2 smart frames grid
latest_map_jpeg = None         # satellite map overlay
frame_lock = threading.Lock()
gps_lock = threading.Lock()
_all_gps_estimates = []
_ground_truth = {"lat": 51.423527, "lon": -2.670937}  # true dummy position
_map_base = None  # loaded map.jpg (once)
bullseye_map_bg = True  # default ON: satellite map crop behind bullseye scatter plots
latest_best_jpeg = None  # best (most central) detection JPEG
_best_center_dist = 999.0  # track best center distance
_best_detection_gps = None  # {"est_lat", "est_lon", "drone_lat", "drone_lon", "center_dist"} or None

# shared state for threaded inference
_inference_frame = None        # latest frame from display thread, read by inference
_inference_lock = threading.Lock()
_inference_eyes = None         # VisionSystem ref, swapped on model switch
_inference_eyes_lock = threading.Lock()

# detection result passes from inference thread -> display thread via tuples
# (tuple assignment is atomic in CPython so no extra lock needed for last_det)
_snap_request_latest = False
_snap_request_best = False
_snap_gps_info = None          # GPS info dict for snapshot overlay (set by inference thread)
_snap_gps_info_best = None     # GPS info dict for best snapshot (set by inference thread)
_snap_jpeg_latest = None       # pre-rendered JPEG from inference thread (exact detection frame)
_snap_jpeg_best = None         # pre-rendered JPEG from inference thread (exact detection frame)
_snap_display_for_smart = None # overlayed frame for smart estimator (set by inference, consumed by display)
_inference_det_count = 0       # detection count (written by inference thread only)
_inference_saved_count = 0     # saved count (written by inference thread only)
# Raw detection (ALL detections regardless of class filter / conf threshold)
# Used to show dim gray markers for rejected detections on the stream.
# Tuple: (cx, cy, conf, 0.0, cls_name, bw, bh) or None.  Fades after 0.5s.
_last_raw_det = None
_last_raw_det_time = 0

# async file I/O queue, keeps imwrite/json off the inference thread
_save_queue = queue.Queue(maxsize=50)


def _save_worker():
    # drains save queue so imwrite/json don't block inference
    while True:
        job = _save_queue.get()
        if job is None:
            break  # poison pill
        try:
            kind = job["kind"]
            if kind == "image":
                cv2.imwrite(job["path"], job["frame"])
            elif kind == "json":
                with open(job["path"], 'w') as jf:
                    json.dump(job["data"], jf, indent=2)
            elif kind == "csv":
                job["writer"].writerow(job["row"])
                job["file"].flush()
        except Exception as e:
            print(f"[SAVE] Error: {e}")
        _save_queue.task_done()


def _enqueue_save(job):
    try:
        _save_queue.put_nowait(job)
    except queue.Full:
        pass  # drop rather than stall inference

# result-mode globals
_smart_result_saved = False   # prevents saving SMART result multiple times per lock
_smart_image_counter = 0      # always-incrementing counter for smart_detections filenames
_survey_result_saved = False
SURVEY_TARGET = 100           # estimates before survey completes
_result_banner = None

# argparse
parser = argparse.ArgumentParser(description="Passive camera watch + stream + snapshots")
parser.add_argument('--port', type=int, default=8090)
parser.add_argument('--conf', type=float, default=0.4)
parser.add_argument('--fps', type=float, default=5)
parser.add_argument('--save-dir', default='detections')
parser.add_argument('--no-save', action='store_true')
parser.add_argument('--no-mavlink', action='store_true', help='Skip mavproxy connection (no GPS)')
parser.add_argument('--model', default='best.tflite', help='Path to .tflite model (default: best.tflite)')
parser.add_argument('--simple-names', action='store_true', help='Simple filenames (no det_ prefix, no JSON sidecars)')
parser.add_argument('--class-filter', type=str, default=None, help='Only save detections of this class (e.g. "person")')
parser.add_argument('--smart-estimate', action='store_true', help='Accumulate central detections, save after 10+ with median GPS')
parser.add_argument('--smart-min', type=int, default=5, help='Min central detections before saving (default 5)')
parser.add_argument('--smart-radius', type=float, default=1.0, help='Max spread for smart cluster (default 1.0m)')
parser.add_argument('--smart-dir', type=str, default=None, help='Directory for SMART result images (default: <save-dir>/smart_detections/)')
parser.add_argument('--auto-clear', type=float, default=0, help='Auto-clear SMART after N seconds of lock (0=off). Allows multiple independent locks per run.')
parser.add_argument('--fake', action='store_true', help='Replay DJI video + SRT telemetry (no camera/mavproxy)')
parser.add_argument('--fake-video', default='RealVideo/DJI_0001_1456x1088_cropped_30fps.mp4')
parser.add_argument('--fake-srt', default='RealVideo/DJI_20260311172332_0001_V.SRT')
parser.add_argument('--no-stream', action='store_true', help='Disable HTTP stream server')
parser.add_argument('--compensate-tilt', action='store_true', help='Compensate for drone pitch/roll when estimating target GPS')
args = parser.parse_args()

# models shown in the browser switcher
MODEL_TABLE = [
    {"id": 0, "name": "Original",      "path": "best.tflite",                         "backend": None},
    {"id": 1, "name": "SAR v2 TFL",    "path": "cv_models/sar_v2_1088/best.tflite",   "backend": None},
    {"id": 2, "name": "SAR v2 NCNN",   "path": "cv_models/sar_v2_1088/best.tflite",   "backend": "ncnn"},
    {"id": 3, "name": "COCO Person",   "path": "cv_models/human.tflite",              "backend": None},
    {"id": 4, "name": "COCO P. NCNN",  "path": "cv_models/human_ncnn",                "backend": "ncnn"},
]

# Runtime state (modified by API endpoints, read by main loop)
runtime_state = {
    "active_model_id": 0,       # index into MODEL_TABLE
    "conf_threshold": args.conf, # current confidence threshold
    "class_filter": args.class_filter or "all",  # "all", "dummy", "person"
    "model_switch_request": None,  # set to model_id to trigger reload in main loop
    "model_switch_error": None,    # set to error string if switch fails
}
runtime_lock = threading.Lock()


# SRT parser for fake mode — returns {frame_num: {lat, lon, alt, yaw, pitch, roll}}
def parse_srt(srt_path):
    import re
    with open(srt_path, 'r') as f:
        text = f.read()
    entries = {}
    blocks = re.split(r'\n\n+', text.strip())
    frame = 0
    for block in blocks:
        lines = block.strip().split('\n')
        if len(lines) < 3:
            continue
        try:
            int(lines[0])
        except ValueError:
            continue
        frame += 1
        data = ' '.join(lines[2:])
        lat = lon = alt = yaw = pitch = roll = 0.0
        m = re.search(r'\[latitude:\s*([-\d.]+)\]', data)
        if m: lat = float(m.group(1))
        m = re.search(r'\[longitude:\s*([-\d.]+)\]', data)
        if m: lon = float(m.group(1))
        m = re.search(r'\[rel_alt:\s*([-\d.]+)', data)
        if m: alt = float(m.group(1))
        m = re.search(r'gb_yaw:\s*([-\d.]+)', data)
        if m: yaw = float(m.group(1))
        m = re.search(r'gb_pitch:\s*([-\d.]+)', data)
        if m: pitch = float(m.group(1))
        m = re.search(r'gb_roll:\s*([-\d.]+)', data)
        if m: roll = float(m.group(1))
        entries[frame] = {'lat': lat, 'lon': lon, 'alt': alt, 'yaw': yaw, 'pitch': pitch, 'roll': roll}
    return entries


# the dashboard HTML (placeholders get substituted in do_GET below)
HTML_PAGE = """<!DOCTYPE html>
<html><head><title>SAR Passive Watch</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box}
  body{background:#111;color:#eee;font-family:monospace;padding:15px}
  h1{color:#0f0;margin-bottom:8px}
  h2{color:#0ff;font-size:0.85em;margin:10px 0 5px}
  .stats{color:#888;margin-bottom:3px;font-size:0.85em}
  .gps{color:#0af;margin-bottom:3px;font-size:0.85em}
  .est{color:#ff00ff;margin-bottom:3px;font-weight:bold;font-size:0.85em}
  .fov{color:#b90;margin-bottom:8px;font-size:0.75em}
  img{border:1px solid #333;display:block}
  /* Row 1: Camera Feed 50% | Latest Det 25% + Best Det 25% */
  .row1{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px}
  .row1 img{width:100%;height:auto}
  .det-pair{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .det-pair img{width:100%;height:auto}
  /* Row 2: Map 25% | GPS Analysis 75% */
  .row2{display:grid;grid-template-columns:1fr 3fr;gap:10px;margin-bottom:10px}
  .row2 img{width:100%;height:auto}
  /* Row 3: CV Pipeline | GPS Pipeline */
  .row3{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px}
  @media(max-width:1000px){.row1,.row2,.row3{grid-template-columns:1fr} .det-pair{grid-template-columns:1fr}}

  /* ── Model selector control bar ── */
  .ctrl-bar{
    display:flex;align-items:center;gap:12px;flex-wrap:wrap;
    background:#1a1a1a;border:1px solid #333;border-radius:4px;
    padding:6px 12px;margin-bottom:8px;font-size:0.8em;
  }
  .ctrl-bar label{color:#999;white-space:nowrap}
  .ctrl-bar select,.ctrl-bar input[type=text]{
    background:#222;color:#0f0;border:1px solid #444;border-radius:3px;
    padding:3px 6px;font-family:monospace;font-size:1em;
  }
  .ctrl-bar select:focus,.ctrl-bar input:focus{outline:none;border-color:#0f0}
  .ctrl-bar .slider-group{display:flex;align-items:center;gap:4px}
  .ctrl-bar input[type=range]{
    width:100px;accent-color:#0f0;cursor:pointer;
  }
  .ctrl-bar .conf-val{color:#0f0;min-width:32px;text-align:right}
  .ctrl-bar .sep{color:#333;margin:0 2px}
  .ctrl-bar .model-status{color:#666;font-size:0.9em}
  .ctrl-bar .model-status.loading{color:#ff0}
  .ctrl-bar .model-status.ok{color:#0f0}
  .ctrl-bar .model-status.err{color:#f44}
  /* Zoom-wrap: scroll to zoom, drag to pan on detection images */
  .zoom-wrap{overflow:hidden;cursor:grab;position:relative}
  .zoom-wrap img{transition:transform 0.1s ease-out;transform-origin:center center}
</style>
</head><body>
<h1>SAR Passive Watch</h1>

<!-- Model selector control bar -->
<div class="ctrl-bar">
  <label>Model:</label>
  <select id="model-sel">
    <option value="0">Original (best.tflite)</option>
    <option value="1">SAR v2 TFLite</option>
    <option value="2">SAR v2 NCNN</option>
    <option value="3">COCO Person</option>
    <option value="4">COCO Person NCNN</option>
  </select>
  <span class="model-status" id="model-status"></span>
  <span id="inference-fps" style="margin-left:8px;padding:2px 8px;background:#003;border:1px solid #0af;border-radius:4px;color:#0af;font-weight:bold;font-size:1.1em">-- FPS</span>
  <span class="sep">|</span>

  <label>Conf:</label>
  <div class="slider-group">
    <input type="range" id="conf-slider" min="0.05" max="0.95" step="0.05" value="__CONF_VAL__">
    <span class="conf-val" id="conf-val">__CONF_VAL__</span>
  </div>
  <span class="sep">|</span>

  <label>Class:</label>
  <span style="display:inline-flex;gap:6px;align-items:center">
    <label style="color:#0f0;cursor:pointer"><input type="checkbox" id="cls-person" __CLS_PERSON__ style="accent-color:#0f0"> person</label>
    <label style="color:#0f0;cursor:pointer"><input type="checkbox" id="cls-bird" __CLS_BIRD__ style="accent-color:#0f0"> bird</label>
    <label style="color:#0f0;cursor:pointer"><input type="checkbox" id="cls-dummy" __CLS_DUMMY__ style="accent-color:#0f0"> dummy</label>
    <label style="color:#888;cursor:pointer"><input type="checkbox" id="cls-other" __CLS_OTHER__ style="accent-color:#888"> other</label>
  </span>
  <span class="sep">|</span>
  <button id="clear-all-btn" onclick="clearAll()" style="padding:3px 10px;background:#600;color:#fff;border:1px solid #f44;border-radius:3px;cursor:pointer;font-family:monospace;font-size:1em">Clear All</button>
  <button id="reset-best-btn" onclick="resetBest()" style="padding:3px 10px;background:#333;color:#0ff;border:1px solid #0ff;border-radius:3px;cursor:pointer;font-family:monospace;font-size:1em">Reset Best</button>
  <span class="sep">|</span>
  <label>Smart:</label>
  <input type="number" id="smart-spread" value="__SMART_SPREAD__" min="0.1" max="5" step="0.05" style="width:50px;background:#222;color:#0f0;border:1px solid #444;border-radius:3px;padding:3px 4px;font-family:monospace;font-size:1em">
  <span style="color:#888">m</span>
  <input type="number" id="smart-count" value="__SMART_COUNT__" min="3" max="50" step="1" style="width:40px;background:#222;color:#0f0;border:1px solid #444;border-radius:3px;padding:3px 4px;font-family:monospace;font-size:1em">
  <span style="color:#888">pts</span>
</div>

<div class="stats" id="stats">Starting...</div>
<div class="gps" id="gps">GPS: waiting...</div>
<div class="est" id="est">ESTIMATE: waiting...</div>
<div class="fov" id="fov">FOV: ---</div>

<!-- Row 1: Camera Feed (50%) | Latest + Best Detection side-by-side (50%) -->
<div class="row1">
  <div>
    <h2>Camera Feed</h2>
    <img src="/stream" alt="Stream">
  </div>
  <div>
    <div class="det-pair">
      <div>
        <h2>Latest Detection</h2>
        <div class="zoom-wrap"><img id="latest" src="/latest" alt="Detection" style="min-height:120px"></div>
      </div>
      <div>
        <h2>Best Detection</h2>
        <div class="zoom-wrap"><img id="best-det" src="/best" alt="Best" style="min-height:120px"></div>
      </div>
    </div>
    <h2>SMART Frames</h2>
    <img id="smart-grid" src="/smart-grid" alt="Grid" style="min-height:80px;width:100%">
  </div>
</div>

<!-- Row 2: Satellite Map (25%) | GPS Analysis (75%) -->
<div class="row2">
  <div>
    <h2>Satellite Map</h2>
    <div id="imap-host"></div>
  </div>
  <div>
    <h2>GPS Analysis</h2>
    <div id="gps-charts-host"></div>
  </div>
</div>

<!-- Row 3: CV Pipeline | GPS Pipeline -->
<div class="row3">
  <div>
    <h2>CV Detection Pipeline</h2>
    <div id="cv-pipeline-host"></div>
  </div>
  <div>
    <h2>GPS Estimation Pipeline</h2>
    <div id="gps-pipeline-host"></div>
  </div>
</div>

<script>
/* ── Control bar logic ── */
const modelSel = document.getElementById('model-sel');
const confSlider = document.getElementById('conf-slider');
const confVal = document.getElementById('conf-val');
const modelStatus = document.getElementById('model-status');

function sendCmd(url) {
  return fetch(url).then(r => r.json()).catch(() => ({ok:false,error:'network'}));
}

function clearAll() {
  sendCmd('/api/clear-all').then(d => {
    if(d.ok) document.getElementById('clear-all-btn').textContent = 'Cleared!';
    setTimeout(()=>document.getElementById('clear-all-btn').textContent='Clear All', 1500);
  });
}
function resetBest() {
  sendCmd('/api/reset-best').then(d => {
    if(d.ok) document.getElementById('reset-best-btn').textContent = 'Reset!';
    setTimeout(()=>document.getElementById('reset-best-btn').textContent='Reset Best', 1500);
  });
}

modelSel.addEventListener('change', () => {
  modelStatus.textContent = 'loading...';
  modelStatus.className = 'model-status loading';
  sendCmd('/api/switch-model?id=' + modelSel.value).then(d => {
    if (d.ok) {
      modelStatus.textContent = d.model || 'ok';
      modelStatus.className = 'model-status ok';
    } else {
      modelStatus.textContent = d.error || 'failed';
      modelStatus.className = 'model-status err';
    }
  });
});

confSlider.addEventListener('input', () => {
  confVal.textContent = parseFloat(confSlider.value).toFixed(2);
});
confSlider.addEventListener('change', () => {
  sendCmd('/api/set-conf?val=' + confSlider.value);
});

// Class filter checkboxes — send comma-separated list of checked classes
function sendClassFilter() {
  const checked = [];
  if (document.getElementById('cls-person').checked) checked.push('person');
  if (document.getElementById('cls-bird').checked) checked.push('bird');
  if (document.getElementById('cls-dummy').checked) checked.push('dummy');
  if (document.getElementById('cls-other').checked) checked.push('other');
  const val = checked.length === 4 ? 'all' : checked.join(',');
  sendCmd('/api/set-class?name=' + encodeURIComponent(val));
}
document.querySelectorAll('[id^="cls-"]').forEach(cb => cb.addEventListener('change', sendClassFilter));
sendClassFilter(); // sync server with default checkbox state on page load

/* ── Smart cluster parameter controls ── */
const smartSpread = document.getElementById('smart-spread');
const smartCount = document.getElementById('smart-count');
function sendSmart() {
  sendCmd('/api/set-smart?spread=' + smartSpread.value + '&count=' + smartCount.value);
}
smartSpread.addEventListener('change', sendSmart);
smartCount.addEventListener('change', sendSmart);

/* ── Zoom+Pan on detection images ── */
document.querySelectorAll('.zoom-wrap').forEach(wrap => {
  let scale = 1, ox = 0, oy = 0, dragging = false, sx, sy;
  const img = wrap.querySelector('img');
  wrap.addEventListener('wheel', e => {
    e.preventDefault();
    const delta = e.deltaY > 0 ? 0.9 : 1.1;
    scale = Math.min(8, Math.max(1, scale * delta));
    if (scale <= 1) { ox = 0; oy = 0; }
    img.style.transform = `translate(${ox}px,${oy}px) scale(${scale})`;
  });
  wrap.addEventListener('mousedown', e => { dragging = true; sx = e.clientX - ox; sy = e.clientY - oy; wrap.style.cursor = 'grabbing'; });
  wrap.addEventListener('mousemove', e => { if (!dragging) return; ox = e.clientX - sx; oy = e.clientY - sy; img.style.transform = `translate(${ox}px,${oy}px) scale(${scale})`; });
  wrap.addEventListener('mouseup', () => { dragging = false; wrap.style.cursor = 'grab'; });
  wrap.addEventListener('mouseleave', () => { dragging = false; wrap.style.cursor = 'grab'; });
  wrap.addEventListener('dblclick', () => { scale = 1; ox = 0; oy = 0; img.style.transform = ''; });
});

/* ── Status polling (sync active model/conf from server) ── */
setInterval(()=>{
  fetch('/api/status').then(r=>r.json()).then(d=>{
    document.getElementById('stats').textContent=
      `CAM:${d.cam_fps} VIS:${d.vis_fps} STR:${d.stream_fps} Det:${d.detections}(${d.det_pct}%) Saved:${d.saved}`;
    document.getElementById('gps').textContent=
      `DRONE:${d.gps_lat},${d.gps_lon} Alt:${d.alt}m Sats:${d.sats} Mode:${d.flight_mode}`;
    document.getElementById('est').textContent=d.est_obs>0
      ?`EST:${d.est_lat},${d.est_lon}(${d.est_obs}obs)`:'EST: waiting...';
    document.getElementById('fov').textContent=
      `FOV: ${d.fov_deg}\u00b0 | @1m: ${d.cal_1m_w}\u00d7${d.cal_1m_h}cm`;
    /* Inference FPS badge */
    const vfps = parseFloat(d.vis_fps)||0;
    const fpsEl = document.getElementById('inference-fps');
    fpsEl.textContent = vfps.toFixed(1) + ' FPS';
    fpsEl.style.borderColor = vfps >= 4 ? '#0f0' : vfps >= 2 ? '#fa0' : '#f44';
    fpsEl.style.color = vfps >= 4 ? '#0f0' : vfps >= 2 ? '#fa0' : '#f44';
    const t=Date.now();
    document.getElementById('latest').src='/latest?'+t;
    document.getElementById('best-det').src='/best?'+t;
    document.getElementById('smart-grid').src='/smart-grid?'+t;
    /* Update pipeline visuals with live data */
    if (typeof updatePipelineData === 'function') {
      updatePipelineData({alt: parseFloat(d.alt)||0, fps: parseFloat(d.vis_fps)||0});
    }
    if (typeof updateGPSData === 'function') {
      updateGPSData({alt: parseFloat(d.alt)||0, fov_deg: parseFloat(d.fov_deg)||49.4});
    }
    /* Sync controls from server state */
    if (d.active_model_id !== undefined && document.activeElement !== modelSel) {
      modelSel.value = d.active_model_id;
    }
    if (d.conf_threshold !== undefined && document.activeElement !== confSlider) {
      confSlider.value = d.conf_threshold;
      confVal.textContent = parseFloat(d.conf_threshold).toFixed(2);
    }
    // Class filter sync from server (checkboxes)
    if (d.class_filter !== undefined) {
      const cf = d.class_filter;
      if (cf === 'all') {
        document.querySelectorAll('[id^="cls-"]').forEach(cb => cb.checked = true);
      } else {
        const allowed = cf.split(',').map(s => s.trim().toLowerCase());
        document.querySelectorAll('[id^="cls-"]').forEach(cb => {
          const cls = cb.id.replace('cls-', '');
          cb.checked = allowed.includes(cls);
        });
      }
    }
    if (d.smart_spread !== undefined && document.activeElement !== smartSpread) {
      smartSpread.value = d.smart_spread;
    }
    if (d.smart_count !== undefined && document.activeElement !== smartCount) {
      smartCount.value = d.smart_count;
    }
  });
},2000);
</script>
</body></html>"""


# HTTP server
class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *a):
        pass  # silent

    def do_GET(self):
        global bullseye_map_bg, _ground_truth
        path = self.path.split('?')[0]  # strip query string for matching
        if path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            # Inject interactive map into the imap-host placeholder
            from field_tools.interactive_map import get_interactive_map_html
            from field_tools.cv_pipeline_visual import get_cv_pipeline_html
            from field_tools.gps_pipeline_visual import get_gps_pipeline_html
            from field_tools.gps_charts import get_gps_charts_html
            imap_html = get_interactive_map_html(
                container_id="map-container", width="100%", height="350px"
            )
            page = HTML_PAGE.replace(
                '<div id="imap-host"></div>',
                imap_html
            ).replace(
                '<div id="cv-pipeline-host"></div>',
                get_cv_pipeline_html()
            ).replace(
                '<div id="gps-pipeline-host"></div>',
                get_gps_pipeline_html()
            ).replace(
                '<div id="gps-charts-host"></div>',
                get_gps_charts_html()
            )
            # Set UI defaults from CLI flags
            page = page.replace("__CONF_VAL__", f"{args.conf:.2f}")
            page = page.replace("__SMART_SPREAD__", str(args.smart_radius))
            page = page.replace("__SMART_COUNT__", str(args.smart_min))
            # Set class filter checkboxes from CLI flag
            cf = runtime_state.get("class_filter", "all")
            if cf == "all":
                for cls in ("PERSON", "BIRD", "DUMMY", "OTHER"):
                    page = page.replace(f"__CLS_{cls}__", "checked")
            else:
                allowed = [c.strip().lower() for c in cf.split(',')]
                for cls in ("PERSON", "BIRD", "DUMMY", "OTHER"):
                    page = page.replace(f"__CLS_{cls}__", "checked" if cls.lower() in allowed else "")
            self.wfile.write(page.encode())

        elif path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                try:
                    with frame_lock:
                        jpeg = latest_jpeg
                    if jpeg:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                        self.wfile.flush()
                    time.sleep(0.033)  # ~30fps max stream (matches video fps)
                except BrokenPipeError:
                    break

        elif path == '/snapshot':
            with frame_lock:
                jpeg = latest_det_jpeg or latest_jpeg
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(503)
                self.end_headers()

        elif path == '/bullseye':
            with frame_lock:
                jpeg = latest_bullseye
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(503)
                self.end_headers()

        elif path == '/latest':
            with frame_lock:
                jpeg = latest_detection_jpeg
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(503)
                self.end_headers()

        elif path == '/best':
            with frame_lock:
                jpeg = latest_best_jpeg
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(503)
                self.end_headers()

        elif path == '/smart-grid':
            with frame_lock:
                jpeg = latest_smart_grid_jpeg
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(503)
                self.end_headers()

        elif path == '/map':
            # Serve raw map image (loaded once by interactive canvas)
            import os as _os
            map_path = config.MAP_FILE
            if _os.path.exists(map_path):
                with open(map_path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'public, max-age=86400')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()

        elif path == '/map-rendered':
            # Legacy: server-rendered map with dots (for non-JS clients)
            with frame_lock:
                jpeg = latest_map_jpeg
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(503)
                self.end_headers()

        elif path == '/api/drone':
            # Near-realtime drone telemetry for interactive map
            import json as _json
            with gps_lock:
                d = {
                    "lat": gps_data.get("lat", 0),
                    "lon": gps_data.get("lon", 0),
                    "alt": gps_data.get("alt", 0),
                    "yaw": gps_data.get("yaw", 0),
                    "sats": gps_data.get("sats", 0),
                    "mode": gps_data.get("mode", "---"),
                }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(_json.dumps(d).encode())

        elif path == '/api/estimates':
            # GPS detection estimates for interactive map
            import json as _json
            est_list = [[e[0], e[1]] for e in _all_gps_estimates]
            smart = None
            if smart_estimator and smart_estimator.locked:
                med = smart_estimator.get_median()
                if med:
                    smart = [med[0], med[1]]
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(_json.dumps({"estimates": est_list, "smart": smart}).encode())

        elif path == '/api/estimates-full':
            # Detailed GPS estimates for client-side charting
            import json as _json
            # Each entry: [lat, lon, pdist, alt, conf]
            est_list = [[e[0], e[1], e[2], e[3], e[4]] for e in _all_gps_estimates]
            # Smart estimator cluster info
            smart_data = None
            if smart_estimator:
                if smart_estimator.locked and smart_estimator.locked_cluster:
                    med = smart_estimator.get_median()
                    smart_data = {
                        "locked": True,
                        "cluster": [[c[0], c[1], c[2]] for c in smart_estimator.locked_cluster],
                        "spread": smart_estimator.locked_spread,
                        "median": [med[0], med[1]] if med else None,
                    }
                else:
                    smart_data = {"locked": False, "cluster": [], "spread": 0, "median": None}
            # Mean of all estimates
            mean = None
            if est_list:
                mean = [
                    sum(e[0] for e in est_list) / len(est_list),
                    sum(e[1] for e in est_list) / len(est_list),
                ]
            payload = {
                "estimates": est_list,
                "smart": smart_data,
                "drone": {"lat": gps_data.get("lat", 0), "lon": gps_data.get("lon", 0)},
                "mean": mean,
                "n": len(est_list),
                "ground_truth": _ground_truth,
                "best": _best_detection_gps,
                "smart_spread": smart_estimator.max_spread if smart_estimator else args.smart_radius,
                "smart_count": smart_estimator.min_samples if smart_estimator else args.smart_min,
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(_json.dumps(payload).encode())

        elif path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            import json
            with runtime_lock:
                stats["active_model_id"] = runtime_state["active_model_id"]
                stats["conf_threshold"] = runtime_state["conf_threshold"]
                stats["class_filter"] = runtime_state["class_filter"]
                stats["bullseye_map_bg"] = bullseye_map_bg
            if smart_estimator:
                stats["smart_spread"] = smart_estimator.max_spread
                stats["smart_count"] = smart_estimator.min_samples
            else:
                stats["smart_spread"] = args.smart_radius
                stats["smart_count"] = args.smart_min
            with frame_lock:
                status_json = json.dumps(stats).encode()
            self.wfile.write(status_json)

        elif path == '/api/switch-model':
            # GET /api/switch-model?id=0|1|2|3
            self._send_json_response(self._handle_switch_model())

        elif path == '/api/set-conf':
            # GET /api/set-conf?val=0.25
            self._send_json_response(self._handle_set_conf())

        elif path == '/api/set-class':
            # GET /api/set-class?name=person|dummy|all
            self._send_json_response(self._handle_set_class())

        elif path == '/api/reset-best':
            # Reset only best detection — starts tracking new best
            # Use sys.modules to get the RUNNING module (handles __main__ vs import)
            _mod = sys.modules.get('field_tools.passive_watch') or sys.modules.get('__main__')
            _mod._best_center_dist = 999.0
            _mod._best_detection_gps = None
            _mod._snap_request_best = False
            _mod._snap_jpeg_best = None
            with frame_lock:
                _mod.latest_best_jpeg = None
            print("[CLEAR] Best detection reset — tracking new best")
            self._send_json_response({"ok": True})

        elif path == '/api/set-ground-truth':
            # GET /api/set-ground-truth?lat=X&lon=Y
            params = self._parse_qs()
            try:
                gt_lat = float(params.get('lat', ''))
                gt_lon = float(params.get('lon', ''))
                _ground_truth = {"lat": gt_lat, "lon": gt_lon}
                print(f"[GT] Ground truth set: {gt_lat:.7f}, {gt_lon:.7f}")
                self._send_json_response({"ok": True, "ground_truth": _ground_truth})
            except (ValueError, TypeError):
                self._send_json_response({"ok": False, "error": "Invalid lat/lon"})
            return

        elif path == '/api/clear-ground-truth':
            _ground_truth = None
            print("[GT] Ground truth cleared")
            self._send_json_response({"ok": True})
            return

        elif path == '/api/toggle-bullseye-bg':
            bullseye_map_bg = not bullseye_map_bg
            self._send_json_response({"ok": True, "map_bg": bullseye_map_bg})

        elif path == '/api/clear-all':
            # this is a bit hacky but it works — we reset everything the state machine
            # touches by poking the module globals dict directly, because the HTTP handler
            # lives in a different module scope than the actual __main__ globals.
            _all_gps_estimates.clear()
            dummy_estimator.reset()
            if smart_estimator:
                smart_estimator.__init__(min_samples=smart_estimator.min_samples, max_spread=smart_estimator.max_spread)
                if hasattr(smart_estimator, '_saved'):
                    smart_estimator._saved = False
            _g = globals()
            _g['_best_center_dist'] = 999.0
            _g['_best_detection_gps'] = None
            _g['_snap_request_best'] = False
            _g['_snap_request_latest'] = False
            _g['_snap_jpeg_best'] = None
            _g['_snap_jpeg_latest'] = None
            draw_overlay._last_class = ''
            _g['_last_det'] = None
            _g['latest_detection_jpeg'] = None
            _g['latest_best_jpeg'] = None
            _g['latest_bullseye'] = None
            _g['latest_smart_grid_jpeg'] = None
            # Reset result mode flags — write to globals dict so inference thread sees it
            _g['_smart_result_saved'] = False
            _g['_survey_result_saved'] = False
            _g['_result_banner'] = None
            print("[CLEAR] All estimates, detections, plots, and result flags reset", flush=True)
            self._send_json_response({"ok": True})

        elif path == '/api/set-smart':
            # GET /api/set-smart?spread=X&count=Y — update SMART cluster parameters
            params = self._parse_qs()
            _mod = sys.modules.get('field_tools.passive_watch') or sys.modules.get('__main__')
            try:
                spread = float(params.get('spread', '0.75'))
                count = int(params.get('count', '10'))
                spread = max(0.1, min(5.0, spread))
                count = max(3, min(50, count))
                if smart_estimator:
                    smart_estimator.max_spread = spread
                    smart_estimator.min_samples = count
                    # Reset lock so new params can trigger a fresh result
                    smart_estimator.locked = False
                    smart_estimator.locked_cluster = None
                    smart_estimator.locked_cluster_indices = None
                    smart_estimator.locked_frame = None
                    smart_estimator.locked_spread = 0
                    smart_estimator.all_estimates = []  # fresh collection
                    if hasattr(smart_estimator, '_saved'):
                        smart_estimator._saved = False
                _mod._smart_result_saved = False
                print(f"[SMART] Parameters updated: spread={spread}m, count={count}")
                self._send_json_response({"ok": True, "spread": spread, "count": count})
            except (ValueError, TypeError):
                self._send_json_response({"ok": False, "error": "Invalid spread/count"})

        else:
            self.send_response(404)
            self.end_headers()

    def _send_json_response(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _parse_qs(self):
        from urllib.parse import urlparse, parse_qs
        qs = parse_qs(urlparse(self.path).query)
        out = {}
        for k in qs:
            out[k] = qs[k][0]
        return out

    def _handle_switch_model(self):
        params = self._parse_qs()
        try:
            mid = int(params.get('id', -1))
        except (ValueError, TypeError):
            return {"ok": False, "error": "invalid id"}
        if mid < 0 or mid >= len(MODEL_TABLE):
            return {"ok": False, "error": f"id must be 0-{len(MODEL_TABLE)-1}"}
        _mod = sys.modules.get('field_tools.passive_watch') or sys.modules.get('__main__')
        with runtime_lock:
            runtime_state["model_switch_request"] = mid
            runtime_state["model_switch_error"] = None
        # Wait for background switch thread (up to 120s — int8 models need dequantization)
        for _ in range(600):
            time.sleep(0.2)
            with runtime_lock:
                err = runtime_state["model_switch_error"]
                if err is not None:
                    runtime_state["model_switch_error"] = None
                    return {"ok": False, "error": err}
                if runtime_state["active_model_id"] == mid and not getattr(_mod, '_model_switch_active', False):
                    return {"ok": True, "model": MODEL_TABLE[mid]["name"],
                            "id": mid, "path": MODEL_TABLE[mid]["path"]}
        return {"ok": False, "error": "timeout waiting for model switch"}

    def _handle_set_conf(self):
        params = self._parse_qs()
        try:
            val = float(params.get('val', -1))
        except (ValueError, TypeError):
            return {"ok": False, "error": "invalid val"}
        if val < 0.01 or val > 0.99:
            return {"ok": False, "error": "val must be 0.01-0.99"}
        with runtime_lock:
            runtime_state["conf_threshold"] = val
        return {"ok": True, "conf": val}

    def _handle_set_class(self):
        params = self._parse_qs()
        name = params.get('name', '').strip().lower()
        if not name:
            return {"ok": False, "error": "missing name"}
        with runtime_lock:
            runtime_state["class_filter"] = name
        # Reset best + latest detection when class filter changes (old detections may be wrong class)
        _mod = sys.modules.get('field_tools.passive_watch') or sys.modules.get('__main__')
        _mod._best_center_dist = 999.0
        _mod._best_detection_gps = None
        # Clear pending snapshot flags so stale pre-filter JPEGs don't get restored
        _mod._snap_request_best = False
        _mod._snap_request_latest = False
        _mod._snap_jpeg_best = None
        _mod._snap_jpeg_latest = None
        # Clear the stale class/detection shown on the live overlay
        draw_overlay._last_class = ''
        _mod._last_det = None
        with frame_lock:
            _mod.latest_best_jpeg = None
            _mod.latest_detection_jpeg = None
        return {"ok": True, "class_filter": name}


class ThreadedServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


# stats — FOV fields are populated further down once get_fov_info is defined
stats = {
    "frames": 0, "detections": 0, "det_pct": "0", "saved": 0,
    "cam_fps": "0.0", "vis_fps": "0.0", "stream_fps": "0.0",
    "gps_lat": "---", "gps_lon": "---", "alt": "---", "sats": 0, "flight_mode": "---",
    "est_lat": "---", "est_lon": "---", "est_obs": 0,
    "fov_deg": "---", "cal_1m_w": "---", "cal_1m_h": "---",
}

# rolling-window FPS
class RollingFPS:
    def __init__(self, window=3.0):
        self.window = window
        self.times = []

    def tick(self):
        now = time.time()
        self.times.append(now)
        cutoff = now - self.window
        # TODO: deque with maxlen would be tidier than filtering every tick
        self.times = [t for t in self.times if t > cutoff]

    def fps(self):
        if len(self.times) < 2:
            return 0.0
        span = self.times[-1] - self.times[0]
        if span <= 0:
            return 0.0
        return (len(self.times) - 1) / span

cam_fps_tracker = RollingFPS()
vis_fps_tracker = RollingFPS()
stream_fps_tracker = RollingFPS()

# FOV + calibration bits
def get_fov_info():
    sw = config.SENSOR_WIDTH_MM
    fl = config.FOCAL_LENGTH_MM
    iw = config.IMAGE_W
    ih = config.IMAGE_H

    hfov_deg = 2 * math.degrees(math.atan(sw / (2 * fl)))
    vfov_deg = hfov_deg * ih / iw  # square pixels assumed
    f_px = fl * iw / sw

    return {
        "hfov_deg": hfov_deg,
        "vfov_deg": vfov_deg,
        "f_px": f_px,
        "sensor_w": sw,
        "focal_mm": fl,
        "img_w": iw,
        "img_h": ih,
    }

def ground_coverage(alt_m):
    # returns (width_m, height_m) of ground visible at alt_m
    sw = config.SENSOR_WIDTH_MM
    fl = config.FOCAL_LENGTH_MM
    w = alt_m * sw / fl
    h = w * config.IMAGE_H / config.IMAGE_W
    return w, h

FOV  =  get_fov_info()

# populate FOV stats now that FOV is built
stats["fov_deg"] = f"{FOV['hfov_deg']:.0f}"
stats["cal_1m_w"] = f"{ground_coverage(1.0)[0]*100:.0f}"
stats["cal_1m_h"] = f"{ground_coverage(1.0)[1]*100:.0f}"


# accumulates detection observations and spits out a weighted GPS estimate for the dummy.
# weight is 1/alt^2 with a bonus for detections near frame centre.
class DummyEstimator:
    def __init__(self):
        self.total_weight = 0.0
        self.weighted_lat = 0.0
        self.weighted_lon = 0.0
        self.count = 0

    def _flat_earth_estimate(self, drone_lat, drone_lon, alt_m, yaw_deg,
                             det_x, det_y, fw, fh):
        # flat-earth projection, no tilt compensation
        dx_px = (det_x - 0.5) * fw
        dy_px = (det_y - 0.5) * fh

        # Metres on ground
        dx_m = dx_px * alt_m / FOV["f_px"]
        dy_m = dy_px * alt_m / FOV["f_px"]

        # Rotate by yaw (yaw=0 means North, positive clockwise)
        yaw_rad = math.radians(yaw_deg)
        forward_m = -dy_m   # negative dy = forward = North
        right_m = dx_m      # positive dx = right = East
        north_m = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
        east_m = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)

        lat_m_per_deg = 111320
        lon_m_per_deg = 111320 * math.cos(math.radians(drone_lat))
        return (drone_lat + north_m / lat_m_per_deg,
                drone_lon + east_m / lon_m_per_deg)

    def add_observation(self, drone_lat, drone_lon, alt_m, yaw_deg, det_x, det_y,
                        pitch_deg=0.0, roll_deg=0.0, compensate_tilt=False):
        # det_x/det_y are normalised (0-1). Returns (est_lat, est_lon) or None.
        # compensate_tilt triggers the full ray-trace path for non-level drones.
        if alt_m < 0.3:
            alt_m = 0.3  # clamp, avoid /0 when on the ground

        fw = FOV["img_w"]
        fh = FOV["img_h"]

        if compensate_tilt and (abs(pitch_deg) > 0.5 or abs(roll_deg) > 0.5):
            # ── Tilt-compensated ray-trace projection ──
            # Full rotation matrix R = Rz(yaw) @ Ry(-pitch) @ Rx(-roll)
            # SAME verified approach as main.py (proven with 1440 tests)
            focal_px = config.FOCAL_LENGTH_MM / config.SENSOR_WIDTH_MM * fw
            u = det_x * fw  # pixel x
            v = det_y * fh  # pixel y

            # Ray from detection pixel in camera frame
            ray_cam = [
                (u - fw / 2) / focal_px,
                (v - fh / 2) / focal_px,
                1.0
            ]

            yaw_rad = math.radians(yaw_deg)
            pitch_rad = math.radians(pitch_deg)
            roll_rad = math.radians(roll_deg)

            # R = Rz(yaw) @ Ry(-pitch) @ Rx(-roll)
            cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
            cp, sp = math.cos(-pitch_rad), math.sin(-pitch_rad)
            cr, sr = math.cos(-roll_rad), math.sin(-roll_rad)

            # Rx(-roll)
            rx0 = ray_cam[0]
            rx1 = cr * ray_cam[1] - sr * ray_cam[2]
            rx2 = sr * ray_cam[1] + cr * ray_cam[2]

            # Ry(-pitch) @ Rx(-roll)
            ry0 = cp * rx0 + sp * rx2
            ry1 = rx1
            ry2 = -sp * rx0 + cp * rx2

            # Rz(yaw) @ Ry(-pitch) @ Rx(-roll)
            rw0 = cy * ry0 - sy * ry1
            rw1 = sy * ry0 + cy * ry1
            rw2 = ry2

            # Intersect with ground plane (NED: X=North, Y=East, Z=Down)
            if rw2 > 0.01:
                t = alt_m / rw2
                north_m = rw0 * t
                east_m = rw1 * t

                # Convert directly to GPS (matches main.py approach)
                lat_m_per_deg = 111132.0
                lon_m_per_deg = 111132.0 * math.cos(math.radians(drone_lat))
                est_lat = drone_lat + north_m / lat_m_per_deg
                est_lon = drone_lon + east_m / lon_m_per_deg
            else:
                # Ray nearly horizontal — fall back to flat-earth (no tilt)
                est_lat, est_lon = self._flat_earth_estimate(
                    drone_lat, drone_lon, alt_m, yaw_deg, det_x, det_y, fw, fh)
        else:
            # ── Original flat-earth projection (no tilt compensation) ──
            est_lat, est_lon = self._flat_earth_estimate(
                drone_lat, drone_lon, alt_m, yaw_deg, det_x, det_y, fw, fh)

        # Weight: inverse altitude squared (10m obs is 9x more valuable than 30m)
        weight = 1.0 / (alt_m * alt_m)

        # bonus: detection near frame center = less projection error
        dx_norm = det_x - 0.5  # normalized offset from centre (-0.5 to 0.5)
        dy_norm = det_y - 0.5
        dist_from_centre = math.sqrt(dx_norm**2 + dy_norm**2)
        max_dist = math.sqrt(0.5**2 + 0.5**2)  # ~0.707
        centre_factor = 1.0 + 4.0 * max(0, 1.0 - dist_from_centre / (max_dist * 0.3))
        weight *= centre_factor

        self.weighted_lat += est_lat * weight
        self.weighted_lon += est_lon * weight
        self.total_weight += weight
        self.count += 1

        return est_lat, est_lon

    def get_estimate(self):
        # (lat, lon, n) or None
        if self.total_weight <= 0:
            return None
        return (
            self.weighted_lat / self.total_weight,
            self.weighted_lon / self.total_weight,
            self.count
        )

    def reset(self):
        self.total_weight = 0.0
        self.weighted_lat = 0.0
        self.weighted_lon = 0.0
        self.count = 0

dummy_estimator = DummyEstimator()


# GPS target-lock via greedy tightest-cluster consensus.
# Each detection adds a noisy (lat,lon) estimate. Once enough estimates
# agree within max_spread metres we LOCK and expose the median. Not thread-safe;
# only the main loop writes, the HTTP server reads after lock (effectively immutable).
class SmartEstimator:
    def __init__(self, min_samples=5, max_spread=1.0):
        self.min_samples = min_samples
        self.max_spread = max_spread
        self.all_estimates = []  # list of (est_lat, est_lon, pixel_dist, frame)
        self.locked = False
        self.lock_id = getattr(self, 'lock_id', 0)  # survives __init__ resets
        self.locked_cluster = None  # list of (lat, lon, pixel_dist, frame) for the winning cluster
        self.locked_cluster_indices = None  # indices into all_estimates (for frame updates)
        self.locked_frame = None    # frame with smallest pixel_dist (most centred detection)
        self.locked_spread = 0      # max pairwise distance in locked cluster (metres)

    def add(self, est_lat, est_lon, pixel_dist, frame):
        # returns True only on the lock-transition frame. Frame can be None
        # if update_last_frame() will be called to substitute the overlayed HUD.
        if self.locked:
            return False
        self.all_estimates.append((est_lat, est_lon, pixel_dist, frame.copy() if frame is not None else None))

        if len(self.all_estimates) < self.min_samples:
            return False

        # Find tightest 10
        indices, spread = self._find_tightest(self.min_samples)
        n = len(self.all_estimates)
        print(f"  [SMART] {n} detections, tightest {self.min_samples} spread: {spread:.2f}m (need <{self.max_spread}m)")

        if spread < self.max_spread:
            # LOCKED — save cluster
            self.locked = True
            self.lock_id += 1
            self.locked_spread = spread
            self.locked_cluster_indices = indices
            cluster = [self.all_estimates[i] for i in indices]
            self.locked_cluster = [(e[0], e[1], e[2], e[3]) for e in cluster]
            # Most central frame (smallest pixel_dist)
            best = min(cluster, key=lambda e: e[2])
            self.locked_frame = best[3]
            med = self.get_median()
            print(f"  [SMART] *** LOCKED! Spread: {spread:.2f}m ***")
            print(f"  [SMART] Median: {med[0]:.7f}, {med[1]:.7f} ({med[2]} samples)")
            return True
        return False

    def update_last_frame(self, display):
        # Swap the raw frame of the most recent estimate for the overlayed HUD frame,
        # so the smart-grid thumbnails show boxes/bars instead of bare camera.
        if not self.all_estimates:
            return
        last = self.all_estimates[-1]
        self.all_estimates[-1] = (last[0], last[1], last[2], display.copy())
        # If we just locked, refresh the cluster frames from all_estimates
        if self.locked and self.locked_cluster_indices is not None:
            cluster = [self.all_estimates[i] for i in self.locked_cluster_indices]
            self.locked_cluster = [(e[0], e[1], e[2], e[3]) for e in cluster]
            best = min(cluster, key=lambda e: e[2])
            self.locked_frame = best[3]

    def _find_tightest(self, target_size):
        # greedy tightest-cluster: seed with closest pair, keep adding the point
        # that grows the cluster's spread by the least. O(n^2) but fine for <100 points.
        estimates = self.all_estimates
        n = len(estimates)

        # Step 1: Pairwise distances (metres) using equirectangular projection
        mean_lat = sum(e[0] for e in estimates) / n
        dists = {}
        for i in range(n):
            for j in range(i + 1, n):
                dn = (estimates[i][0] - estimates[j][0]) * 111320
                de = (estimates[i][1] - estimates[j][1]) * 111320 * math.cos(math.radians(mean_lat))
                dists[(i, j)] = math.sqrt(dn**2 + de**2)

        # Edge case: fewer points than target — return all of them
        if n <= target_size:
            spread = max(dists.values()) if dists else 0
            return list(range(n)), spread

        # Step 2: Seed cluster with the closest pair
        min_pair = min(dists, key=dists.get)
        cluster = set(min_pair)

        # Step 3: Greedily add the point that minimises the cluster's max spread
        while len(cluster) < target_size:
            best_pt = -1
            best_spread = float('inf')
            for c in range(n):
                if c in cluster:
                    continue
                # What would the max distance be if we added point c?
                max_d = max(dists.get((min(c, m), max(c, m)), 0) for m in cluster)
                if max_d < best_spread:
                    best_spread = max_d
                    best_pt = c
            if best_pt >= 0:
                cluster.add(best_pt)
            else:
                break

        # Step 4: Compute the actual max pairwise distance in the final cluster
        cluster_list = sorted(cluster)
        spread = 0
        for i in cluster_list:
            for j in cluster_list:
                if i < j:
                    spread = max(spread, dists.get((i, j), 0))
        return cluster_list, spread

    def ready(self):
        return self.locked

    def get_median(self):
        # component-wise median (robust to outliers) → (lat, lon, n) or None
        if not self.locked_cluster:
            return None
        lats = sorted(e[0] for e in self.locked_cluster)
        lons = sorted(e[1] for e in self.locked_cluster)
        n = len(lats)
        mid = n // 2
        if n % 2 == 0:
            m_lat = (lats[mid - 1] + lats[mid]) / 2
            m_lon = (lons[mid - 1] + lons[mid]) / 2
        else:
            m_lat = lats[mid]
            m_lon = lons[mid]
        return m_lat, m_lon, n

    def get_cep50(self):
        # median distance (m) from cluster centre — i.e. CEP50
        med = self.get_median()
        if not med:
            return 0
        m_lat, m_lon, _ = med
        dists = []
        for lat, lon, *_ in self.locked_cluster:
            dn = (lat - m_lat) * 111320
            de = (lon - m_lon) * 111320 * math.cos(math.radians(m_lat))
            dists.append(math.sqrt(dn**2 + de**2))
        dists.sort()
        return dists[len(dists) // 2] if dists else 0

smart_estimator = None  # initialized in main() if --smart-estimate


def _snapshot_overlay(display, label=None, thumb_w=728, gps_info=None):
    # resized JPEG of the overlayed display frame for Latest/Best panels.
    # label is "LATEST" or "BEST". gps_info adds a drone/estimate info bar at the bottom.
    if display is None:
        return None
    h, w = display.shape[:2]
    s = thumb_w / w
    thumb = cv2.resize(display, (thumb_w, int(h * s)))
    th = thumb.shape[0]

    # --- Label badge (top-left) ---
    if label:
        badge_color = (0, 200, 0) if label == "BEST" else (200, 160, 0)  # green / teal
        font_scale = 0.7 if thumb_w >= 1024 else 0.5
        thickness = 2 if thumb_w >= 1024 else 1
        (tw_txt, th_txt), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        pad = 6
        cv2.rectangle(thumb, (0, 0), (tw_txt + pad * 2, th_txt + pad * 2), (0, 0, 0), -1)
        cv2.putText(thumb, label, (pad, th_txt + pad), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, badge_color, thickness, cv2.LINE_AA)

    # --- Info bar at bottom ---
    # First, paint a fully opaque black rectangle over the bottom 75px to cover
    # the camera feed's own GPS/FOV/estimate bars that are baked into the display frame.
    # Without this, those old bars bleed through as faint text behind the snapshot info.
    cover_h = 75  # covers GPS bar (25px) + FOV bar (25px) + estimate bar (25px)
    cv2.rectangle(thumb, (0, th - cover_h), (thumb_w, th), (0, 0, 0), -1)

    if gps_info:
        bar_h = 52 if thumb_w >= 1024 else 38
        # Info bar is drawn directly (fully opaque) on the already-cleared area
        cv2.rectangle(thumb, (0, th - bar_h), (thumb_w, th), (0, 0, 0), -1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        fs = 0.45 if thumb_w >= 1024 else 0.32
        lw = 1
        y_line1 = th - bar_h + 16 if thumb_w >= 1024 else th - bar_h + 13
        y_line2 = y_line1 + 18 if thumb_w >= 1024 else y_line1 + 14

        d_lat = gps_info.get("drone_lat", 0)
        d_lon = gps_info.get("drone_lon", 0)
        e_lat = gps_info.get("est_lat")
        e_lon = gps_info.get("est_lon")

        # Line 1: drone position
        drone_txt = f"DRONE: {d_lat:.7f}, {d_lon:.7f}"
        cv2.putText(thumb, drone_txt, (6, y_line1), font, fs, (255, 255, 0), lw, cv2.LINE_AA)  # cyan BGR

        # Line 2: dummy estimate + offset
        if e_lat is not None and e_lon is not None:
            dn = (d_lat - e_lat) * 111320
            de = (d_lon - e_lon) * 111320 * math.cos(math.radians(d_lat))
            offset_m = math.sqrt(dn ** 2 + de ** 2)
            est_txt = f"DUMMY EST: {e_lat:.7f}, {e_lon:.7f}"
            cls_name = gps_info.get("cls", "?")
            det_conf_snap = gps_info.get("conf", 0)
            off_txt = f"OFFSET: {offset_m:.1f}m  [{cls_name} {det_conf_snap:.2f}]"
            cv2.putText(thumb, est_txt, (6, y_line2), font, fs, (255, 0, 255), lw, cv2.LINE_AA)  # magenta
            # Offset + class after estimate text
            est_tw, _ = cv2.getTextSize(est_txt, font, fs, lw)
            cv2.putText(thumb, "  " + off_txt, (6 + est_tw[0], y_line2), font, fs, (0, 255, 255), lw, cv2.LINE_AA)  # yellow
        else:
            cv2.putText(thumb, "DUMMY EST: waiting...", (6, y_line2), font, fs, (128, 128, 128), lw, cv2.LINE_AA)

    quality = 92 if thumb_w >= 1024 else 85
    _, jpg = cv2.imencode('.jpg', thumb, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return jpg.tobytes()


_map_full = None  # cached full-res map.jpg

def _build_map_panel(gps_lat, gps_lon, target_width):
    # returns a BGR map crop with a magenta star at (lat, lon), or None if map.jpg missing
    global _map_full
    if _map_full is None:
        map_path = config.MAP_FILE
        if os.path.exists(map_path):
            _map_full = cv2.imread(map_path)
        if _map_full is None:
            return None

    full_h, full_w = _map_full.shape[:2]

    # GPS → pixel on full map
    dn = (config.REF_LAT - gps_lat) * 111320
    de = (gps_lon - config.REF_LON) * 111320 * math.cos(math.radians(config.REF_LAT))
    cx = int(de / config.MAP_WIDTH_METERS * full_w)
    cy = int(dn / config.MAP_WIDTH_METERS * full_w)

    # Resize full map to target width, preserving aspect ratio
    panel_h = int(full_h * target_width / full_w)
    panel = cv2.resize(_map_full, (target_width, panel_h))

    # Helper: GPS → panel pixel
    def to_px(lat, lon):
        _dn = (config.REF_LAT - lat) * 111320
        _de = (lon - config.REF_LON) * 111320 * math.cos(math.radians(config.REF_LAT))
        return (int(_de / config.MAP_WIDTH_METERS * target_width),
                int(_dn / config.MAP_WIDTH_METERS * target_width))

    # Draw SSSI no-fly zone polygon (red)
    if hasattr(config, 'SSSI_GPS') and config.SSSI_GPS:
        pts = np.array([to_px(lat, lon) for lat, lon in config.SSSI_GPS], dtype=np.int32)
        cv2.polylines(panel, [pts], True, (0, 0, 255), 2, cv2.LINE_AA)

    # Draw search area polygon (yellow)
    if hasattr(config, 'SEARCH_AREA_GPS') and config.SEARCH_AREA_GPS:
        pts = np.array([to_px(lat, lon) for lat, lon in config.SEARCH_AREA_GPS], dtype=np.int32)
        cv2.polylines(panel, [pts], True, (0, 255, 255), 2, cv2.LINE_AA)

    # Star position in panel pixels
    star_x = int(cx * target_width / full_w)
    star_y = int(cy * target_width / full_w)
    star_x = max(15, min(target_width - 15, star_x))
    star_y = max(15, min(panel_h - 15, star_y))

    # Draw magenta star
    cv2.drawMarker(panel, (star_x, star_y), (255, 0, 255), cv2.MARKER_STAR, 25, 3)
    # White circle outline for visibility
    cv2.circle(panel, (star_x, star_y), 18, (255, 255, 255), 1, cv2.LINE_AA)

    # Label bar at top of map panel
    cv2.rectangle(panel, (0, 0), (target_width, 22), (0, 0, 0), -1)
    coord_text = f"MAP: {gps_lat:.7f}, {gps_lon:.7f}"
    cv2.putText(panel, coord_text, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (255, 0, 255), 1, cv2.LINE_AA)

    return panel


def generate_result_image(frame, title, gps_lat, gps_lon, stats_text, filename,
                          title_color=(0, 255, 0), drone_lat=None, drone_lon=None,
                          gps_label="DUMMY EST"):
    # final result image: big banner + GPS coord + stats line, saved to filename.
    # if drone_lat/lon supplied we also show DRONE: and OFFSET: lines.
    if frame is None:
        return None
    result = frame.copy()
    h, w = result.shape[:2]

    # Determine banner height based on whether we have drone GPS
    has_drone = drone_lat is not None and drone_lon is not None
    banner_h = 180 if has_drone else 120

    # Large banner at top (semi-transparent black background)
    overlay = result.copy()
    cv2.rectangle(overlay, (0, 0), (w, banner_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, result, 0.3, 0, result)

    # Title text (large, bold)
    font = cv2.FONT_HERSHEY_SIMPLEX
    # Outline for readability
    cv2.putText(result, title, (15, 45), font, 1.4, (0, 0, 0), 6, cv2.LINE_AA)
    cv2.putText(result, title, (15, 45), font, 1.4, title_color, 3, cv2.LINE_AA)

    # GPS coordinate (large monospace-style text)
    gps_text = f"{gps_label}: {gps_lat:.7f}, {gps_lon:.7f}"
    cv2.putText(result, gps_text, (15, 80), font, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(result, gps_text, (15, 80), font, 0.7, (255, 0, 255), 2, cv2.LINE_AA)  # magenta

    # Stats text (smaller, below GPS)
    cv2.putText(result, stats_text, (15, 110), font, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(result, stats_text, (15, 110), font, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

    # Drone GPS + offset (if available)
    if has_drone:
        drone_text = f"DRONE: {drone_lat:.7f}, {drone_lon:.7f}"
        cv2.putText(result, drone_text, (15, 140), font, 0.65, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(result, drone_text, (15, 140), font, 0.65, (255, 255, 0), 2, cv2.LINE_AA)  # cyan

        # Compute offset distance
        dn = (gps_lat - drone_lat) * 111320
        de = (gps_lon - drone_lon) * 111320 * math.cos(math.radians(drone_lat))
        offset_m = math.sqrt(dn ** 2 + de ** 2)
        offset_text = f"OFFSET: {offset_m:.1f}m"
        cv2.putText(result, offset_text, (15, 170), font, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(result, offset_text, (15, 170), font, 0.7, (0, 255, 255), 2, cv2.LINE_AA)  # yellow

    # ── Build map panel with star at coordinate ──
    map_panel = _build_map_panel(gps_lat, gps_lon, w)

    if map_panel is not None:
        # Stack vertically: detection frame on top, map on bottom
        result = np.vstack([result, map_panel])

    # Save (PNG for lossless, JPEG otherwise)
    os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
    if filename.lower().endswith('.png'):
        cv2.imwrite(filename, result)
    else:
        cv2.imwrite(filename, result, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return result


def compute_survey_analysis(all_estimates):
    # picks the best coordinate from all GPS estimates using several methods
    # (mean, weighted, median, tightest-10, central-only) and returns a dict.
    n = len(all_estimates)
    if n == 0:
        return None

    lats = [e[0] for e in all_estimates]
    lons = [e[1] for e in all_estimates]

    # 1. Simple mean
    mean_lat = sum(lats) / n
    mean_lon = sum(lons) / n

    # 2. Weighted mean (by centrality: 1/pixel_dist^2, clamped)
    w_lat, w_lon, total_w = 0.0, 0.0, 0.0
    for e in all_estimates:
        pdist = max(e[2], 1.0)
        w = 1.0 / (pdist * pdist)
        w_lat += e[0] * w
        w_lon += e[1] * w
        total_w += w
    weighted_lat = w_lat / total_w if total_w > 0 else mean_lat
    weighted_lon = w_lon / total_w if total_w > 0 else mean_lon

    # 3. Median
    sorted_lats = sorted(lats)
    sorted_lons = sorted(lons)
    mid = n // 2
    if n % 2 == 0:
        median_lat = (sorted_lats[mid - 1] + sorted_lats[mid]) / 2
        median_lon = (sorted_lons[mid - 1] + sorted_lons[mid]) / 2
    else:
        median_lat = sorted_lats[mid]
        median_lon = sorted_lons[mid]

    # 4. Tightest-10 cluster median (reuse SmartEstimator logic)
    temp_smart = SmartEstimator(min_samples=min(10, n), max_spread=999.0)
    for e in all_estimates:
        temp_smart.all_estimates.append((e[0], e[1], e[2], None))
    if len(temp_smart.all_estimates) >= temp_smart.min_samples:
        indices, spread = temp_smart._find_tightest(temp_smart.min_samples)
        temp_smart.locked = True
        temp_smart.locked_cluster = [(temp_smart.all_estimates[i][0],
                                       temp_smart.all_estimates[i][1],
                                       temp_smart.all_estimates[i][2]) for i in indices]
        temp_smart.locked_spread = spread
    tight_med = temp_smart.get_median()
    tight_lat = tight_med[0] if tight_med else median_lat
    tight_lon = tight_med[1] if tight_med else median_lon
    tight_spread = temp_smart.locked_spread if temp_smart.locked else 0

    # 5. Central-only median (pixel_dist < 200)
    central = [e for e in all_estimates if e[2] < 200]
    if len(central) >= 3:
        c_lats = sorted(e[0] for e in central)
        c_lons = sorted(e[1] for e in central)
        c_mid = len(central) // 2
        central_lat = c_lats[c_mid]
        central_lon = c_lons[c_mid]
    else:
        central_lat = median_lat
        central_lon = median_lon

    # Pick the BEST method: use tightest-10 cluster as reference, then pick closest
    # The tightest cluster is the most robust against outliers
    candidates = {
        "mean": (mean_lat, mean_lon),
        "weighted": (weighted_lat, weighted_lon),
        "median": (median_lat, median_lon),
        "tight-10": (tight_lat, tight_lon),
        "central": (central_lat, central_lon),
    }

    # Distance from each candidate to the tightest-10 cluster
    ref_lat, ref_lon = tight_lat, tight_lon
    best_method = "tight-10"
    best_dist = 0
    best_lat, best_lon = tight_lat, tight_lon

    for name, (clat, clon) in candidates.items():
        dn = (clat - ref_lat) * 111320
        de = (clon - ref_lon) * 111320 * math.cos(math.radians(ref_lat))
        d = math.sqrt(dn ** 2 + de ** 2)
        if name == "tight-10":
            best_dist = d  # 0 by definition
            continue

    # CEP50 from the chosen best estimate
    dists_from_best = []
    for e in all_estimates:
        dn = (e[0] - best_lat) * 111320
        de = (e[1] - best_lon) * 111320 * math.cos(math.radians(best_lat))
        dists_from_best.append(math.sqrt(dn ** 2 + de ** 2))
    dists_from_best.sort()
    cep50 = dists_from_best[len(dists_from_best) // 2] if dists_from_best else 0

    stats_text = (f"N={n}  CEP50={cep50:.2f}m  method={best_method}  "
                  f"tight10_spread={tight_spread:.2f}m  central={len(central)}")

    return {
        "lat": best_lat, "lon": best_lon,
        "method": best_method, "cep50": cep50,
        "stats_text": stats_text,
        "n": n, "tight_spread": tight_spread,
        "n_central": len(central),
    }


def render_smart_grid(smart_est):
    # 5x2 thumbnail grid. Before lock: all collected frames in order.
    # After lock: only the tightest cluster frames (same ones shown on the bullseye).
    global latest_smart_grid_jpeg
    if not smart_est:
        return

    best_frame_idx = -1  # index of most central frame (smallest pixel_dist)
    if smart_est.locked and smart_est.locked_cluster:
        # After lock: show ONLY the tightest cluster frames
        frames = [e[3] for e in smart_est.locked_cluster if len(e) >= 4 and e[3] is not None]
        # Find most central frame (smallest pixel_dist, index 2 in tuple)
        valid = [(i, e[2]) for i, e in enumerate(smart_est.locked_cluster) if len(e) >= 4 and e[3] is not None]
        if valid:
            best_frame_idx = min(valid, key=lambda x: x[1])[0]
    else:
        # Before lock: show all collected frames chronologically
        frames = [e[3] for e in smart_est.all_estimates if len(e) >= 4 and e[3] is not None]

    n_have = len(frames)
    n_need = smart_est.min_samples
    cw, ch = 240, 180
    banner_h = 28
    grid = np.zeros((ch * 2 + banner_h, cw * 5, 3), dtype=np.uint8)

    # --- Status banner at top ---
    if smart_est.locked:
        banner_color = (0, 140, 0)   # dark green
        med = smart_est.get_median()
        if med:
            status_text = f"LOCKED  {n_need}/{n_need}  spread {smart_est.locked_spread:.2f}m  |  {med[0]:.7f}, {med[1]:.7f}"
        else:
            status_text = f"LOCKED  {n_need}/{n_need}  spread {smart_est.locked_spread:.2f}m"
    elif n_have == 0:
        banner_color = (60, 60, 60)
        status_text = f"WAITING FOR DETECTIONS  0/{n_need}"
    else:
        banner_color = (0, 100, 180)  # dark orange/amber
        status_text = f"COLLECTING  {n_have}/{n_need}"
    cv2.rectangle(grid, (0, 0), (cw * 5, banner_h), banner_color, -1)
    cv2.putText(grid, status_text, (8, banner_h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    # --- Thumbnail slots ---
    y_off = banner_h
    for i in range(min(n_need, 10)):
        r, c = i // 5, i % 5
        y0 = y_off + r * ch
        x0 = c * cw
        if i < n_have:
            # Filled slot — show detection frame
            thumb = cv2.resize(frames[i], (cw, ch))
            grid[y0:y0 + ch, x0:x0 + cw] = thumb

            is_best = (smart_est.locked and i == best_frame_idx)
            if is_best:
                # Most central frame — green border (3px) + star label
                cv2.rectangle(grid, (x0, y0), (x0 + cw - 1, y0 + ch - 1), (0, 255, 0), 3)
                label = f"#{i+1} \u2605"
                cv2.putText(grid, label, (x0 + 5, y0 + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(grid, label, (x0 + 5, y0 + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
                # GPS coordinates of the best frame (from locked cluster)
                best_entry = smart_est.locked_cluster[best_frame_idx]
                coord_text = f"{best_entry[0]:.7f}, {best_entry[1]:.7f}"
                cv2.putText(grid, coord_text, (x0 + 5, y0 + ch - 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(grid, coord_text, (x0 + 5, y0 + ch - 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 0), 1, cv2.LINE_AA)
                # "BEST" badge bottom-right
                cv2.putText(grid, "BEST", (x0 + cw - 48, y0 + ch - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(grid, "BEST", (x0 + cw - 48, y0 + ch - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
            else:
                # Normal frame — white number label
                cv2.putText(grid, f"#{i+1}", (x0 + 5, y0 + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(grid, f"#{i+1}", (x0 + 5, y0 + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1, cv2.LINE_AA)
                # Border: green if locked, yellow/amber if still collecting
                if smart_est.locked:
                    cv2.rectangle(grid, (x0, y0), (x0 + cw - 1, y0 + ch - 1), (0, 255, 0), 2)
                else:
                    cv2.rectangle(grid, (x0, y0), (x0 + cw - 1, y0 + ch - 1), (0, 200, 255), 1)
        else:
            # Empty slot — dim placeholder
            cv2.rectangle(grid, (x0 + 2, y0 + 2), (x0 + cw - 3, y0 + ch - 3), (50, 50, 50), 1)
            cv2.putText(grid, f"#{i+1}", (x0 + cw // 2 - 12, y0 + ch // 2 + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (50, 50, 50), 1, cv2.LINE_AA)

    _, jpg = cv2.imencode('.jpg', grid, [cv2.IMWRITE_JPEG_QUALITY, 80])
    with frame_lock:
        latest_smart_grid_jpeg = jpg.tobytes()


def render_map(all_estimates, smart_est=None):
    global latest_map_jpeg, _map_base
    if _map_base is None:
        # Try loading map
        map_path = config.MAP_FILE
        if os.path.exists(map_path):
            img = cv2.imread(map_path)
            if img is not None:
                _map_base = cv2.resize(img, (600, int(img.shape[0] * 600 / img.shape[1])))
        if _map_base is None:
            # Placeholder
            _map_base = np.zeros((400, 600, 3), dtype=np.uint8)
            cv2.putText(_map_base, "map.jpg not found", (150, 200),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 1)
    disp = _map_base.copy()
    mh, mw = disp.shape[:2]

    def gps_to_px(lat, lon):
        dn = (config.REF_LAT - lat) * 111320
        de = (lon - config.REF_LON) * 111320 * math.cos(math.radians(config.REF_LAT))
        px = int(de / config.MAP_WIDTH_METERS * mw)
        py = int(dn / config.MAP_WIDTH_METERS * mw)
        return px, py

    # Plot all estimates
    for e in all_estimates:
        lat, lon = e[0], e[1]
        px, py = gps_to_px(lat, lon)
        if 0 <= px < mw and 0 <= py < mh:
            cv2.circle(disp, (px, py), 3, (0, 255, 0), -1)

    # Smart median star
    if smart_est and smart_est.locked:
        med = smart_est.get_median()
        if med:
            px, py = gps_to_px(med[0], med[1])
            if 0 <= px < mw and 0 <= py < mh:
                cv2.drawMarker(disp, (px, py), (255, 0, 255), cv2.MARKER_STAR, 15, 2)

    # Drone position
    with gps_lock:
        _drone_lat, _drone_lon = gps_data["lat"], gps_data["lon"]
    if _drone_lat != 0:
        px, py = gps_to_px(_drone_lat, _drone_lon)
        if 0 <= px < mw and 0 <= py < mh:
            cv2.circle(disp, (px, py), 5, (255, 0, 0), -1)

    # Title
    cv2.rectangle(disp, (0, 0), (mw, 20), (0, 0, 0), -1)
    cv2.putText(disp, f"MAP: {len(all_estimates)} estimates", (5, 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    _, jpg = cv2.imencode('.jpg', disp, [cv2.IMWRITE_JPEG_QUALITY, 80])
    with frame_lock:
        latest_map_jpeg = jpg.tobytes()


def render_bullseye(all_estimates, smart_est=None):
    # 5 GPS plot types side-by-side (or 3 when smart disabled)
    # all_estimates items: (est_lat, est_lon, pixel_dist, altitude, confidence)
    global latest_bullseye
    S = 350          # each plot is SxS
    BG = (20, 20, 20)
    SEP = 2          # separator width
    RING_CLR = (40, 40, 40)
    MARGIN = 35

    def heat_color(val):
        # 0..1 maps green -> red (BGR)
        v = max(0.0, min(1.0, val))
        return (0, int(255 * (1 - v)), int(255 * v))

    def make_panel():
        return np.full((S, S, 3), BG, dtype=np.uint8)

    def make_map_panel(mean_lat, mean_lon, range_m):
        # crop of the satellite map centred on (mean_lat, mean_lon), +/- range_m
        global _map_base
        if _map_base is None:
            map_path = config.MAP_FILE
            if os.path.exists(map_path):
                img = cv2.imread(map_path)
                if img is not None:
                    _map_base = cv2.resize(img, (600, int(img.shape[0] * 600 / img.shape[1])))
        if _map_base is None:
            return make_panel()

        mh, mw = _map_base.shape[:2]
        # GPS to pixel on the map
        dn = (config.REF_LAT - mean_lat) * 111320
        de = (mean_lon - config.REF_LON) * 111320 * math.cos(math.radians(config.REF_LAT))
        center_px = int(de / config.MAP_WIDTH_METERS * mw)
        center_py = int(dn / config.MAP_WIDTH_METERS * mw)
        # Crop radius in pixels: range_m maps to half the panel
        px_per_m = mw / config.MAP_WIDTH_METERS
        half_px = int(range_m * px_per_m)
        if half_px < 10:
            half_px = 50  # minimum crop size

        x1 = center_px - half_px
        y1 = center_py - half_px
        x2 = center_px + half_px
        y2 = center_py + half_px

        # Clamp to map bounds, pad with black if out of range
        pad_l = max(0, -x1)
        pad_t = max(0, -y1)
        pad_r = max(0, x2 - mw)
        pad_b = max(0, y2 - mh)
        cx1 = max(0, x1)
        cy1 = max(0, y1)
        cx2 = min(mw, x2)
        cy2 = min(mh, y2)

        if cx2 <= cx1 or cy2 <= cy1:
            return make_panel()

        crop = _map_base[cy1:cy2, cx1:cx2].copy()
        if pad_l or pad_t or pad_r or pad_b:
            crop = cv2.copyMakeBorder(crop, pad_t, pad_b, pad_l, pad_r,
                                       cv2.BORDER_CONSTANT, value=(0, 0, 0))

        panel = cv2.resize(crop, (S, S))
        # Semi-transparent dark overlay for readability
        overlay = np.full((S, S, 3), (0, 0, 0), dtype=np.uint8)
        cv2.addWeighted(panel, 0.6, overlay, 0.4, 0, panel)
        return panel

    def draw_rings(panel, cx, cy, rings_m, scale, usable):
        for r_m in rings_m:
            r_px = int(r_m * scale)
            if 5 < r_px < usable // 2:
                cv2.circle(panel, (cx, cy), r_px, RING_CLR, 1)
                cv2.putText(panel, f"{r_m}m", (cx + r_px + 2, cy - 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.25, (60, 60, 60), 1)

    def gps_to_meters(lat, lon, ref_lat, ref_lon):
        n = (lat - ref_lat) * 111320
        e = (lon - ref_lon) * 111320 * math.cos(math.radians(ref_lat))
        return e, n

    def auto_scale(pts_m, margin=MARGIN):
        dists = [math.sqrt(p[0]**2 + p[1]**2) for p in pts_m]
        p95 = sorted(dists)[int(len(dists) * 0.95)] if dists else 5.0
        max_range = max(p95 * 2, 3.0)
        usable = S - 2 * margin
        sc = usable / max_range
        return sc, usable, S // 2, S // 2

    # ── Empty state ──
    has_smart = smart_est is not None
    n_plots = 5 if has_smart else 3
    total_w = n_plots * S + (n_plots - 1) * SEP

    if not all_estimates:
        plot = np.full((S, total_w, 3), BG, dtype=np.uint8)
        cv2.putText(plot, "Waiting for detections...", (50, S // 2),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
        _, jpg = cv2.imencode('.jpg', plot, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with frame_lock:
            latest_bullseye = jpg.tobytes()
        return

    # ── Shared data ──
    lats = [e[0] for e in all_estimates]
    lons = [e[1] for e in all_estimates]
    mean_lat = sum(lats) / len(lats)
    mean_lon = sum(lons) / len(lons)

    pts_m = []   # (east_m, north_m, pixel_dist, altitude, confidence)
    for est_e in all_estimates:
        lat, lon = est_e[0], est_e[1]
        pdist = est_e[2]
        alt = est_e[3] if len(est_e) > 3 else 0
        conf = est_e[4] if len(est_e) > 4 else 0
        em, nm = gps_to_meters(lat, lon, mean_lat, mean_lon)
        pts_m.append((em, nm, pdist, alt, conf))

    dists_from_mean = [math.sqrt(p[0]**2 + p[1]**2) for p in pts_m]
    cep50 = sorted(dists_from_mean)[len(dists_from_mean) // 2] if dists_from_mean else 0
    max_spread = max(dists_from_mean) if dists_from_mean else 0
    scale, usable, cx, cy = auto_scale([(p[0], p[1]) for p in pts_m])

    panels = []

    # ── Compute map range for satellite background ──
    # Range in meters from center that the plot covers (matches auto_scale)
    _dists = [math.sqrt(p[0]**2 + p[1]**2) for p in pts_m]
    _p95 = sorted(_dists)[int(len(_dists) * 0.95)] if _dists else 5.0
    _map_range_m = max(_p95 * 1.1, 2.0)  # slightly beyond 95th percentile

    # ════════════════════════════════════════════════════════════
    # PLOT 1: GPS by Center Distance (pixel centrality)
    # ════════════════════════════════════════════════════════════
    p1 = make_map_panel(mean_lat, mean_lon, _map_range_m) if bullseye_map_bg else make_panel()
    cv2.putText(p1, f"BY CENTER DIST (N={len(all_estimates)})", (5, 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

    draw_rings(p1, cx, cy, [1, 2, 3, 5, 10], scale, usable)
    cv2.drawMarker(p1, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 12, 2)

    max_pd = max(p[2] for p in pts_m) if pts_m else 1
    for em, nm, pdist, alt, conf in pts_m:
        px = cx + int(em * scale)
        py = cy - int(nm * scale)
        if MARGIN < px < S - MARGIN and MARGIN < py < S - MARGIN:
            val = min(1.0, pdist / max(max_pd, 1))
            cv2.circle(p1, (px, py), 3, heat_color(val), -1)

    cv2.putText(p1, f"CEP50: {cep50:.1f}m  max: {max_spread:.1f}m", (5, S - 20),
               cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1)
    est = dummy_estimator.get_estimate()
    if est:
        cv2.putText(p1, f"Est: {est[0]:.7f}, {est[1]:.7f}", (5, S - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.30, (255, 0, 255), 1)
    panels.append(p1)

    # ════════════════════════════════════════════════════════════
    # PLOT 2: GPS by Altitude
    # ════════════════════════════════════════════════════════════
    p2 = make_map_panel(mean_lat, mean_lon, _map_range_m) if bullseye_map_bg else make_panel()
    alts = [p[3] for p in pts_m]
    min_alt = min(alts) if alts else 0
    max_alt = max(alts) if alts else 1
    alt_range = max(max_alt - min_alt, 0.1)

    cv2.putText(p2, f"BY ALTITUDE ({min_alt:.0f}-{max_alt:.0f}m)", (5, 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

    draw_rings(p2, cx, cy, [1, 2, 3, 5, 10], scale, usable)
    cv2.drawMarker(p2, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 12, 2)

    for em, nm, pdist, alt, conf in pts_m:
        px = cx + int(em * scale)
        py = cy - int(nm * scale)
        if MARGIN < px < S - MARGIN and MARGIN < py < S - MARGIN:
            val = (alt - min_alt) / alt_range
            cv2.circle(p2, (px, py), 3, heat_color(val), -1)

    # Color bar (vertical, right side)
    bar_x = S - 18
    bar_top, bar_bot = 30, S - 40
    for y in range(bar_top, bar_bot):
        val = (y - bar_top) / max(bar_bot - bar_top, 1)
        cv2.line(p2, (bar_x, y), (bar_x + 8, y), heat_color(val), 1)
    cv2.putText(p2, f"{min_alt:.0f}", (bar_x - 5, bar_top - 3),
               cv2.FONT_HERSHEY_SIMPLEX, 0.22, (120, 120, 120), 1)
    cv2.putText(p2, f"{max_alt:.0f}", (bar_x - 5, bar_bot + 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.22, (120, 120, 120), 1)

    cv2.putText(p2, f"CEP50: {cep50:.1f}m", (5, S - 5),
               cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1)
    panels.append(p2)

    # ════════════════════════════════════════════════════════════
    # PLOT 3: Error Convergence (line chart)
    # ════════════════════════════════════════════════════════════
    p3 = make_panel()
    cv2.putText(p3, "ERROR CONVERGENCE", (5, 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

    n_pts = len(pts_m)
    if n_pts >= 2:
        # Compute running estimates for each detection count
        running_mean_err = []
        running_weighted_err = []
        running_median_err = []

        for k in range(1, n_pts + 1):
            subset = pts_m[:k]
            # Running mean
            rm_e = sum(p[0] for p in subset) / k
            rm_n = sum(p[1] for p in subset) / k
            # Running weighted by 1/cdist^2
            total_w_val = 0
            w_e, w_n = 0, 0
            for p in subset:
                cdist = max(p[2], 1.0)
                w = 1.0 / (cdist * cdist)
                w_e += p[0] * w
                w_n += p[1] * w
                total_w_val += w
            if total_w_val > 0:
                w_e /= total_w_val
                w_n /= total_w_val
            # Running median
            sorted_e = sorted(p[0] for p in subset)
            sorted_n = sorted(p[1] for p in subset)
            med_e = sorted_e[len(sorted_e) // 2]
            med_n = sorted_n[len(sorted_n) // 2]

            # Error = distance from final mean (best estimate)
            final_mean_e = sum(p[0] for p in pts_m) / n_pts
            final_mean_n = sum(p[1] for p in pts_m) / n_pts
            running_mean_err.append(math.sqrt((rm_e - final_mean_e)**2 + (rm_n - final_mean_n)**2))
            running_weighted_err.append(math.sqrt((w_e - final_mean_e)**2 + (w_n - final_mean_n)**2))
            running_median_err.append(math.sqrt((med_e - final_mean_e)**2 + (med_n - final_mean_n)**2))

        # Chart area
        chart_l, chart_r = 45, S - 10
        chart_t, chart_b = 30, S - 25
        chart_w = chart_r - chart_l
        chart_h = chart_b - chart_t

        # Y axis max
        all_errs = running_mean_err + running_weighted_err + running_median_err
        y_max = max(all_errs) if all_errs else 1.0
        y_max = max(y_max, 0.5)

        # Grid lines
        for ym in [0.5, 1.0, 2.0, 5.0, 10.0]:
            if ym <= y_max:
                gy = chart_b - int((ym / y_max) * chart_h)
                cv2.line(p3, (chart_l, gy), (chart_r, gy), (40, 40, 40), 1)
                cv2.putText(p3, f"{ym:.1f}m", (2, gy + 4),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.22, (80, 80, 80), 1)

        # Axes
        cv2.line(p3, (chart_l, chart_t), (chart_l, chart_b), (60, 60, 60), 1)
        cv2.line(p3, (chart_l, chart_b), (chart_r, chart_b), (60, 60, 60), 1)

        def plot_line(errs, color):
            prev = None
            for i, err in enumerate(errs):
                x = chart_l + int((i / max(n_pts - 1, 1)) * chart_w)
                y = chart_b - int((min(err, y_max) / y_max) * chart_h)
                if prev is not None:
                    cv2.line(p3, prev, (x, y), color, 1)
                prev = (x, y)

        plot_line(running_mean_err, (0, 255, 0))       # green = mean
        plot_line(running_weighted_err, (255, 255, 0))  # cyan = weighted
        plot_line(running_median_err, (255, 0, 255))    # magenta = median

        # Legend
        ly = S - 8
        cv2.putText(p3, "mean", (chart_l, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 255, 0), 1)
        cv2.putText(p3, "weighted", (chart_l + 45, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 0), 1)
        cv2.putText(p3, "median", (chart_l + 110, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 0, 255), 1)

        # X axis label
        cv2.putText(p3, f"N={n_pts}", (chart_r - 30, chart_b + 12),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.25, (100, 100, 100), 1)
    else:
        cv2.putText(p3, "Need 2+ detections", (30, S // 2),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
    panels.append(p3)

    # ════════════════════════════════════════════════════════════
    # PLOT 4: Smart Bullseye (tightest 10) — only if smart enabled
    # ════════════════════════════════════════════════════════════
    if has_smart:
        p4 = make_panel()
        if smart_est.locked and smart_est.locked_cluster:
            cluster = smart_est.locked_cluster  # list of (lat, lon, pixel_dist, frame)
            cv2.putText(p4, "LOCKED", (5, 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 2)
            cv2.putText(p4, f"spread: {smart_est.locked_spread:.2f}m", (80, 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 0), 1)
            _med = smart_est.get_median()
            if _med:
                coord_text = f"{_med[0]:.7f}, {_med[1]:.7f}"
                cv2.rectangle(p4, (0, 20), (S, 50), (0, 0, 0), -1)
                cv2.putText(p4, coord_text, (5, 44),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)

            c_lats = [e[0] for e in cluster]
            c_lons = [e[1] for e in cluster]
            c_mean_lat = sum(c_lats) / len(c_lats)
            c_mean_lon = sum(c_lons) / len(c_lons)
            c_pts = []
            for c_e in cluster:
                em, nm = gps_to_meters(c_e[0], c_e[1], c_mean_lat, c_mean_lon)
                c_pts.append((em, nm, c_e[2]))

            c_dists = [math.sqrt(p[0]**2 + p[1]**2) for p in c_pts]
            c_max = max(c_dists) * 1.5 if c_dists else 1.0
            c_max = max(c_max, 0.5)
            c_usable = S - 2 * MARGIN
            c_scale = c_usable / (2 * c_max)
            c_cx, c_cy = S // 2, S // 2

            draw_rings(p4, c_cx, c_cy, [0.1, 0.2, 0.5, 1.0], c_scale, c_usable)
            cv2.drawMarker(p4, (c_cx, c_cy), (0, 255, 255), cv2.MARKER_CROSS, 15, 2)

            for i, (em, nm, pdist) in enumerate(c_pts):
                px = c_cx + int(em * c_scale)
                py = c_cy - int(nm * c_scale)
                cv2.circle(p4, (px, py), 5, (0, 255, 255), -1)
                cv2.circle(p4, (px, py), 5, (255, 255, 255), 1)
                # Number label
                cv2.putText(p4, str(i + 1), (px + 7, py + 3),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.25, (200, 200, 200), 1)
                # Radial line from center
                cv2.line(p4, (c_cx, c_cy), (px, py), (40, 40, 40), 1)

            # Draw median SMART star
            med = smart_est.get_median()
            if med:
                med_em, med_nm = gps_to_meters(med[0], med[1], c_mean_lat, c_mean_lon)
                star_px = c_cx + int(med_em * c_scale)
                star_py = c_cy - int(med_nm * c_scale)
                cv2.drawMarker(p4, (star_px, star_py), (255, 0, 255), cv2.MARKER_STAR, 18, 2)

                # Bottom info: prominent GPS coordinate + stats
                cv2.rectangle(p4, (0, S - 50), (S, S), (0, 0, 0), -1)  # black bar
                cv2.putText(p4, f"SMART: {med[0]:.7f}, {med[1]:.7f}",
                           (5, S - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)
                cv2.putText(p4, f"Total:{len(smart_est.all_estimates)}  Cluster:{med[2]}  Rejected:{len(smart_est.all_estimates)-med[2]}",
                           (5, S - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (180, 180, 180), 1)
                cv2.putText(p4, f"Spread:{smart_est.locked_spread:.2f}m  Med err:{smart_est.get_cep50():.2f}m",
                           (5, S - 1), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (180, 180, 180), 1)
        else:
            n_est = len(smart_est.all_estimates)
            cv2.putText(p4, "searching...", (5, 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 0, 255), 1)
            cv2.putText(p4, f"{n_est} detections", (5, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
            cv2.putText(p4, f"Need tightest 10 <{smart_est.max_spread}m", (5, 55),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.30, (100, 100, 100), 1)
        panels.append(p4)

    # ════════════════════════════════════════════════════════════
    # PLOT 5: 4m Bullseye — central detections (<200px from center)
    # ════════════════════════════════════════════════════════════
    if has_smart:
        p5 = make_panel()
        central = [(em, nm, pdist, alt, conf) for em, nm, pdist, alt, conf in pts_m if pdist < 200]
        central_10 = central[:10]
        n_central = len(central_10)

        cv2.putText(p5, f"CENTRAL ({n_central}/10, <200px)", (5, 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        if central_10:
            c5_scale, c5_usable, c5_cx, c5_cy = auto_scale(
                [(p[0], p[1]) for p in central_10])
            draw_rings(p5, c5_cx, c5_cy, [1, 2, 3, 5, 10], c5_scale, c5_usable)
            cv2.drawMarker(p5, (c5_cx, c5_cy), (0, 255, 0), cv2.MARKER_CROSS, 12, 2)

            c5_max_pd = max(p[2] for p in central_10) if central_10 else 1
            for em, nm, pdist, alt, conf in central_10:
                px = c5_cx + int(em * c5_scale)
                py = c5_cy - int(nm * c5_scale)
                if MARGIN < px < S - MARGIN and MARGIN < py < S - MARGIN:
                    val = min(1.0, pdist / max(c5_max_pd, 1))
                    cv2.circle(p5, (px, py), 3, heat_color(val), -1)

            c5_dists = [math.sqrt(p[0]**2 + p[1]**2) for p in central_10]
            c5_cep = sorted(c5_dists)[len(c5_dists) // 2] if c5_dists else 0
            cv2.putText(p5, f"CEP50: {c5_cep:.1f}m", (5, S - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1)
        else:
            cv2.putText(p5, "No central detections yet", (20, S // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)
        panels.append(p5)

    # ── Assemble all panels with separators ──
    separator = np.full((S, SEP, 3), (60, 60, 60), dtype=np.uint8)
    strips = []
    for i, panel in enumerate(panels):
        if i > 0:
            strips.append(separator)
        strips.append(panel)
    plot = np.hstack(strips)

    # ── SMART GPS banner at bottom of bullseye when locked ──
    if smart_est is not None and smart_est.locked:
        _smart_med = smart_est.get_median()
        if _smart_med:
            banner_h = 30
            total_w = plot.shape[1]
            gps_banner = np.full((banner_h, total_w, 3), (0, 80, 0), dtype=np.uint8)
            gps_text = f"SMART GPS:  {_smart_med[0]:.7f}, {_smart_med[1]:.7f}   ({_smart_med[2]} samples, spread {smart_est.locked_spread:.2f}m)"
            cv2.putText(gps_banner, gps_text, (10, banner_h - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 1, cv2.LINE_AA)
            plot = np.vstack([plot, gps_banner])

    _, jpg = cv2.imencode('.jpg', plot, [cv2.IMWRITE_JPEG_QUALITY, 80])
    with frame_lock:
        latest_bullseye = jpg.tobytes()


# GPS state, read-only from mavproxy
gps_data = {
    "lat": 0.0, "lon": 0.0, "alt": 0.0, "sats": 0, "fix": 0,
    "yaw": 0.0, "pitch": 0.0, "roll": 0.0, "mode": "---"
}

COPTER_MODES = {
    0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 9: "LAND", 16: "POSHOLD",
}


def mavlink_reader(mav):
    # background thread: just reads telemetry from mavproxy. Sends nothing.
    # FIXME: the bare except at the bottom of this loop swallows everything,
    # including KeyboardInterrupt-derived noise. Narrow it when there's time.
    while True:
        try:
            msg = mav.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            mtype = msg.get_type()

            if mtype == 'GLOBAL_POSITION_INT':
                with gps_lock:
                    gps_data["lat"] = msg.lat / 1e7
                    gps_data["lon"] = msg.lon / 1e7
                    gps_data["alt"] = msg.relative_alt / 1000.0

            elif mtype == 'GPS_RAW_INT':
                with gps_lock:
                    gps_data["sats"] = msg.satellites_visible
                    gps_data["fix"] = msg.fix_type

            elif mtype == 'HEARTBEAT':
                if msg.type != 6:  # skip GCS heartbeats (mavproxy)
                    with gps_lock:
                        gps_data["mode"] = COPTER_MODES.get(msg.custom_mode, f"MODE_{msg.custom_mode}")

            elif mtype == 'ATTITUDE':
                with gps_lock:
                    gps_data["yaw"] = msg.yaw * 57.2958  # rad to deg
                    gps_data["pitch"] = msg.pitch * 57.2958
                    gps_data["roll"] = msg.roll * 57.2958

        except Exception:
            time.sleep(0.1)


def draw_overlay(frame, last_det, raw_det=None):
    # Puts the detection box + GPS/FOV info onto the frame for the stream.
    # raw_det is the pre-filter detection (tuple with bbox + class name);
    # drawn in yellow when the class/conf filter rejected it.
    h, w = frame.shape[:2]
    display = frame.copy()

    # GPS snapshot (needed early for altitude-dependent overlays)
    with gps_lock:
        g = dict(gps_data)

    # ── Yellow marker for rejected (raw) detections ──
    # Only draw if raw_det exists AND is different from the filtered last_det
    # (i.e. the detection was rejected by class/conf filter).
    if raw_det is not None and raw_det[3] < 0.5:
        _is_same = (last_det is not None and last_det[3] < 2.0
                     and abs(raw_det[0] - last_det[0]) < 5
                     and abs(raw_det[1] - last_det[1]) < 5)
        if not _is_same:
            rcx, rcy = int(raw_det[0]), int(raw_det[1])
            r_conf = raw_det[2]
            r_cls = raw_det[4] if len(raw_det) > 4 else ''
            r_bw = int(raw_det[5]) if len(raw_det) > 5 else 60
            r_bh = int(raw_det[6]) if len(raw_det) > 6 else 60
            yellow = (0, 255, 255)
            # Yellow bounding box (rejected detection — informational)
            rx1, ry1 = max(0, rcx - r_bw // 2), max(0, rcy - r_bh // 2)
            rx2, ry2 = min(w, rcx + r_bw // 2), min(h, rcy + r_bh // 2)
            cv2.rectangle(display, (rx1, ry1), (rx2, ry2), yellow, 2)
            # Small crosshair at center
            cv2.line(display, (rcx - 6, rcy), (rcx + 6, rcy), yellow, 1)
            cv2.line(display, (rcx, rcy - 6), (rcx, rcy + 6), yellow, 1)
            # Class + confidence label with black background
            r_label = f"{r_cls} {r_conf:.2f}"
            (rtw, rth), _ = cv2.getTextSize(r_label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(display, (rx1, ry1 - 28), (rx1 + rtw + 6, ry1 - 2), (0, 0, 0), -1)
            cv2.putText(display, r_label, (rx1 + 3, ry1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, yellow, 2)

    # Draw last detection box (persists between frames)
    # Persistent detection box — redraw on EVERY frame (vision.py only draws on inference frames)
    if last_det is not None:
        det_cx, det_cy, conf, age = last_det
        if age < 2.0:
            alpha = max(0.3, 1.0 - age / 2.0)
            color = (0, int(255 * alpha), 0)
            bw = getattr(draw_overlay, '_last_bw', 80)
            bh = getattr(draw_overlay, '_last_bh', 80)
            x1 = max(0, det_cx - bw // 2)
            y1 = max(0, det_cy - bh // 2)
            x2 = min(w, det_cx + bw // 2)
            y2 = min(h, det_cy + bh // 2)
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 3)
            cls_name = getattr(draw_overlay, '_last_class', '')
            label = f"AI {conf:.2f} [{cls_name}]"
            # Black background rectangle behind label for readability
            (tw, th_txt), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(display, (x1, y1 - 28), (x1 + tw + 6, y1 - 2), (0, 0, 0), -1)
            cv2.putText(display, label, (x1 + 3, y1 - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # Centre crosshair (helps pilot align directly over target)
    cx, cy = w // 2, h // 2
    cross_len = 25
    # Black outline for visibility on any background
    cv2.line(display, (cx - cross_len, cy), (cx + cross_len, cy), (0, 0, 0), 4)
    cv2.line(display, (cx, cy - cross_len), (cx, cy + cross_len), (0, 0, 0), 4)
    # Cyan inner line
    cv2.line(display, (cx - cross_len, cy), (cx + cross_len, cy), (0, 200, 200), 2)
    cv2.line(display, (cx, cy - cross_len), (cx, cy + cross_len), (0, 200, 200), 2)

    # ── Pink line: center → detection (like video_test_compare.py) ──
    if last_det is not None:
        det_cx, det_cy, conf, age = last_det
        if age < 2.0:
            dcx_i, dcy_i = int(det_cx), int(det_cy)
            # Pink/magenta line from frame center to detection center
            cv2.line(display, (cx, cy), (dcx_i, dcy_i), (255, 0, 255), 2)
            # Pixel distance
            px_dist = math.sqrt((dcx_i - cx)**2 + (dcy_i - cy)**2)
            mid_x = (cx + dcx_i) // 2
            mid_y = (cy + dcy_i) // 2
            # Bigger, bolder pixel distance with dark background
            px_label = f"{px_dist:.0f}px"
            (tw, th), _ = cv2.getTextSize(px_label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            cv2.rectangle(display, (mid_x + 5, mid_y - 12 - th), (mid_x + 12 + tw, mid_y - 6), (0, 0, 0), -1)
            cv2.putText(display, px_label, (mid_x + 8, mid_y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)
            # Estimated real distance in meters (using FOV + altitude)
            alt_now = g["alt"] if g["alt"] > 0.5 else 0
            if alt_now > 0.5:
                fov_h_rad = math.radians(FOV["hfov_deg"])
                ground_w_now = 2 * alt_now * math.tan(fov_h_rad / 2)
                px_per_m = w / ground_w_now
                dist_m = px_dist / px_per_m
                # Bigger, bolder meters distance with dark background
                m_label = f"{dist_m:.1f}m"
                (tw2, th2), _ = cv2.getTextSize(m_label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
                cv2.rectangle(display, (mid_x + 5, mid_y + 8 - th2), (mid_x + 12 + tw2, mid_y + 16), (0, 0, 0), -1)
                cv2.putText(display, m_label, (mid_x + 8, mid_y + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

    # GPS overlay (bottom of frame)
    lat, lon = g["lat"], g["lon"]
    alt = g["alt"]
    sats = g["sats"]
    mode = g["mode"]

    if lat != 0.0 or lon != 0.0:
        gps_text = f"GPS: {lat:.7f}, {lon:.7f} | Alt: {alt:.1f}m | Sats: {sats}"
    else:
        gps_text = f"GPS: No Fix | Sats: {sats}"

    cv2.rectangle(display, (0, h - 25), (w, h), (0, 0, 0), -1)
    cv2.putText(display, gps_text, (5, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 170, 255), 1)

    # top bar: FPS + mode + attitude
    cv2.rectangle(display, (0, 0), (w, 30), (0, 0, 0), -1)
    cam_f = cam_fps_tracker.fps()
    vis_f = vis_fps_tracker.fps()
    str_f = stream_fps_tracker.fps()
    fps_text = f"CAM:{cam_f:.1f}  VIS:{vis_f:.1f}  STR:{str_f:.1f}"
    cv2.putText(display, fps_text, (5, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    yaw_deg = g["yaw"]
    pitch_deg = g["pitch"]
    roll_deg = g["roll"]
    att_text = f"Y:{yaw_deg:.0f} P:{pitch_deg:.1f} R:{roll_deg:.1f}"
    cv2.putText(display, att_text, (w - 220, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    cv2.putText(display, mode, (w - 80, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # Compass rose (top-right corner)
    compass_cx = w - 45
    compass_cy = 65
    compass_r = 25
    cv2.circle(display, (compass_cx, compass_cy), compass_r, (80, 80, 80), 1)

    # N/S/E/W labels (outline for readability)
    cv2.putText(display, "N", (compass_cx - 4, compass_cy - compass_r - 3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 3)
    cv2.putText(display, "N", (compass_cx - 4, compass_cy - compass_r - 3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)

    # Drone heading arrow (yaw = 0 means North, positive = clockwise)
    yaw_rad = math.radians(yaw_deg)
    arrow_x = int(compass_cx + compass_r * 0.8 * math.sin(yaw_rad))
    arrow_y = int(compass_cy - compass_r * 0.8 * math.cos(yaw_rad))
    cv2.arrowedLine(display, (compass_cx, compass_cy), (arrow_x, arrow_y),
                    (0, 255, 0), 2, tipLength=0.4)

    # "FRONT" label on the frame edge matching drone forward (outline for readability)
    cv2.putText(display, "FRONT", (w // 2 - 25, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 3)
    cv2.putText(display, "FRONT", (w // 2 - 25, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)

    # ── FOV / calibration info bar (second from bottom) ──
    alt_val = g["alt"]
    gw, gh = ground_coverage(alt_val) if alt_val > 0.5 else (0, 0)
    cal_w, cal_h = ground_coverage(1.0)  # at 1m for calibration reference

    fov_y = h - 50  # above the GPS bar
    cv2.rectangle(display, (0, fov_y), (w, fov_y + 25), (0, 0, 0), -1)

    if alt_val > 0.5:
        fov_text = (f"FOV:{FOV['hfov_deg']:.0f}deg | "
                    f"Ground:{gw:.1f}x{gh:.1f}m @{alt_val:.0f}m | "
                    f"Cal@1m:{cal_w*100:.0f}x{cal_h*100:.0f}cm")
        # Draw ground coverage dimensions on frame edges (subtle, not overlapping UI)
        dim_color = (140, 110, 0)  # muted amber
        # Width: short arrows + label at bottom-left area (above info bars) — outline
        w_label = f"<-- {gw:.1f}m -->"
        cv2.putText(display, w_label, (w // 2 - 50, h - 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
        cv2.putText(display, w_label, (w // 2 - 50, h - 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, dim_color, 1)
        # Height: label on left edge — outline
        h_label = f"{gh:.1f}m"
        cv2.putText(display, h_label, (w - 45, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
        cv2.putText(display, h_label, (w - 45, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, dim_color, 1)
    else:
        fov_text = (f"FOV:{FOV['hfov_deg']:.0f}deg  f={FOV['focal_mm']}mm  "
                    f"sens={FOV['sensor_w']}mm | Cal@1m:{cal_w*100:.0f}x{cal_h*100:.0f}cm")
    cv2.putText(display, fov_text, (5, fov_y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 140, 0), 1)

    # ── Scale bar: 1m reference at current altitude (like video_test_compare.py) ──
    alt_scale = g["alt"]
    if alt_scale > 0.5:
        fov_h_rad_s = math.radians(FOV["hfov_deg"])
        ground_w_s = 2 * alt_scale * math.tan(fov_h_rad_s / 2)
        scale_1m = int(w / ground_w_s)  # pixels per meter
        if scale_1m > 5:
            sx2 = w - 20
            sx1 = sx2 - scale_1m
            sy1 = h - 100  # above the info bars
            cv2.line(display, (sx1, sy1), (sx2, sy1), (255, 255, 255), 2)
            cv2.line(display, (sx1, sy1 - 5), (sx1, sy1 + 5), (255, 255, 255), 2)
            cv2.line(display, (sx2, sy1 - 5), (sx2, sy1 + 5), (255, 255, 255), 2)
            # Outline for scale bar label
            scale_label = f"1m ({alt_scale:.0f}m alt)"
            cv2.putText(display, scale_label, (sx1, sy1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 3)
            cv2.putText(display, scale_label, (sx1, sy1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

    # ── Pink dot on detection center ──
    if last_det is not None and last_det[3] < 1.0:  # (cx, cy, conf, age)
        det_cx = int(last_det[0])  # already pixel coords from line 609
        det_cy = int(last_det[1])
        cv2.circle(display, (det_cx, det_cy), 12, (255, 0, 255), -1)  # filled pink
        cv2.circle(display, (det_cx, det_cy), 12, (255, 255, 255), 2)  # white border

    # ── Estimated dummy position (if we have observations) ──
    est = dummy_estimator.get_estimate()
    est_y = h - 75  # above FOV bar
    cv2.rectangle(display, (0, est_y), (w, est_y + 25), (0, 0, 0), -1)

    cls = getattr(draw_overlay, '_last_class', '') or ''
    if est is not None:
        e_lat, e_lon, n_obs = est
        est_text = f"DUMMY EST: {e_lat:.7f}, {e_lon:.7f} ({n_obs} obs)"
        est_color = (255, 0, 255)  # pink/magenta
    elif lat == 0.0 and lon == 0.0:
        est_text = "DUMMY EST: NO GPS — cannot estimate"
        est_color = (180, 0, 255)  # pink-red
    else:
        est_text = "DUMMY EST: waiting for detection..."
        est_color = (100, 100, 100)  # grey
    cv2.putText(display, est_text, (5, est_y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, est_color, 1)
    # Class name + confidence — prominent cyan at end of estimate bar
    if cls:
        det_conf = last_det[2] if last_det is not None else 0
        cls_conf_text = f"[{cls} {det_conf:.2f}]"
        txt_w = cv2.getTextSize(est_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0]
        cv2.putText(display, cls_conf_text, (txt_w + 12, est_y + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)

    # ── Result banner (shown for 5 seconds after SMART lock or SURVEY complete) ──
    if _result_banner is not None and time.time() < _result_banner["until"]:
        banner_text = _result_banner["text"]
        banner_color = _result_banner["color"]
        # Large semi-transparent banner at top center
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(banner_text, font, 1.2, 3)
        bx = (w - tw) // 2 - 10
        by = 35
        overlay_banner = display.copy()
        cv2.rectangle(overlay_banner, (bx - 10, by - th - 10), (bx + tw + 10, by + 10), (0, 0, 0), -1)
        cv2.addWeighted(overlay_banner, 0.7, display, 0.3, 0, display)
        cv2.putText(display, banner_text, (bx, by), font, 1.2, (0, 0, 0), 6, cv2.LINE_AA)
        cv2.putText(display, banner_text, (bx, by), font, 1.2, banner_color, 3, cv2.LINE_AA)

    return display


# inference worker thread — detection runs at ~1.4fps laptop / ~4.8fps Pi,
# display thread is independent and stays at ~30fps.
def inference_worker(args_ref, csv_writer_ref, csv_file_ref):
    global _inference_frame, _inference_eyes, _snap_request_latest, _snap_request_best
    global _snap_gps_info, _snap_gps_info_best, _snap_jpeg_latest, _snap_jpeg_best, _snap_display_for_smart
    global _inference_det_count, _inference_saved_count
    global _best_center_dist, _best_detection_gps, _smart_result_saved, _survey_result_saved, _smart_image_counter
    global _result_banner

    _auto_clear_at = None    # timestamp when auto-clear should fire
    _auto_clear_done = False  # True after first auto-clear (only fires once)

    # last_det is set on the module so display thread can read it
    _mod = sys.modules.get('field_tools.passive_watch') or sys.modules.get('__main__')
    _mod._auto_clear_reset_requested = False  # P key sets this to restart cycle

    while True:
        # Grab latest frame
        with _inference_lock:
            frame = _inference_frame
        if frame is None:
            time.sleep(0.01)
            continue

        # Get current eyes reference (may change on model switch)
        with _inference_eyes_lock:
            eyes = _inference_eyes
        if eyes is None or not eyes.using_ai:
            time.sleep(0.05)
            continue

        # Read runtime conf + class filter
        with runtime_lock:
            current_conf = runtime_state["conf_threshold"]
            current_class_filter = runtime_state["class_filter"]

        # Run detection on a COPY so rejected classes don't leave boxes on frame.
        # If detection passes class filter, we swap frame←det_frame so the bbox
        # is on the EXACT frame it was detected on (for snapshots + overlays).
        det_frame = frame.copy()
        found, x, y, conf = eyes.detect_in_image(det_frame)
        vis_fps_tracker.tick()
        h, w = frame.shape[:2]

        # Store RAW detection (before any filter) for dim gray overlay
        if found:
            raw_cls = getattr(eyes, 'last_class_name', '') or ''
            raw_bw = eyes.last_bbox_w if eyes.last_bbox_w > 0 else 80
            raw_bh = eyes.last_bbox_h if eyes.last_bbox_h > 0 else 80
            _mod._last_raw_det = (int(x), int(y), conf, 0.0, raw_cls, raw_bw, raw_bh)
            _mod._last_raw_det_time = time.time()

        # Class filter: reject detection if class doesn't match
        class_rejected = False
        if found and conf >= current_conf:
            if current_class_filter and current_class_filter != "all" and hasattr(eyes, 'last_class_name'):
                det_cls = eyes.last_class_name.lower()
                allowed = [c.strip().lower() for c in current_class_filter.split(',')]
                if 'other' in allowed:
                    named = {'person', 'bird', 'dummy'}
                    if det_cls not in named:
                        pass  # accepted as "other"
                    elif det_cls not in allowed:
                        class_rejected = True
                elif det_cls not in allowed:
                    class_rejected = True

        if found and conf >= current_conf and not class_rejected:
            # Accepted — use det_frame (has green bbox from vision.py on exact frame)
            frame = det_frame
            _inference_det_count += 1
            cx, cy = int(x), int(y)
            now = time.time()

            # Update last_det (tuple = atomic assignment, safe for display thread to read)
            _mod._last_det = (cx, cy, conf, 0.0)
            _mod._last_det_time = now

            # Store bbox size + class for persistent overlay
            draw_overlay._last_bw = eyes.last_bbox_w if eyes.last_bbox_w > 0 else 80
            draw_overlay._last_bh = eyes.last_bbox_h if eyes.last_bbox_h > 0 else 80
            draw_overlay._last_class = getattr(eyes, 'last_class_name', '')

            # Estimate dummy GPS position
            est_result = None
            with gps_lock:
                d_lat, d_lon = gps_data["lat"], gps_data["lon"]
                d_alt = gps_data["alt"]
                d_yaw = gps_data["yaw"]
                d_pitch = gps_data["pitch"]
                d_roll = gps_data["roll"]
                d_sats = gps_data["sats"]
                d_mode = gps_data["mode"]
            if d_lat != 0.0 or d_lon != 0.0:
                norm_x = x / w if w > 0 else 0.5
                norm_y = y / h if h > 0 else 0.5
                est_result = dummy_estimator.add_observation(
                    d_lat, d_lon, d_alt, d_yaw, norm_x, norm_y,
                    pitch_deg=d_pitch, roll_deg=d_roll,
                    compensate_tilt=args_ref.compensate_tilt
                )

                if est_result:
                    est_lat, est_lon = est_result
                    pixel_dist = math.sqrt((cx - w/2)**2 + (cy - h/2)**2)
                    _all_gps_estimates.append((est_lat, est_lon, pixel_dist, d_alt, conf))

                    # Smart estimator — add() returns True the instant it locks
                    _smart_added = False
                    _just_locked = False
                    if smart_estimator and not smart_estimator.locked:
                        _just_locked = smart_estimator.add(est_lat, est_lon, pixel_dist, None)
                        _smart_added = True
                        _mod._smart_added_flag = True  # signal display thread

                    # Bullseye plot updated periodically by display thread (not here — saves ~50-100ms)

                    # SMART image save moved outside detection block (see below)

                    # ── Mode 2: Full Survey result ──
                    if len(_all_gps_estimates) >= SURVEY_TARGET and not _survey_result_saved:
                        _survey_result_saved = True
                        survey = compute_survey_analysis(_all_gps_estimates[:SURVEY_TARGET])
                        if survey:
                            sv_lat, sv_lon = survey["lat"], survey["lon"]
                            sv_stats = survey["stats_text"]
                            result_frame = draw_overlay(frame, _mod._last_det)
                            sv_fname = os.path.join(args_ref.save_dir, f"RESULT_SURVEY_{sv_lat:.7f}_{sv_lon:.7f}.jpg")
                            generate_result_image(result_frame, "SURVEY COMPLETE",
                                                  sv_lat, sv_lon, sv_stats, sv_fname,
                                                  title_color=(255, 255, 0))
                            _result_banner = {"text": "SURVEY COMPLETE", "color": (255, 255, 0),
                                              "until": time.time() + 5}
                            print(f"\n[RESULT] SURVEY: {sv_lat:.7f}, {sv_lon:.7f} (N={survey['n']}, CEP50={survey['cep50']:.2f}m)")
                            print(f"         Method: {survey['method']}  Saved: {sv_fname}\n")

            # ── SMART image save — runs every cycle, uses lock_id to detect new locks ──
            if smart_estimator and smart_estimator.locked and smart_estimator.lock_id > _smart_image_counter:
                med = smart_estimator.get_median()
                if med:
                    s_lat, s_lon = med[0], med[1]
                    s_spread = smart_estimator.locked_spread
                    s_cep = smart_estimator.get_cep50()
                    s_stats = f"Spread: {s_spread:.2f}m  CEP50: {s_cep:.2f}m  N={med[2]}"
                    result_frame = draw_overlay(frame, _mod._last_det)
                    rh, rw = result_frame.shape[:2]
                    est_y = rh - 75
                    cv2.rectangle(result_frame, (0, est_y), (rw, est_y + 25), (0, 0, 0), -1)
                    smart_text = f"SMART: {s_lat:.7f}, {s_lon:.7f} (spread {s_spread:.2f}m)"
                    cv2.putText(result_frame, smart_text, (5, est_y + 16),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                    smart_dir = args_ref.smart_dir or os.path.join(args_ref.save_dir, "smart_detections")
                    os.makedirs(smart_dir, exist_ok=True)
                    _smart_image_counter = smart_estimator.lock_id
                    s_fname = os.path.join(smart_dir, f"{_smart_image_counter:04d}_{s_lat:.7f}_{s_lon:.7f}.png")
                    generate_result_image(result_frame, "SMART COORDINATE",
                                          s_lat, s_lon, s_stats, s_fname,
                                          title_color=(0, 255, 0),
                                          gps_label="SMART")
                    _result_banner = {"text": "TARGET FOUND", "color": (0, 255, 0),
                                      "until": time.time() + 5}
                    print(f"\n{'='*60}")
                    print(f"  [SMART IMAGE SAVED] {s_fname}")
                    print(f"  Coordinate: {s_lat:.7f}, {s_lon:.7f}  Spread: {s_spread:.2f}m")
                    print(f"{'='*60}\n", flush=True)

                    # Auto-clear: fire ONCE after first lock, so second lock is independent
                    if args_ref.auto_clear > 0 and not _auto_clear_done:
                        _auto_clear_at = time.time() + args_ref.auto_clear

            # ── P key: reset auto-clear cycle ──
            if getattr(_mod, '_auto_clear_reset_requested', False):
                _mod._auto_clear_reset_requested = False
                _auto_clear_done = False
                _auto_clear_at = None

            # ── Auto-clear timer (fires once only) ──
            if _auto_clear_at is not None and time.time() >= _auto_clear_at:
                _auto_clear_at = None
                _auto_clear_done = True  # never fires again
                _all_gps_estimates.clear()
                dummy_estimator.reset()
                if smart_estimator:
                    smart_estimator.__init__(min_samples=smart_estimator.min_samples, max_spread=smart_estimator.max_spread)
                _g = globals()
                _g['_best_center_dist'] = 999.0
                _g['_best_detection_gps'] = None
                _g['_snap_request_best'] = False
                _g['_snap_request_latest'] = False
                _g['_snap_jpeg_best'] = None
                _g['_snap_jpeg_latest'] = None
                draw_overlay._last_class = ''
                _g['_last_det'] = None
                _g['latest_detection_jpeg'] = None
                _g['latest_best_jpeg'] = None
                _g['latest_bullseye'] = None
                _g['latest_smart_grid_jpeg'] = None
                _g['_smart_result_saved'] = False
                _g['_survey_result_saved'] = False
                stats["detections"] = 0
                stats["saved"] = 0
                print(f"\n  [AUTO-CLEAR] Reset after {args_ref.auto_clear}s. Collecting second lock...\n")

            # Flag snapshot requests for display thread
            # Build GPS info dicts here so display thread has everything it needs
            gps_info_snap = None
            if d_lat != 0.0 or d_lon != 0.0:
                _cls = getattr(draw_overlay, '_last_class', '') or '?'
                gps_info_snap = {"drone_lat": d_lat, "drone_lon": d_lon, "cls": _cls, "conf": conf}
                if est_result:
                    gps_info_snap["est_lat"] = est_lat
                    gps_info_snap["est_lon"] = est_lon
                else:
                    _cum_est = dummy_estimator.get_estimate()
                    if _cum_est:
                        gps_info_snap["est_lat"] = _cum_est[0]
                        gps_info_snap["est_lon"] = _cum_est[1]

            _snap_gps_info = gps_info_snap

            # Render snapshot on EXACT detection frame (has green bbox from vision.py)
            snap_display = draw_overlay(frame, _mod._last_det)
            _snap_jpeg_latest = _snapshot_overlay(snap_display, label="LATEST", thumb_w=728, gps_info=gps_info_snap)
            _snap_display_for_smart = snap_display  # for smart estimator frame capture
            _snap_request_latest = True

            # Check if this detection is closer to center (for Best Detection panel)
            center_dist = math.sqrt((cx - w/2)**2 + (cy - h/2)**2) / (w/2)
            if center_dist < _best_center_dist:
                _best_center_dist = center_dist
                _snap_gps_info_best = gps_info_snap
                _snap_jpeg_best = _snapshot_overlay(snap_display, label="BEST", thumb_w=1024, gps_info=gps_info_snap)
                _snap_request_best = True
                # Store GPS info for best detection
                _best_gps_entry = {
                    "center_dist": round(center_dist, 4),
                    "drone_lat": round(d_lat, 7),
                    "drone_lon": round(d_lon, 7),
                }
                if est_result:
                    _best_gps_entry["est_lat"] = round(est_lat, 7)
                    _best_gps_entry["est_lon"] = round(est_lon, 7)
                else:
                    _cum_est = dummy_estimator.get_estimate()
                    if _cum_est:
                        _best_gps_entry["est_lat"] = round(_cum_est[0], 7)
                        _best_gps_entry["est_lon"] = round(_cum_est[1], 7)
                _best_detection_gps = _best_gps_entry

            # Save snapshot (normal mode, skipped in smart mode)
            if not args_ref.no_save and not args_ref.smart_estimate:
                _inference_saved_count += 1
                save_frame = draw_overlay(frame, _mod._last_det)
                sh, sw = save_frame.shape[:2]

                with gps_lock:
                    lat, lon = gps_data["lat"], gps_data["lon"]
                    alt = gps_data["alt"]
                ts = datetime.now().strftime("%H:%M:%S")

                if args_ref.simple_names:
                    est_snap = dummy_estimator.get_estimate()
                    if est_snap:
                        fname = f"{_inference_saved_count:04d}_{est_snap[0]:.7f}_{est_snap[1]:.7f}.jpg"
                    elif lat != 0.0 or lon != 0.0:
                        fname = f"{_inference_saved_count:04d}_{lat:.7f}_{lon:.7f}.jpg"
                    else:
                        fname = f"{_inference_saved_count:04d}_nogps.jpg"
                else:
                    if lat != 0.0 or lon != 0.0:
                        fname = f"det_{_inference_saved_count:04d}_{conf:.2f}_{lat:.7f}_{lon:.7f}.jpg"
                    else:
                        fname = f"det_{_inference_saved_count:04d}_{conf:.2f}_nogps.jpg"

                stamp_lines = [f"{ts} conf:{conf:.2f}"]
                if lat != 0.0 or lon != 0.0:
                    stamp_lines.append(f"DRONE: {lat:.7f},{lon:.7f} @{alt:.0f}m")
                else:
                    stamp_lines.append("DRONE: NO GPS")
                est_snap = dummy_estimator.get_estimate()
                if est_snap:
                    stamp_lines.append(f"DUMMY: {est_snap[0]:.7f},{est_snap[1]:.7f} ({est_snap[2]}obs)")
                for i, line in enumerate(stamp_lines):
                    ty = sh // 3 + i * 20
                    cv2.rectangle(save_frame, (sw - 280, ty - 14), (sw, ty + 4), (0, 0, 0), -1)
                    cv2.putText(save_frame, line, (sw - 275, ty),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                _enqueue_save({"kind": "image", "path": os.path.join(args_ref.save_dir, fname), "frame": save_frame})

                # JSON sidecar
                if not args_ref.simple_names:
                    g_w, g_h = ground_coverage(d_alt) if d_alt > 0.5 else (0, 0)
                    meta = {
                        "timestamp": datetime.now().isoformat(),
                        "frame": _mod._frame_count,
                        "detection": {
                            "confidence": round(conf, 3),
                            "pixel_x": cx, "pixel_y": cy,
                            "bbox_centre": [cx, cy],
                        },
                        "drone": {
                            "lat": round(d_lat, 7), "lon": round(d_lon, 7),
                            "alt_m": round(d_alt, 1),
                            "yaw_deg": round(d_yaw, 1),
                            "pitch_deg": round(d_pitch, 1),
                            "roll_deg": round(d_roll, 1),
                            "sats": d_sats,
                            "mode": d_mode,
                        },
                        "fov": {
                            "focal_mm": FOV["focal_mm"],
                            "sensor_w_mm": FOV["sensor_w"],
                            "hfov_deg": round(FOV["hfov_deg"], 1),
                            "ground_w_m": round(g_w, 2) if d_alt > 0.5 else None,
                            "ground_h_m": round(g_h, 2) if d_alt > 0.5 else None,
                        },
                        "estimate": {
                            "lat": round(est_snap[0], 7) if est_snap else None,
                            "lon": round(est_snap[1], 7) if est_snap else None,
                            "n_observations": est_snap[2] if est_snap else 0,
                        },
                        "image": fname,
                    }
                    json_fname = fname.replace('.jpg', '.json')
                    _enqueue_save({"kind": "json", "path": os.path.join(args_ref.save_dir, json_fname), "data": meta})

                # CSV log
                if csv_writer_ref[0]:
                    est = dummy_estimator.get_estimate()
                    _enqueue_save({"kind": "csv", "writer": csv_writer_ref[0], "file": csv_file_ref[0], "row": [
                        datetime.now().isoformat(), _mod._frame_count, f"{conf:.3f}",
                        cx, cy, f"{d_lat:.7f}", f"{d_lon:.7f}", f"{d_alt:.1f}",
                        d_sats, f"{d_yaw:.0f}",
                        d_mode,
                        f"{est[0]:.7f}" if est else "",
                        f"{est[1]:.7f}" if est else "",
                        est[2] if est else 0,
                        fname
                    ]})


# main
def main():
    global latest_jpeg, latest_det_jpeg, smart_estimator, _all_gps_estimates, _best_center_dist, latest_detection_jpeg, latest_best_jpeg, _best_detection_gps, _smart_result_saved, _survey_result_saved, _result_banner
    global _inference_frame, _inference_eyes, _snap_request_latest, _snap_request_best, _snap_gps_info, _snap_gps_info_best, _snap_jpeg_latest, _snap_jpeg_best, _snap_display_for_smart

    if args.smart_estimate:
        smart_estimator = SmartEstimator(
            min_samples=args.smart_min,
            max_spread=args.smart_radius)
        print(f"[SMART] Enabled: {args.smart_min} central samples within {args.smart_radius}m")

    # Auto-detect IP
    pi_ip = "localhost"
    try:
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3)
        ip = result.stdout.strip().split()[0]
        if ip:
            pi_ip = ip
    except Exception:
        pass

    # Non-blocking terminal key check
    try:
        import msvcrt
        def _check_key():
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8', errors='ignore').lower()
            return None
    except ImportError:
        import select as _sel
        def _check_key():
            if _sel.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
            return None

    print("=" * 50)
    print("  SAR PASSIVE WATCH")
    print("  Camera + Detection + Stream + Snapshots")
    print("  ZERO commands sent. Safe to run anytime.")
    print("=" * 50)
    print(f"  Confidence: {args.conf}")
    print(f"  Max FPS:    {args.fps}")
    if not args.no_save:
        os.makedirs(args.save_dir, exist_ok=True)
        print(f"  Saving to:  {args.save_dir}/")
    else:
        print("  Saving:     OFF")
    print(f"  Keys:       C = Clear All (reset SMART + detections)")
    print()
    print(f"  Dashboard:  http://{pi_ip}:{args.port}/")
    print(f"  Stream:     http://{pi_ip}:{args.port}/stream")
    print(f"  Snapshot:   http://{pi_ip}:{args.port}/snapshot")
    print()

    # Fake mode: replay video + SRT telemetry
    fake_cap = None
    fake_telem = None
    fake_frame_idx = [0]
    fake_fps = 30
    _fake_start = [time.time()]

    if args.fake:
        print(f"[FAKE] Loading video: {args.fake_video}")
        fake_cap = cv2.VideoCapture(args.fake_video)
        if not fake_cap.isOpened():
            print(f"[FAKE] ERROR: Cannot open {args.fake_video}")
            return
        fake_fps = fake_cap.get(cv2.CAP_PROP_FPS) or 30
        total = int(fake_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        print(f"[FAKE] Video: {int(fake_cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(fake_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ {fake_fps}fps, {total} frames")
        print(f"[FAKE] Loading SRT: {args.fake_srt}")
        fake_telem = parse_srt(args.fake_srt)
        print(f"[FAKE] SRT loaded: {len(fake_telem)} entries")
        print("[FAKE] Mavproxy disabled — using SRT telemetry")
    else:
        # Connect to mavproxy (read-only) for GPS
        mav = None
        if not args.no_mavlink:
            try:
                from pymavlink import mavutil
                print("[MAV] Connecting to udpin:0.0.0.0:14550 (read-only)...")
                mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
                msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
                if msg:
                    print(f"[MAV] Connected! Reading telemetry (zero commands)")
                    t = threading.Thread(target=mavlink_reader, args=(mav,), daemon=True)
                    t.start()
                else:
                    print("[MAV] No heartbeat — running without GPS")
                    mav = None
            except Exception as e:
                print(f"[MAV] Could not connect: {e} — running without GPS")
                mav = None
        else:
            print("[MAV] Skipped (--no-mavlink)")

    # Start camera + AI
    # camera_source owns the camera handle, applies lens undistortion in get_frame(),
    # and provides undistort() for fake-mode frames.  Used ONLY for frame capture.
    # _inference_eyes is used for detect_in_image() and is swapped on model switch.
    # It is created with undistort=False because the display loop already undistorts
    # every frame before handing it to the inference thread (avoiding double-undistortion).
    if args.fake:
        camera_source = VisionSystem(camera_index=None, model_path=args.model, undistort=True)
        # Separate inference eyes — no undistortion (frames arrive pre-undistorted)
        eyes = VisionSystem(camera_index=None, model_path=args.model, undistort=False)
    else:
        camera_source = VisionSystem(camera_index=0, model_path=args.model, undistort=True)
        # Separate inference eyes — no undistortion (get_frame() already undistorts)
        eyes = VisionSystem(camera_index=None, model_path=args.model, undistort=False)
    if not eyes.using_ai:
        print("[WARN] AI model not loaded — stream only, no detection")

    # Set initial model ID from --model argument (try abspath match, then normpath, then basename)
    model_matched = False
    for m in MODEL_TABLE:
        if os.path.abspath(m["path"]) == os.path.abspath(args.model):
            runtime_state["active_model_id"] = m["id"]
            model_matched = True
            break
    if not model_matched:
        # Fallback: match on normalized path or basename
        norm_arg = os.path.normpath(args.model)
        base_arg = os.path.basename(args.model)
        for m in MODEL_TABLE:
            if os.path.normpath(m["path"]) == norm_arg or os.path.basename(m["path"]) == base_arg:
                runtime_state["active_model_id"] = m["id"]
                model_matched = True
                break
    if model_matched:
        print(f"[OK] Model matched: {MODEL_TABLE[runtime_state['active_model_id']]['name']} (id={runtime_state['active_model_id']})")
    else:
        print(f"[WARN] --model '{args.model}' not found in MODEL_TABLE, defaulting to id=0")
    print("[OK] Ready. Ctrl+C to stop.\n")

    # Start HTTP server
    if not args.no_stream:
        server = ThreadedServer(('0.0.0.0', args.port), Handler)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        print(f"[OK] Stream serving on port {args.port}\n")
    else:
        print("[OK] Stream server disabled (--no-stream)\n")

    # Render initial empty bullseye
    render_bullseye([], smart_estimator)

    # CSV log for detections
    csv_path = os.path.join(args.save_dir, "detection_log.csv") if (not args.no_save and not args.simple_names) else None
    csv_file = None
    csv_writer = None
    if csv_path:
        write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
        csv_file = open(csv_path, 'a', newline='')
        csv_writer = csv.writer(csv_file)
        if write_header:
            csv_writer.writerow(['timestamp', 'frame', 'confidence', 'px_x', 'px_y',
                                 'drone_lat', 'drone_lon', 'alt_m', 'sats', 'yaw', 'mode',
                                 'est_dummy_lat', 'est_dummy_lon', 'est_n_obs', 'filename'])

    # ── Shared state for display ↔ inference communication ──
    # Use module-level attributes so inference_worker can access them
    _mod = sys.modules.get('field_tools.passive_watch') or sys.modules.get('__main__')
    _mod._last_det = None       # (cx, cy, conf, age) — written by inference, read by display
    _mod._last_det_time = 0     # timestamp of last detection
    _mod._last_raw_det = None   # (cx, cy, conf, age, cls, bw, bh) — ALL detections (pre-filter)
    _mod._last_raw_det_time = 0
    _mod._frame_count = 0       # shared frame counter
    _mod._smart_added_flag = False  # signal from inference that smart estimator got new data

    # Set up inference eyes reference
    with _inference_eyes_lock:
        _inference_eyes = eyes

    # Mutable refs so inference thread can access csv objects
    csv_writer_ref = [csv_writer]
    csv_file_ref = [csv_file]

    # Start file I/O worker thread (keeps imwrite/json off inference thread)
    save_thread = threading.Thread(target=_save_worker, daemon=True)
    save_thread.start()

    # Start inference worker thread
    inf_thread = threading.Thread(
        target=inference_worker,
        args=(args, csv_writer_ref, csv_file_ref),
        daemon=True
    )
    inf_thread.start()
    print("[OK] Inference thread started (decoupled from stream)\n")

    # Display loop — runs at ~30fps, independent of inference speed
    frame_count = 0
    start_time = time.time()

    while True:
        loop_start = time.time()

        # grab a frame
        if args.fake:
            target_frame = int((time.time() - _fake_start[0]) * fake_fps) + 1
            while fake_frame_idx[0] < target_frame:
                ret, frame = fake_cap.read()
                if not ret:
                    fake_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    fake_frame_idx[0] = 0
                    _fake_start[0] = time.time()
                    break
                fake_frame_idx[0] += 1
            # Update GPS from SRT telemetry
            t = fake_telem.get(fake_frame_idx[0])
            if t:
                with gps_lock:
                    gps_data["lat"] = t["lat"]
                    gps_data["lon"] = t["lon"]
                    gps_data["alt"] = t["alt"]
                    gps_data["yaw"] = t["yaw"]
                    gps_data["pitch"] = t["pitch"]
                    gps_data["roll"] = t["roll"]
                    gps_data["sats"] = 12
                    gps_data["mode"] = "FAKE"
            if fake_frame_idx[0] % 100 == 1:
                with gps_lock:
                    print(f"  [FAKE] Frame {fake_frame_idx[0]} GPS:{gps_data['lat']:.7f},{gps_data['lon']:.7f} Alt:{gps_data['alt']:.0f}m Yaw:{gps_data['yaw']:.0f}")
        else:
            frame = camera_source.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

        if frame is None:
            continue

        # lens undistort (get_frame already does this in non-fake mode)
        if args.fake:
            frame = camera_source.undistort(frame)

        frame_count += 1
        _mod._frame_count = frame_count
        now = time.time()
        cam_fps_tracker.tick()

        # hand frame to inference thread
        with _inference_lock:
            _inference_frame = frame.copy()

        # ── Handle model switch request from browser (non-blocking) ──
        with runtime_lock:
            switch_req = runtime_state["model_switch_request"]
        if switch_req is not None and not getattr(_mod, '_model_switch_active', False):
            _mod._model_switch_active = True
            mid = switch_req
            m = MODEL_TABLE[mid]
            with runtime_lock:
                runtime_state["model_switch_request"] = None  # consumed

            def _do_model_switch(mid, m):
                # load + warmup then swap into the inference slot
                global _inference_eyes
                print(f"[MODEL] Loading {m['name']} ({m['path']}, backend={m['backend']}) in background...")
                t0 = time.time()
                try:
                    new_eyes = VisionSystem(
                        camera_index=None,
                        model_path=m['path'],
                        backend=m['backend'],
                        undistort=False,  # frames are already undistorted by display loop
                    )
                    if new_eyes.using_ai:
                        test_frame = np.zeros((640, 640, 3), dtype=np.uint8)
                        new_eyes.detect_in_image(test_frame)
                    # Swap
                    old_inf_eyes = None
                    with _inference_eyes_lock:
                        old_inf_eyes = _inference_eyes
                        _inference_eyes = new_eyes
                    if old_inf_eyes is not None and old_inf_eyes is not camera_source:
                        try:
                            old_inf_eyes.release()
                        except Exception:
                            pass
                    dt = time.time() - t0
                    print(f"[MODEL] Loaded: {m['name']} (backend={new_eyes.backend_name}) in {dt:.2f}s")
                    # Reset panels + filter + pending snapshot flags
                    _mod._best_center_dist = 999.0
                    _mod._best_detection_gps = None
                    _mod._snap_request_best = False
                    _mod._snap_request_latest = False
                    _mod._snap_jpeg_best = None
                    _mod._snap_jpeg_latest = None
                    draw_overlay._last_class = ''
                    _mod._last_det = None
                    with frame_lock:
                        _mod.latest_best_jpeg = None
                        _mod.latest_detection_jpeg = None
                    with runtime_lock:
                        runtime_state["active_model_id"] = mid
                        runtime_state["class_filter"] = "all"
                except Exception as e:
                    print(f"[MODEL] ERROR: {e}")
                    with runtime_lock:
                        runtime_state["model_switch_error"] = str(e)
                finally:
                    _mod._model_switch_active = False

            threading.Thread(target=_do_model_switch, args=(mid, m), daemon=True).start()

        # ── Read last_det from inference thread (tuple = atomic read) ──
        last_det = _mod._last_det
        last_det_time = _mod._last_det_time

        # Update detection age for fading overlay
        if last_det is not None:
            age = now - last_det_time
            last_det = (last_det[0], last_det[1], last_det[2], age)

        # ── Read raw detection (all detections, pre-filter) ──
        raw_det = _mod._last_raw_det
        raw_det_time = _mod._last_raw_det_time
        if raw_det is not None:
            raw_age = now - raw_det_time
            raw_det = (raw_det[0], raw_det[1], raw_det[2], raw_age,
                       raw_det[4], raw_det[5], raw_det[6])
            if raw_age > 0.5:
                raw_det = None  # fade after 0.5s

        # ── Update plots periodically (every ~60 frames ≈ 2s at 30fps) ──
        # When smart is active, bullseye/grid/map ALSO render from the
        # _smart_added_flag block below — but we still do periodic renders
        # so non-smart panels (Panel 1-3) update even between detections.
        if frame_count % max(1, int(fake_fps * 2 if args.fake else 60)) == 0:
            render_bullseye(_all_gps_estimates, smart_estimator)
            render_map(_all_gps_estimates, smart_estimator)

        # ── Draw overlay on every frame for smooth stream ──
        display = draw_overlay(frame, last_det, raw_det=raw_det)

        # ── Capture detection snapshots (pre-rendered by inference thread on exact frame) ──
        if _snap_request_latest:
            _snap_request_latest = False
            snap_bytes = _snap_jpeg_latest
            if snap_bytes:
                with frame_lock:
                    latest_detection_jpeg = snap_bytes
        if _snap_request_best:
            _snap_request_best = False
            snap_bytes = _snap_jpeg_best
            if snap_bytes:
                with frame_lock:
                    latest_best_jpeg = snap_bytes

        # ── Update smart estimator's last frame with exact detection frame ──
        if smart_estimator and _mod._smart_added_flag:
            _mod._smart_added_flag = False
            smart_frame = _snap_display_for_smart
            if smart_frame is not None:
                smart_estimator.update_last_frame(smart_frame)
                # Render BOTH grid AND bullseye together so they're always in sync
                render_smart_grid(smart_estimator)
                render_bullseye(_all_gps_estimates, smart_estimator)
                render_map(_all_gps_estimates, smart_estimator)

        # encode JPEG for the stream
        _, jpg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with frame_lock:
            latest_jpeg = jpg.tobytes()
        stream_fps_tracker.tick()

        # update stats
        det_count = _inference_det_count
        saved_count = _inference_saved_count
        det_pct = (det_count / frame_count * 100) if frame_count > 0 else 0
        with gps_lock:
            lat, lon = gps_data["lat"], gps_data["lon"]
            g_alt = gps_data["alt"]
            g_sats = gps_data["sats"]
            g_mode = gps_data["mode"]
        c_fps = cam_fps_tracker.fps()
        v_fps = vis_fps_tracker.fps()
        s_fps = stream_fps_tracker.fps()
        est = dummy_estimator.get_estimate()
        with frame_lock:
            stats.update({
                "frames": frame_count,
                "detections": det_count,
                "det_pct": f"{det_pct:.0f}",
                "cam_fps": f"{c_fps:.1f}",
                "vis_fps": f"{v_fps:.1f}",
                "stream_fps": f"{s_fps:.1f}",
                "saved": saved_count,
                "gps_lat": f"{lat:.7f}" if lat != 0 else "---",
                "gps_lon": f"{lon:.7f}" if lon != 0 else "---",
                "alt": f"{g_alt:.1f}" if g_alt != 0 else "---",
                "sats": g_sats,
                "flight_mode": g_mode,
                "est_lat": f"{est[0]:.7f}" if est else "---",
                "est_lon": f"{est[1]:.7f}" if est else "---",
                "est_obs": est[2] if est else 0,
            })

        # Terminal output every 50 frames
        if frame_count % 50 == 0:
            gps_str = f"GPS:{lat:.7f},{lon:.7f}" if lat != 0 else "GPS:---"
            print(f"  #{frame_count} CAM:{c_fps:.1f} VIS:{v_fps:.1f} STR:{s_fps:.1f} Det:{det_count} ({det_pct:.0f}%) Saved:{saved_count} {gps_str}")

        # ── Terminal key check ──
        try:
            key = _check_key()
            if key == 'c':
                # Clear All — same as browser button
                _all_gps_estimates.clear()
                dummy_estimator.reset()
                if smart_estimator:
                    smart_estimator.__init__(min_samples=smart_estimator.min_samples, max_spread=smart_estimator.max_spread)
                    if hasattr(smart_estimator, '_saved'):
                        smart_estimator._saved = False
                _g = globals()
                _g['_best_center_dist'] = 999.0
                _g['_best_detection_gps'] = None
                _g['_snap_request_best'] = False
                _g['_snap_request_latest'] = False
                _g['_snap_jpeg_best'] = None
                _g['_snap_jpeg_latest'] = None
                draw_overlay._last_class = ''
                _g['_last_det'] = None
                _g['latest_detection_jpeg'] = None
                _g['latest_best_jpeg'] = None
                _g['latest_bullseye'] = None
                _g['latest_smart_grid_jpeg'] = None
                _g['_smart_result_saved'] = False
                _g['_survey_result_saved'] = False
                stats["detections"] = 0
                stats["saved"] = 0
                print("\n  [CLEAR ALL] Reset SMART + detections. Ready for next detection.\n")
            elif key == 'p' and args.auto_clear > 0:
                # Re-trigger the two-image cycle: clear → detect → lock → wait → clear → detect → lock → done
                _all_gps_estimates.clear()
                dummy_estimator.reset()
                if smart_estimator:
                    smart_estimator.__init__(min_samples=smart_estimator.min_samples, max_spread=smart_estimator.max_spread)
                _g = globals()
                _g['_best_center_dist'] = 999.0
                _g['_best_detection_gps'] = None
                _g['_snap_request_best'] = False
                _g['_snap_request_latest'] = False
                _g['_snap_jpeg_best'] = None
                _g['_snap_jpeg_latest'] = None
                draw_overlay._last_class = ''
                _g['_last_det'] = None
                _g['latest_detection_jpeg'] = None
                _g['latest_best_jpeg'] = None
                _g['latest_bullseye'] = None
                _g['latest_smart_grid_jpeg'] = None
                _g['_smart_result_saved'] = False
                _g['_survey_result_saved'] = False
                stats["detections"] = 0
                stats["saved"] = 0
                # Signal inference thread to reset auto-clear cycle
                _mod._auto_clear_reset_requested = True
                print(f"\n  [P] Two-image cycle restarted. Detecting first target...\n")
        except Exception:
            pass

        # pace to ~30fps
        elapsed = time.time() - loop_start
        sleep_time = max(0, 0.033 - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)

    if csv_file:
        csv_file.close()
    camera_source.release()
    # Release inference eyes if it's a separate object from camera_source
    with _inference_eyes_lock:
        inf_eyes = _inference_eyes
    if inf_eyes is not None and inf_eyes is not camera_source:
        inf_eyes.release()
    server.shutdown()


if __name__ == "__main__":
    main()
