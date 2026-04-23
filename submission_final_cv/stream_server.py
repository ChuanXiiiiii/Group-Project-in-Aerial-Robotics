"""HTTP server for MJPEG stream + operator commands (port 8090 by default)."""

import json as _json
import time
import threading
import queue
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

import cv2

# FIXME: module-level state is not great for testing; fine for now since we only
# ever have one server instance per process.

# -- frame buffer ------------------------------------------------------------

_stream_frame = None
_stream_lock = threading.Lock()


def set_stream_frame(frame):
    """Publish the latest BGR frame to the stream."""
    global _stream_frame
    with _stream_lock:
        _stream_frame = frame


def get_stream_frame():
    with _stream_lock:
        return _stream_frame


# -- telemetry buffer --------------------------------------------------------

_telemetry: dict = {}
_telemetry_lock = threading.Lock()


def set_telemetry(**kwargs):
    """Update telemetry fields (partial updates are fine)."""
    with _telemetry_lock:
        _telemetry.update(kwargs)


def get_telemetry() -> dict:
    with _telemetry_lock:
        return dict(_telemetry)


# last 20 detection events
_det_log: deque = deque(maxlen=20)
_det_log_lock = threading.Lock()


def add_detection_event(event: dict):
    """Append one detection event (time, lat, lon, conf, decision)."""
    with _det_log_lock:
        _det_log.append(event)


def get_detection_log() -> list:
    with _det_log_lock:
        return list(_det_log)


# operator key codes
cmd_queue: queue.Queue = queue.Queue()

# stream config (set once in start_stream_server)
_cfg_stream_w: int = 320
_cfg_stream_h: int = 240
_cfg_stream_fps: int = 5
_cfg_stream_quality: int = 50


class StreamHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == '/stream':
            self._serve_stream()
        elif self.path == '/snapshot':
            self._serve_snapshot()
        elif self.path == '/status':
            self._serve_status()
        elif self.path == '/detlog':
            self._serve_detlog()
        elif self.path.startswith('/cmd?key='):
            self._serve_cmd()
        elif self.path == '/':
            self._serve_html()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_stream(self):
        """MJPEG multipart stream. Drops the client after 30s with no frames."""
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()

        idle_ticks = 0
        while True:
            with _stream_lock:
                f = _stream_frame
            if f is None:
                idle_ticks += 1
                if idle_ticks > 300:          # 300 * 0.1s = 30s
                    break
                time.sleep(0.1)
                continue
            idle_ticks = 0

            small = cv2.resize(f, (_cfg_stream_w, _cfg_stream_h))
            ret, jpeg = cv2.imencode(
                '.jpg', small,
                [cv2.IMWRITE_JPEG_QUALITY, _cfg_stream_quality],
            )
            if not ret or jpeg is None:
                time.sleep(0.1)
                continue

            data = jpeg.tobytes()
            try:
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(data)}\r\n\r\n'.encode())
                self.wfile.write(data)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                break

            time.sleep(1.0 / _cfg_stream_fps)

    def _serve_snapshot(self):
        with _stream_lock:
            f = _stream_frame
        if f is None:
            self.send_response(503)
            self.end_headers()
            return

        small = cv2.resize(f, (_cfg_stream_w, _cfg_stream_h))
        _, jpeg = cv2.imencode(
            '.jpg', small,
            [cv2.IMWRITE_JPEG_QUALITY, _cfg_stream_quality],
        )
        data = jpeg.tobytes()
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_status(self):
        data = get_telemetry()
        body = _json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_detlog(self):
        data = get_detection_log()
        body = _json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_cmd(self):
        """Parse /cmd?key=<char> and push the keycode onto cmd_queue."""
        parts = self.path.split('key=')
        if len(parts) < 2 or not parts[1]:
            self.send_response(400)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":false,"error":"missing key parameter"}')
            return

        key_char = parts[1][0].lower()
        valid = {'y', 'n', 'i', 'x', 'e', 'w', 's', 'm', 'r', 'k', 'b'}
        if key_char in valid:
            cmd_queue.put(ord(key_char))
            resp = f'{{"ok":true,"key":"{key_char}"}}'
        else:
            resp = f'{{"ok":false,"error":"invalid key: {key_char}"}}'

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(resp.encode())

    def _serve_html(self):
        html = _build_dashboard_html()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.end_headers()
        self.wfile.write(html.encode())

    def log_message(self, format, *args):
        # silence per-request stderr spam
        pass


def _build_dashboard_html() -> str:
    """Return the single-page operator dashboard as an HTML string."""
    return """<!DOCTYPE html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>
<title>SAR Drone Ground Station</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#0a0a0f;--panel:#151520;--border:#2a2a3a;
  --green:#2ecc71;--red:#e74c3c;--orange:#f39c12;--blue:#3498db;
  --cyan:#1abc9c;--purple:#9b59b6;--grey:#95a5a6;--white:#ecf0f1;
  --dim:#666;--dark:#222;
}
html,body{height:100%;overflow:hidden}
body{background:var(--bg);color:var(--white);font-family:'Courier New',monospace;
  display:flex;flex-direction:column}

/* top status bar */
.status-bar{
  display:flex;align-items:center;gap:2px;padding:4px 8px;
  background:#0d0d15;border-bottom:1px solid var(--border);
  flex-shrink:0;flex-wrap:wrap;min-height:36px;
}
.sb-item{
  display:flex;align-items:center;gap:4px;padding:2px 8px;
  font-size:12px;border-right:1px solid var(--border);white-space:nowrap;
}
.sb-item:last-child{border-right:none}
.sb-label{color:var(--dim);font-size:10px;text-transform:uppercase}
.sb-val{font-weight:bold;font-size:13px}
#sb-state{font-size:14px;padding:2px 12px;border-radius:3px;font-weight:bold;letter-spacing:1px}
.state-search{background:var(--blue);color:#fff}
.state-verify{background:var(--red);color:#fff;animation:pulse 1s infinite}
.state-manual{background:var(--orange);color:#000}
.state-centering,.state-descending{background:var(--purple);color:#fff}
.state-landing,.state-approach{background:var(--cyan);color:#000}
.state-done{background:#333;color:#888}
.state-hover{background:var(--green);color:#000}
.state-default{background:var(--dark);color:var(--green)}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.5}}

.gps-dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:3px}
.gps-good{background:var(--green)}
.gps-warn{background:var(--orange)}
.gps-bad{background:var(--red)}
.gps-none{background:#444}

.main-content{
  display:flex;flex:1;min-height:0;overflow:hidden;
}

.video-panel{
  flex:1;display:flex;flex-direction:column;min-width:0;position:relative;
}
.video-wrap{
  flex:1;display:flex;align-items:center;justify-content:center;
  background:#000;position:relative;overflow:hidden;
}
.video-wrap img{
  max-width:100%;max-height:100%;object-fit:contain;
  border:2px solid #1a3a1a;border-radius:4px;
}
#video-prompt{
  position:absolute;bottom:12px;left:50%;transform:translateX(-50%);
  padding:8px 24px;border-radius:6px;font-size:16px;font-weight:bold;
  display:none;z-index:10;text-align:center;
  backdrop-filter:blur(4px);
}
#video-prompt.verify-prompt{
  display:block;background:rgba(231,76,60,0.85);color:#fff;
  animation:pulse 1s infinite;
}
#video-prompt.side-prompt{
  display:block;background:rgba(26,188,156,0.85);color:#000;
}

.sidebar{
  width:280px;flex-shrink:0;display:flex;flex-direction:column;
  background:var(--panel);border-left:1px solid var(--border);
  overflow-y:auto;
}
.sidebar-section{padding:8px 10px;border-bottom:1px solid var(--border)}
.sidebar-section h3{
  font-size:11px;text-transform:uppercase;color:var(--dim);
  letter-spacing:1px;margin-bottom:6px;
}
.t-row{display:flex;justify-content:space-between;padding:2px 0;font-size:12px}
.t-label{color:var(--dim)}.t-val{color:var(--white);text-align:right}

.t-bar{height:5px;background:#1a1a2a;border-radius:3px;overflow:hidden;margin:4px 0}
.t-bar-fill{height:100%;background:var(--green);transition:width 0.3s}

.btn-group{
  display:flex;gap:6px;flex-wrap:wrap;justify-content:center;padding:4px;
}
.btn{
  padding:12px 16px;font-size:14px;font-weight:bold;border:none;border-radius:8px;
  cursor:pointer;font-family:'Courier New',monospace;min-width:60px;
  transition:transform 0.1s,box-shadow 0.1s;
  -webkit-tap-highlight-color:transparent;
  touch-action:manipulation;
}
.btn:active{transform:scale(0.95)}
.btn-y{background:var(--green);color:#000}
.btn-n{background:var(--red);color:#fff}
.btn-i{background:var(--orange);color:#000}
.btn-x{background:#7f8c8d;color:#fff}
.btn-m{background:var(--orange);color:#000;border:2px solid #e67e22}
.btn-rtl{background:#e67e22;color:#fff;border:2px solid #d35400}
.btn-kill{background:#c0392b;color:#fff;border:2px solid #a93226;font-size:13px}
.btn-dir{background:var(--blue);color:#fff;min-width:50px;padding:10px 14px}
.btn-dir:active{background:#2980b9}

.danger-group{
  display:flex;gap:6px;justify-content:center;padding:4px;
  border-top:1px solid #3a1a1a;margin-top:2px;
}

#cmd-feedback{
  text-align:center;font-size:11px;color:var(--green);min-height:16px;
  padding:2px 4px;
}

.det-log{max-height:140px;overflow-y:auto;font-size:11px}
.det-entry{
  padding:3px 6px;border-bottom:1px solid #1a1a2a;display:flex;
  justify-content:space-between;align-items:center;gap:4px;
}
.det-entry:nth-child(odd){background:rgba(255,255,255,0.02)}
.det-time{color:var(--dim);min-width:55px}
.det-conf{min-width:35px;text-align:right}
.det-gps{color:var(--dim);font-size:10px;flex:1;overflow:hidden;text-overflow:ellipsis}
.det-decision{
  font-weight:bold;min-width:18px;text-align:center;padding:1px 4px;
  border-radius:3px;font-size:10px;
}
.dec-Y{background:var(--green);color:#000}
.dec-N{background:var(--red);color:#fff}
.dec-I{background:var(--orange);color:#000}
.dec-X{background:#7f8c8d;color:#fff}
.dec-pending{background:var(--blue);color:#fff}

#mission-timer{
  font-size:16px;font-weight:bold;text-align:center;
  color:var(--green);padding:4px;font-variant-numeric:tabular-nums;
}

.geo-ok{color:var(--green)}
.geo-warn{color:var(--orange)}
.geo-bad{color:var(--red)}

@media(max-width:700px){
  .main-content{flex-direction:column}
  .sidebar{width:100%;max-height:45vh;flex-shrink:1;border-left:none;border-top:1px solid var(--border)}
  .video-panel{min-height:35vh}
  .status-bar{padding:2px 4px}
  .sb-item{padding:1px 4px;font-size:11px}
  .sb-val{font-size:11px}
  #sb-state{font-size:12px;padding:1px 8px}
  .btn{padding:14px 12px;font-size:16px;min-width:52px}
  .sidebar-section{padding:6px 8px}
}
@media(max-width:400px){
  .sb-item.hide-narrow{display:none}
  .btn{padding:12px 10px;font-size:14px;min-width:48px}
}
</style></head><body>

<!-- STATUS BAR -->
<div class="status-bar">
  <div class="sb-item"><span id="sb-state" class="state-default">INIT</span></div>
  <div class="sb-item"><span class="sb-label">ALT</span><span class="sb-val" id="sb-alt">--</span></div>
  <div class="sb-item"><span class="sb-label">SPD</span><span class="sb-val" id="sb-spd">--</span></div>
  <div class="sb-item hide-narrow">
    <span class="gps-dot gps-none" id="sb-gps-dot"></span>
    <span class="sb-val" id="sb-gps-txt">No GPS</span>
  </div>
  <div class="sb-item"><span class="sb-label">CONF</span><span class="sb-val" id="sb-conf">--</span></div>
  <div class="sb-item"><span class="sb-label">DET</span><span class="sb-val" id="sb-det">0</span></div>
  <div class="sb-item" style="margin-left:auto"><span id="mission-timer">00:00</span></div>
</div>

<!-- MAIN CONTENT -->
<div class="main-content">
  <div class="video-panel">
    <div class="video-wrap">
      <img src="/stream" alt="Video Stream" id="video-stream">
      <div id="video-prompt"></div>
    </div>
  </div>

  <div class="sidebar">
    <div class="sidebar-section">
      <h3>Operator Commands</h3>
      <div class="btn-group">
        <button class="btn btn-y" onclick="cmd('y')" title="Y - Confirm target">Y</button>
        <button class="btn btn-n" onclick="cmd('n')" title="N - Reject target">N</button>
        <button class="btn btn-i" onclick="cmd('i')" title="I - Item of interest">I</button>
        <button class="btn btn-x" onclick="cmd('x')" title="X - False positive">X</button>
        <button class="btn btn-m" onclick="cmd('m')" title="M - Manual override toggle">M</button>
      </div>
      <div id="cmd-feedback"></div>
    </div>

    <div class="sidebar-section" id="landing-section">
      <h3>Landing Direction</h3>
      <div class="btn-group">
        <button class="btn btn-dir" onclick="cmd('n')" title="Land North">N</button>
        <button class="btn btn-dir" onclick="cmd('e')" title="Land East">E</button>
        <button class="btn btn-dir" onclick="cmd('s')" title="Land South">S</button>
        <button class="btn btn-dir" onclick="cmd('w')" title="Land West">W</button>
      </div>
    </div>

    <div class="sidebar-section">
      <h3>Safety (R09)</h3>
      <div class="danger-group">
        <button class="btn btn-rtl" onclick="confirmCmd('r','Send RTL?')" title="R - Return to Launch">RTL</button>
        <button class="btn btn-kill" onclick="confirmCmd('k','DISARM motors?')" title="K - Kill/Disarm">KILL</button>
      </div>
    </div>

    <div class="sidebar-section">
      <h3>Telemetry</h3>
      <div class="t-row"><span class="t-label">Altitude</span><span class="t-val" id="t-alt">--</span></div>
      <div class="t-row"><span class="t-label">Speed</span><span class="t-val" id="t-spd">--</span></div>
      <div class="t-row"><span class="t-label">Latitude</span><span class="t-val" id="t-lat">--</span></div>
      <div class="t-row"><span class="t-label">Longitude</span><span class="t-val" id="t-lon">--</span></div>
      <div class="t-row"><span class="t-label">Heading</span><span class="t-val" id="t-hdg">--</span></div>
      <div class="t-row"><span class="t-label">Mode</span><span class="t-val" id="t-mode">--</span></div>
    </div>

    <div class="sidebar-section">
      <h3>GPS</h3>
      <div class="t-row">
        <span class="t-label">Fix</span>
        <span class="t-val" id="t-fix">--</span>
      </div>
      <div class="t-row">
        <span class="t-label">Satellites</span>
        <span class="t-val" id="t-sats">--</span>
      </div>
      <div class="t-row">
        <span class="t-label">Geofence</span>
        <span class="t-val geo-ok" id="t-geo">--</span>
      </div>
    </div>

    <div class="sidebar-section">
      <h3>Search Progress</h3>
      <div class="t-row"><span class="t-label">Waypoint</span><span class="t-val" id="t-wp">--</span></div>
      <div class="t-bar"><div class="t-bar-fill" id="t-wp-bar" style="width:0%"></div></div>
      <div class="t-row"><span class="t-label">Confidence</span><span class="t-val" id="t-conf">--</span></div>
      <div class="t-row"><span class="t-label">Detections</span><span class="t-val" id="t-det">0</span></div>
      <div class="t-row"><span class="t-label">Rejected</span><span class="t-val" id="t-rej">0</span></div>
      <div class="t-row"><span class="t-label">Interests</span><span class="t-val" id="t-ioi">0</span></div>
    </div>

    <div class="sidebar-section">
      <h3>Detection Log</h3>
      <div class="det-log" id="det-log">
        <div style="color:var(--dim);text-align:center;padding:8px">No detections yet</div>
      </div>
    </div>
  </div>
</div>

<script>
function cmd(k){
  fetch('/cmd?key='+k).then(function(r){return r.json()}).then(function(d){
    var fb=document.getElementById('cmd-feedback');
    fb.textContent='Sent: '+k.toUpperCase()+' ('+new Date().toLocaleTimeString()+')';
    setTimeout(function(){fb.textContent=''},3000);
  }).catch(function(e){
    document.getElementById('cmd-feedback').textContent='Error: '+e;
  });
}
function confirmCmd(k,msg){
  if(confirm(msg)){cmd(k)}
}

document.addEventListener('keydown',function(e){
  var k=e.key.toLowerCase();
  if(['y','n','i','x','e','w','s','m','b'].indexOf(k)>=0){
    e.preventDefault();cmd(k);
  }
  if(k==='r'){e.preventDefault();confirmCmd('r','Send RTL?')}
  if(k==='k'){e.preventDefault();confirmCmd('k','DISARM motors?')}
});

function fmtTime(secs){
  if(secs===undefined||secs===null)return'--:--';
  var m=Math.floor(secs/60);var s=Math.floor(secs%60);
  return (m<10?'0':'')+m+':'+(s<10?'0':'')+s;
}

var fixNames={0:'No GPS',1:'No Fix',2:'2D Fix',3:'3D Fix',4:'DGPS',5:'RTK Float',6:'RTK Fix'};
function gpsClass(fix){
  if(fix>=3)return'gps-good';
  if(fix===2)return'gps-warn';
  if(fix>=1)return'gps-bad';
  return'gps-none';
}

function stateClass(s){
  var sl=s.toLowerCase();
  if(sl==='verify')return'state-verify';
  if(sl==='manual')return'state-manual';
  if(sl.indexOf('search')>=0)return'state-search';
  if(sl==='centering')return'state-centering';
  if(sl==='descending')return'state-descending';
  if(sl==='landing'||sl==='approach')return'state-landing';
  if(sl==='done')return'state-done';
  if(sl.indexOf('hover')>=0)return'state-hover';
  return'state-default';
}

function poll(){
  fetch('/status').then(function(r){return r.json()}).then(function(d){
    var stEl=document.getElementById('sb-state');
    var st=d.state||'INIT';
    stEl.textContent=st;
    stEl.className=stateClass(st);

    document.getElementById('sb-alt').textContent=
      d.alt!==undefined?d.alt.toFixed(1)+'m':'--';
    document.getElementById('sb-spd').textContent=
      d.speed!==undefined?d.speed.toFixed(1)+'m/s':'--';

    var gpsDot=document.getElementById('sb-gps-dot');
    var gpsTxt=document.getElementById('sb-gps-txt');
    var fix=d.gps_fix!==undefined?d.gps_fix:0;
    var sats=d.gps_sats!==undefined?d.gps_sats:0;
    gpsDot.className='gps-dot '+gpsClass(fix);
    gpsTxt.textContent=(fixNames[fix]||fix)+' ('+sats+')';

    var sbConf=document.getElementById('sb-conf');
    if(d.conf!==undefined&&d.conf>0){
      sbConf.textContent=d.conf.toFixed(2);
      sbConf.style.color=d.conf>0.5?'var(--green)':d.conf>0.3?'var(--orange)':'var(--red)';
    }else{sbConf.textContent='--';sbConf.style.color='var(--dim)'}

    var dc=d.det_count!==undefined?d.det_count:0;
    document.getElementById('sb-det').textContent=dc;
    document.getElementById('t-det').textContent=dc;

    document.getElementById('mission-timer').textContent=fmtTime(d.mission_elapsed);

    document.getElementById('t-alt').textContent=
      d.alt!==undefined?d.alt.toFixed(1)+' m':'--';
    document.getElementById('t-spd').textContent=
      d.speed!==undefined?d.speed.toFixed(1)+' m/s':'--';
    document.getElementById('t-lat').textContent=
      d.lat!==undefined?d.lat.toFixed(6):'--';
    document.getElementById('t-lon').textContent=
      d.lon!==undefined?d.lon.toFixed(6):'--';
    document.getElementById('t-hdg').textContent=
      d.heading!==undefined?Math.round(d.heading)+'deg':'--';
    document.getElementById('t-mode').textContent=d.mode||'--';

    document.getElementById('t-fix').textContent=
      fixNames[fix]||'--';
    document.getElementById('t-sats').textContent=sats;

    var geoEl=document.getElementById('t-geo');
    var geoVal=d.geofence||'--';
    geoEl.textContent=geoVal;
    geoEl.className='t-val '+(geoVal==='OK'?'geo-ok':geoVal==='WARN'?'geo-warn':
      geoVal==='VIOLATION'?'geo-bad':'');

    var confEl=document.getElementById('t-conf');
    if(d.conf!==undefined){
      confEl.textContent=d.conf.toFixed(3);
      confEl.style.color=d.conf>0.5?'var(--green)':d.conf>0?'var(--orange)':'var(--dim)';
    }else{confEl.textContent='--'}

    document.getElementById('t-rej').textContent=d.rejected!==undefined?d.rejected:0;
    document.getElementById('t-ioi').textContent=d.interests!==undefined?d.interests:0;

    var wpEl=document.getElementById('t-wp');
    var wpBar=document.getElementById('t-wp-bar');
    if(d.wp_index!==undefined&&d.wp_total!==undefined&&d.wp_total>0){
      wpEl.textContent=d.wp_index+' / '+d.wp_total;
      wpBar.style.width=Math.round(d.wp_index/d.wp_total*100)+'%';
    }else{wpEl.textContent='--';wpBar.style.width='0%'}

    var pr=document.getElementById('video-prompt');
    if(d.selecting_side){
      pr.className='side-prompt';
      pr.textContent='SELECT LANDING SIDE: N / E / S / W';
      pr.style.display='block';
    }else if(d.waiting){
      var vt=d.verify_timeout!==undefined?' ('+Math.round(d.verify_timeout)+'s)':'';
      pr.className='verify-prompt';
      pr.textContent='VERIFY TARGET: Y / N / I'+vt;
      pr.style.display='block';
    }else{
      pr.style.display='none';pr.className='';
    }
  }).catch(function(){});
}

function pollLog(){
  fetch('/detlog').then(function(r){return r.json()}).then(function(arr){
    var el=document.getElementById('det-log');
    if(!arr||arr.length===0){
      el.innerHTML='<div style="color:var(--dim);text-align:center;padding:8px">No detections yet</div>';
      return;
    }
    var recent=arr.slice(-5).reverse();
    var html='';
    for(var i=0;i<recent.length;i++){
      var e=recent[i];
      var decClass='dec-'+(e.decision||'pending');
      var gps=e.lat!==undefined?e.lat.toFixed(5)+', '+e.lon.toFixed(5):'--';
      var conf=e.conf!==undefined?e.conf.toFixed(2):'--';
      html+='<div class="det-entry">';
      html+='<span class="det-time">'+(e.time||'--')+'</span>';
      html+='<span class="det-gps">'+gps+'</span>';
      html+='<span class="det-conf">'+conf+'</span>';
      html+='<span class="det-decision '+decClass+'">'+(e.decision||'?')+'</span>';
      html+='</div>';
    }
    el.innerHTML=html;
  }).catch(function(){});
}

setInterval(poll,800);poll();
setInterval(pollLog,2000);pollLog();
</script></body></html>"""


class _ThreadingHTTP(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def start_stream_server(port: int = 8090, host: str = '0.0.0.0',
                        stream_w: int = 320, stream_h: int = 240,
                        stream_fps: int = 5, stream_quality: int = 50,
                        max_port_retries: int = 5):
    """Start the MJPEG server on a daemon thread. Returns the server or None."""
    global _cfg_stream_w, _cfg_stream_h, _cfg_stream_fps, _cfg_stream_quality
    _cfg_stream_w = stream_w
    _cfg_stream_h = stream_h
    _cfg_stream_fps = stream_fps
    _cfg_stream_quality = stream_quality

    # bit hacky: shell out to `hostname -I` just to print a friendly URL on boot
    pi_ip = "localhost"
    try:
        import subprocess
        result = subprocess.run(
            ['hostname', '-I'],
            capture_output=True, text=True, timeout=3,
        )
        pi_ip = result.stdout.strip().split()[0]
    except Exception:
        pass

    last_error = None
    for attempt_port in range(port, port + max_port_retries):
        try:
            server = _ThreadingHTTP((host, attempt_port), StreamHandler)
            threading.Thread(target=server.serve_forever, daemon=True).start()

            if attempt_port != port:
                print(f"[STREAM] Port {port} in use, bound to {attempt_port} instead")
            print(f"[STREAM] Live feed: http://{pi_ip}:{attempt_port}/")
            print(f"[STREAM] Settings: {stream_w}x{stream_h} "
                  f"@ {stream_fps}fps, quality {stream_quality}%")
            return server
        except OSError as e:
            last_error = e
            continue

    print(f"[STREAM] ERROR: could not bind ports {port}-{port + max_port_retries - 1}: "
          f"{last_error}")
    return None
