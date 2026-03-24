"""
tools/mission_planner.py — Web-based mission route editor + SIL simulator.

Usage:
  python tools/mission_planner.py [--port 8089] [--lat LAT] [--lon LON]

Then open:  http://localhost:8089
  - Click canvas to add waypoints (enable with "+ Add WP" button)
  - Right-click a waypoint marker to delete it
  - Drag markers to reposition
  - Import / Export CSV (same format as mission_uploader.py)
  - Press Simulate to run sim_navigator SIL and see the path on the canvas
  - Save / Load named missions as JSON (includes obstacles)
  - Upload current mission directly to rover via MAVLink
  - Import any mission that GQC uploads to the rover (passive snooper on :14550)
"""

from __future__ import annotations

import csv
import glob as _glob
import io
import json
import math
import os
import socket as _socket
import sys
import threading as _threading
import time as _time
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_navigator import (  # noqa: E402
    DEFAULT_NAV, DEFAULT_PHYS, SimWaypoint, simulate, compute_pivot_wps,
)

DEFAULT_LAT  = 20.727715
DEFAULT_LON  = -103.566782
DEFAULT_PORT = 8089

# Unique per-launch token — forces browser cache miss on every server restart
_SERVER_VER  = str(int(_time.time()))

# ── Mission file storage ───────────────────────────────────────────────────────
MISSIONS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'missions'))
os.makedirs(MISSIONS_DIR, exist_ok=True)

def _save_mission_file(name: str, waypoints: list, obstacles: list):
    path = os.path.join(MISSIONS_DIR, f'{name}.json')
    with open(path, 'w') as f:
        json.dump({'waypoints': waypoints, 'obstacles': obstacles}, f, indent=2)
    return path

def _load_mission_file(name: str) -> dict:
    path = os.path.join(MISSIONS_DIR, f'{name}.json')
    with open(path) as f:
        return json.load(f)

def _list_mission_files() -> list[dict]:
    files = sorted(_glob.glob(os.path.join(MISSIONS_DIR, '*.json')),
                   key=os.path.getmtime, reverse=True)
    result = []
    for p in files:
        name = os.path.splitext(os.path.basename(p))[0]
        try:
            d = json.loads(open(p).read())
            wp_count  = len(d.get('waypoints', []))
            obs_count = len(d.get('obstacles', []))
        except Exception:
            wp_count = obs_count = 0
        result.append({'name': name, 'wp_count': wp_count, 'obs_count': obs_count,
                       'mtime': int(os.path.getmtime(p))})
    return result

# ── MAVLink passive snooper ────────────────────────────────────────────────────
_snooped:      dict             = {}   # last captured mission {waypoints, obstacles, rover}
_snooped_lock: _threading.Lock = _threading.Lock()
_rover_live:   dict             = {}   # sysid → {lat, lon, hdg, ts}
_rover_live_lock: _threading.Lock = _threading.Lock()

def _parse_fence_buf(fence_buf: list) -> list:
    """[(lat, lon, n), ...] → [[[lat,lon],...], ...] grouped by vertex count."""
    polys, i = [], 0
    while i < len(fence_buf):
        lat, lon, n = fence_buf[i]
        if n <= 0 or i + n > len(fence_buf):
            i += 1; continue
        polys.append([[fence_buf[i + j][0], fence_buf[i + j][1]] for j in range(n)])
        i += n
    return polys

def _start_snooper():
    """Background thread: passively capture missions re-broadcast by mavlink_bridge."""
    def _run():
        try:
            os.environ.setdefault('MAVLINK20', '1')
            from pymavlink.dialects.v20 import ardupilotmega as _mav_def
        except ImportError:
            print('[snoop] pymavlink not installed — network import disabled', flush=True)
            return
        try:
            sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
            sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
            sock.settimeout(1.0)
            sock.bind(('', 14550))
        except OSError as e:
            print(f'[snoop] Cannot bind :14550 ({e}) — network import disabled', flush=True)
            return
        print('[snoop] Listening on :14550 for mission broadcasts', flush=True)
        parsers: dict = {}
        buf:     dict = {}   # sysid → {count, nav, fence}
        while True:
            try:
                data, _ = sock.recvfrom(2048)
            except _socket.timeout:
                continue
            except Exception:
                break
            if len(data) < 8:
                continue
            sysid = data[5]   # MAVLink v2 sysid byte
            if sysid not in parsers:
                p = _mav_def.MAVLink(None)
                p.robust_parsing = True
                parsers[sysid] = p
            try:
                msgs = parsers[sysid].parse_buffer(data)
            except Exception:
                continue
            for msg in (msgs or []):
                t = msg.get_type()
                if t == 'GLOBAL_POSITION_INT':
                    hdg = msg.hdg  # cdeg, 65535=unknown
                    with _rover_live_lock:
                        _rover_live[sysid] = {
                            'lat': msg.lat / 1e7,
                            'lon': msg.lon / 1e7,
                            'hdg': hdg / 100.0 if hdg != 65535 else None,
                            'ts': _time.time(),
                        }
                elif t == 'MISSION_COUNT':
                    buf[sysid] = {'count': msg.count, 'nav': {}, 'fence': []}
                elif t == 'MISSION_ITEM_INT' and sysid in buf:
                    b = buf[sysid]
                    if msg.command == 5003:
                        b['fence'].append((msg.x / 1e7, msg.y / 1e7, int(msg.param1)))
                    elif msg.command == 16:
                        b['nav'][msg.seq] = {
                            'lat': msg.x / 1e7, 'lon': msg.y / 1e7,
                            'speed': float(msg.param4) if msg.param4 else 0.0,
                            'hold_secs': 0.0,
                        }
                    if b['count'] > 0 and msg.seq >= b['count'] - 1:
                        wps = [b['nav'][k] for k in sorted(b['nav'])]
                        obs = _parse_fence_buf(b['fence'])
                        with _snooped_lock:
                            _snooped.clear()
                            _snooped.update({'waypoints': wps, 'obstacles': obs,
                                             'rover': sysid, 'ts': _time.time()})
                        print(f'[snoop] RV{sysid}: {len(wps)} wps, '
                              f'{len(obs)} obstacles captured', flush=True)
    t = _threading.Thread(target=_run, daemon=True, name='mavlink-snooper')
    t.start()

# ── MAVLink upload to rover ────────────────────────────────────────────────────
def _mavlink_upload(waypoints: list, obstacles: list,
                    rover_ip: str, rover_port: int, rover_sysid: int) -> dict:
    """Upload mission to rover using GQC streaming protocol (no wait for REQUEST_INT)."""
    os.environ.setdefault('MAVLINK20', '1')
    try:
        from pymavlink.dialects.v20 import ardupilotmega as _mav_def
    except ImportError:
        return {'ok': False, 'message': 'pymavlink not installed'}
    try:
        # Build item list: fence vertices first, then nav waypoints
        items = []
        for poly in (obstacles or []):
            n = len(poly)
            for v in poly:
                items.append({'cmd': 5003, 'p1': float(n), 'p2': 0.0, 'p3': 0.0, 'p4': 0.0,
                              'lat': float(v[0]), 'lon': float(v[1]), 'z': 0.0})
        for wp in waypoints:
            items.append({'cmd': 16, 'p1': 0.0,
                          'p2': float(wp.get('acceptance_radius') or DEFAULT_NAV['default_acceptance_radius']),
                          'p3': 0.0, 'p4': 0.0,
                          'lat': float(wp['lat']), 'lon': float(wp['lon']),
                          'z': float(wp.get('speed') or 0.0)})
        if not items:
            return {'ok': False, 'message': 'No items to upload'}
    except Exception as e:
        return {'ok': False, 'message': f'Build error: {e}'}
    try:
        sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        mav  = _mav_def.MAVLink(None)
        mav.srcSystem    = 255   # GCS sysid
        mav.srcComponent = 0
        def _send(pkt):
            sock.sendto(pkt.pack(mav), (rover_ip, rover_port))
        _send(mav.mission_count_encode(rover_sysid, 0, len(items)))
        _time.sleep(0.15)   # GQC streaming delay
        for seq, item in enumerate(items):
            _send(mav.mission_item_int_encode(
                rover_sysid, 0, seq,
                6,             # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                item['cmd'],
                0, 1,          # current=0, autocontinue=1
                item['p1'], item['p2'], item['p3'], item['p4'],
                int(item['lat'] * 1e7), int(item['lon'] * 1e7), item['z'],
            ))
            _time.sleep(0.02)
        sock.close()
        n_wps = len(waypoints)
        n_obs = len(obstacles or [])
        return {'ok': True, 'message': f'Uploaded {n_wps} waypoints + {n_obs} obstacles to RV{rover_sysid}'}
    except Exception as e:
        return {'ok': False, 'message': str(e)}

# ── HTML page (single-file app) ───────────────────────────────────────────────
_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>AgriRover Mission Planner</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{display:flex;height:100vh;font-family:Arial,sans-serif;font-size:13px;background:#111}
#sidebar{width:310px;background:#1a1a2e;color:#ddd;display:flex;flex-direction:column;overflow:hidden;flex-shrink:0}
#cv-wrap{flex:1;position:relative;overflow:hidden;background:#0a0a14}
#canvas{display:block}
.toolbar{padding:6px;background:#0d0d1a;display:flex;flex-wrap:wrap;gap:4px;border-bottom:1px solid #333}
button{padding:5px 9px;border:none;border-radius:3px;cursor:pointer;font-size:12px;font-weight:bold}
.btn-blue{background:#0f3460;color:#fff}.btn-green{background:#1a7a3a;color:#fff}
.btn-orange{background:#a05010;color:#fff}.btn-red{background:#7a1515;color:#fff}
.btn-active{background:#27ae60!important;color:#fff!important}
button:hover{opacity:.85}
.section{padding:6px 8px;background:#12122a;border-bottom:1px solid #333;font-size:11px}
.section label{color:#aaa;margin-right:4px}
.section input{background:#0a0a1e;color:#eee;border:1px solid #444;padding:2px 4px;border-radius:2px}
#wp-list{flex:1;overflow-y:auto}
table{width:100%;border-collapse:collapse;font-size:11px}
th{background:#0d0d1a;padding:4px 5px;text-align:left;position:sticky;top:0;color:#aaa;border-bottom:1px solid #333}
td{padding:3px 5px;border-bottom:1px solid #222;vertical-align:middle}
td input{width:100%;background:transparent;color:#ddd;border:none;font-size:11px}
td input:focus{background:#0f0f2a;outline:1px solid #0f3460}
tr:hover td{background:#1e1e3a}
#stats{padding:8px;font-size:12px;line-height:1.7;border-top:2px solid #0f3460;display:none}
#stats .lbl{color:#888}
#status-bar{padding:4px 8px;font-size:11px;color:#27ae60;background:#0d0d1a;border-bottom:1px solid #222;min-height:22px}
#gen-panel{display:none;position:fixed;left:318px;top:45px;z-index:2000;background:#1a1a2e;border:1px solid #0f3460;border-radius:4px;padding:10px;width:240px;font-size:11px;color:#ddd;box-shadow:0 4px 14px rgba(0,0,0,.8)}
#gen-panel label{display:block;color:#aaa;margin-top:5px;margin-bottom:1px}
#gen-panel input,#gen-panel select{width:100%;background:#0a0a1e;color:#eee;border:1px solid #444;padding:2px 4px;border-radius:2px;font-size:11px}
#gen-panel .grow{display:flex;gap:4px;margin-top:8px}
#map-ctrl{position:absolute;right:10px;bottom:10px;display:flex;flex-direction:column;gap:4px;z-index:10}
#map-ctrl button{width:32px;height:32px;padding:0;font-size:16px;background:#1a1a2e;color:#ddd;border:1px solid #444;border-radius:4px}
#map-ctrl button:hover{background:#0f3460}
</style>
</head>
<body>
<div id="sidebar">
  <div class="toolbar">
    <span style="font-size:9px;color:#556;margin-right:auto;align-self:center">v=SERVER_VER</span>
    <button class="btn-blue" id="btn-add" onclick="toggleAddMode()">+ Add WP</button>
    <button id="btn-obs" class="btn-red" onclick="toggleObsMode()">&#9632; Obstacle</button>
    <button id="btn-meas" class="btn-blue" onclick="toggleMeasureMode()" title="Measure distance from cursor to planned route">&#8614; Dist</button>
    <button class="btn-green" onclick="runSimulate()">&#9654; Simulate</button>
    <button class="btn-orange" onclick="exportCSV()">&#8595; CSV</button>
    <button class="btn-blue" onclick="document.getElementById('file-import').click()">&#8593; Import</button>
    <button class="btn-red" onclick="clearAll()">&#10005; Clear</button>
    <button class="btn-orange" onclick="toggleGenPanel()">&#9881; Gen</button>
    <input type="file" id="file-import" accept=".csv" style="display:none" onchange="importCSV(event)">
  </div>
  <div style="padding:3px 6px;background:#0d0d1a;border-bottom:1px solid #333;display:flex;align-items:center;gap:4px;flex-wrap:wrap">
    <span style="color:#888;font-size:10px">All spd:</span>
    <input id="bulk-speed" type="number" value="1.0" min="0" max="1.5" step="0.1"
           style="width:42px;background:#0a1020;color:#eee;border:1px solid #446;padding:2px 3px;border-radius:2px;font-size:11px">
    <span style="color:#888;font-size:10px">m/s</span>
    <button onclick="applyBulkSpeed()" style="padding:2px 7px;font-size:11px;background:#0f3460;color:#fff;border:none;border-radius:2px;cursor:pointer">&#10003;</button>
    <span style="color:#888;font-size:10px;margin-left:6px">Algo:</span>
    <select id="algo-select" style="background:#0a1020;color:#eee;border:1px solid #446;padding:2px 3px;border-radius:2px;font-size:11px">
      <option value="">YAML default</option>
      <option value="stanley">Stanley</option>
      <option value="mpc">MPC</option>
      <option value="ttr">TTR</option>
    </select>
  </div>
  <div id="status-bar">Click "+ Add WP" then click the canvas to place waypoints.</div>
  <div class="section">
    <label>Start lat</label><input id="s-lat" size="11" value="START_LAT" onchange="redraw()">
    <label>lon</label><input id="s-lon" size="12" value="START_LON" onchange="redraw()">
    <label>hdg&deg;</label><input id="s-hdg" size="4" value="0">
  </div>
  <div class="section" style="background:#0d1a2e">
    <div style="display:flex;gap:3px;align-items:center;margin-bottom:3px">
      <label style="color:#7ab">&#128190;</label>
      <input id="m-name" size="13" placeholder="mission name" style="background:#0a1020;color:#eee;border:1px solid #446;padding:2px 4px;border-radius:2px;flex:1">
      <button class="btn-blue" style="padding:3px 7px;font-size:11px" onclick="saveMission()">Save</button>
    </div>
    <div style="display:flex;gap:3px;align-items:center">
      <select id="m-select" style="flex:1;background:#0a1020;color:#eee;border:1px solid #446;padding:2px;border-radius:2px;font-size:11px">
        <option value="">— saved missions —</option>
      </select>
      <button class="btn-blue" style="padding:3px 7px;font-size:11px" onclick="loadMission()">Load</button>
    </div>
  </div>
  <div class="section" style="background:#1a0d2e">
    <div style="display:flex;gap:3px;align-items:center;margin-bottom:3px">
      <label style="color:#b7a">&#128225;</label>
      <input id="r-ip-rv1" size="13" value="192.168.100.19" style="background:#0a1020;color:#eee;border:1px solid #446;padding:2px 4px;border-radius:2px;flex:1">
      <button class="btn-green" style="padding:3px 9px;font-size:11px" onclick="uploadRover(1)">&#9650; RV1</button>
    </div>
    <div style="display:flex;gap:3px;align-items:center;margin-bottom:3px">
      <label style="color:#b7a">&#128225;</label>
      <input id="r-ip-rv2" size="13" value="192.168.100.20" style="background:#0a1020;color:#eee;border:1px solid #446;padding:2px 4px;border-radius:2px;flex:1">
      <button class="btn-blue" style="padding:3px 9px;font-size:11px" onclick="uploadRover(2)">&#9650; RV2</button>
    </div>
    <div style="display:flex;gap:3px;align-items:center">
      <button id="btn-net" class="btn-orange" style="padding:3px 9px;font-size:11px" onclick="importSnooped()">&#8595; Net</button>
      <span id="snoop-status" style="font-size:10px;color:#888;margin-left:4px">no network mission</span>
    </div>
  </div>
  <div id="wp-list">
    <table>
      <thead><tr><th>#</th><th>Lat</th><th>Lon</th><th>Spd</th><th></th></tr></thead>
      <tbody id="wp-tbody"></tbody>
    </table>
  </div>
  <div id="stats"></div>
  <div id="playback" style="display:none;padding:6px 8px;border-top:1px solid #333;background:#0a0a14">
    <div style="display:flex;align-items:center;gap:4px;margin-bottom:4px">
      <button id="pb-play" onclick="pbToggle()" style="width:28px;padding:2px;font-size:14px;background:#0f3460;color:#fff;border:none;border-radius:3px;cursor:pointer">&#9654;</button>
      <input id="pb-slider" type="range" min="0" max="0" value="0" step="1"
             style="flex:1;height:14px;cursor:pointer" oninput="pbSeek(this.value)">
      <select id="pb-speed" style="width:48px;background:#0a1020;color:#eee;border:1px solid #446;padding:1px;border-radius:2px;font-size:10px"
              onchange="pbSetSpeed(this.value)">
        <option value="0.25">0.25x</option>
        <option value="0.5">0.5x</option>
        <option value="1" selected>1x</option>
        <option value="2">2x</option>
        <option value="4">4x</option>
      </select>
      <button onclick="pbDownload()" title="Download debug trace JSON"
              style="padding:2px 5px;font-size:10px;background:#1a3050;color:#8cf;border:1px solid #446;border-radius:3px;cursor:pointer">&#x2B73;</button>
    </div>
    <div id="pb-time" style="font-size:10px;color:#888;margin-bottom:3px">t=0.00s  frame 0/0</div>
    <div id="pb-info" style="font-size:11px;color:#ccc;line-height:1.5;font-family:monospace"></div>
  </div>
</div>
<!-- ── Mission generator panel ──────────────────────────────────────── -->
<div id="gen-panel">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:7px">
    <b style="color:#fff;font-size:12px">&#9881; Generate Mission</b>
    <button onclick="toggleGenPanel()" style="background:none;border:none;color:#aaa;font-size:15px;cursor:pointer;padding:0 3px">&#10005;</button>
  </div>
  <label>Pattern</label>
  <select id="gen-pattern" onchange="updateGenFields()">
    <option value="grid">Grid — serpentine rows (180&#176; U-turns)</option>
    <option value="zigzag">Zigzag — sharp diagonal turns</option>
    <option value="scatter">Scatter — random points</option>
    <option value="spiral">Spiral — outward arc</option>
  </select>

  <div id="gen-grid-fields">
    <label>Rows</label><input id="gen-rows" type="number" value="4" min="2" max="20">
    <label>Points per row</label><input id="gen-cols" type="number" value="5" min="2" max="20">
    <label>Row spacing (m)</label><input id="gen-row-sp" type="number" value="5" min="1">
    <label>Col spacing (m)</label><input id="gen-col-sp" type="number" value="5" min="1">
    <label>Heading &deg; (row direction)</label><input id="gen-hdg" type="number" value="0" min="0" max="359">
  </div>
  <div id="gen-zigzag-fields" style="display:none">
    <label>Legs</label><input id="gen-legs" type="number" value="6" min="2" max="30">
    <label>Leg length (m)</label><input id="gen-leg-len" type="number" value="10" min="2">
    <label>Lateral step per leg (m)</label><input id="gen-leg-off" type="number" value="5" min="1">
    <label>Heading &deg;</label><input id="gen-zhdg" type="number" value="0" min="0" max="359">
  </div>
  <div id="gen-scatter-fields" style="display:none">
    <label>Points</label><input id="gen-npts" type="number" value="10" min="3" max="50">
    <label>Radius (m)</label><input id="gen-radius" type="number" value="20" min="3">
    <label>Seed (0 = random)</label><input id="gen-seed" type="number" value="0" min="0">
  </div>
  <div id="gen-spiral-fields" style="display:none">
    <label>Turns</label><input id="gen-turns" type="number" value="3" min="1" max="10" step="0.5">
    <label>Arm spacing (m)</label><input id="gen-arm-sp" type="number" value="5" min="1">
    <label>Points per turn</label><input id="gen-pts-turn" type="number" value="10" min="4" max="32">
  </div>

  <label style="margin-top:7px">Speed (m/s, 0 = navigator default)</label>
  <input id="gen-speed" type="number" value="1.0" min="0" max="1.5" step="0.1">
  <label>Center on</label>
  <select id="gen-center">
    <option value="view">View centre</option>
    <option value="start">Start lat/lon</option>
  </select>
  <div style="margin-top:6px;display:flex;align-items:center;gap:5px">
    <input type="checkbox" id="gen-anchor" checked style="width:auto">
    <label style="margin:0;color:#eee">Anchor WP[0] to start pos</label>
  </div>
  <div class="grow">
    <button class="btn-red" style="flex:1;padding:4px" onclick="applyGenerate(false)">&#9654; Replace</button>
    <button class="btn-blue" style="flex:1;padding:4px" onclick="applyGenerate(true)">&#43; Append</button>
  </div>
</div>
<div id="cv-wrap">
  <canvas id="canvas"></canvas>
  <div id="map-ctrl">
    <button onclick="zoomIn()" title="Zoom in">+</button>
    <button onclick="fitAll()" title="Fit all">&#9635;</button>
    <button onclick="zoomOut()" title="Zoom out">&minus;</button>
  </div>
</div>

<script>
// ── Canvas & view state ───────────────────────────────────────────
const canvas = document.getElementById('canvas');
const ctx    = canvas.getContext('2d');
let viewLat  = START_LAT;
let viewLon  = START_LON;
let scale    = 50;   // pixels per metre

function resizeCanvas() {
  const wrap = document.getElementById('cv-wrap');
  canvas.width  = wrap.clientWidth;
  canvas.height = wrap.clientHeight;
  redraw();
}
window.addEventListener('resize', resizeCanvas);

function project(lat, lon) {
  const cosLat = Math.cos(viewLat * Math.PI / 180);
  return {
    x: canvas.width  / 2 + (lon - viewLon) * 111320 * cosLat * scale,
    y: canvas.height / 2 - (lat - viewLat) * 111320 * scale,
  };
}
function unproject(x, y) {
  const cosLat = Math.cos(viewLat * Math.PI / 180);
  return {
    lat: viewLat - (y - canvas.height / 2) / (111320 * scale),
    lon: viewLon + (x - canvas.width  / 2) / (111320 * cosLat * scale),
  };
}

// ── App state ─────────────────────────────────────────────────────
let waypoints  = [];
let addMode    = false;
let simResult  = null;
let obstacles  = [];
let obsMode    = false;
let obsCurPts  = [];
let liveRovers = {};   // sysid → {lat, lon, hdg, ts}

// Drag / pan tracking
let _drag    = null;   // {type:'wp',idx} | {type:'pan',sx,sy,sLat,sLon}
let _didDrag = false;

// ── Zoom / fit ────────────────────────────────────────────────────
function zoomIn()  { scale *= 1.4; redraw(); }
function zoomOut() { scale /= 1.4; redraw(); }

function fitAll() {
  const pts = waypoints.map(w => [w.lat, w.lon]);
  if (simResult) pts.push(...(simResult.path || []));
  obstacles.forEach(poly => pts.push(...poly));
  obsCurPts.forEach(p => pts.push(p));
  if (!pts.length) return;
  let minLat = Infinity, maxLat = -Infinity;
  let minLon = Infinity, maxLon = -Infinity;
  pts.forEach(([la, lo]) => {
    if (la < minLat) minLat = la; if (la > maxLat) maxLat = la;
    if (lo < minLon) minLon = lo; if (lo > maxLon) maxLon = lo;
  });
  viewLat = (minLat + maxLat) / 2;
  viewLon = (minLon + maxLon) / 2;
  const cosLat = Math.cos(viewLat * Math.PI / 180);
  const spanLat = Math.max((maxLat - minLat) * 111320, 2);
  const spanLon = Math.max((maxLon - minLon) * 111320 * cosLat, 2);
  scale = Math.min(canvas.width * 0.85 / spanLon, canvas.height * 0.85 / spanLat);
  redraw();
}

// ── Drawing ───────────────────────────────────────────────────────
function redraw() {
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#0a0a14';
  ctx.fillRect(0, 0, W, H);
  drawGrid(W, H);
  drawObstacles();
  drawObsPreview();
  drawRoute();
  drawSimPath();
  drawPivotMarkers();
  drawWaypoints();
  drawStartMarker();
  drawRover();
  drawLiveRovers();
  drawMeasureOverlay();
}

function drawGrid(W, H) {
  const sizes = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000];
  const gridM = sizes.find(g => g * scale >= 70) || 1000;
  const cosLat = Math.cos(viewLat * Math.PI / 180);
  const latStep = gridM / 111320;
  const lonStep = gridM / (111320 * cosLat);
  const tl = unproject(0, 0), br = unproject(W, H);
  ctx.save();
  ctx.strokeStyle = 'rgba(255,255,255,0.06)';
  ctx.lineWidth = 1;
  for (let la = Math.floor(br.lat / latStep) * latStep; la <= tl.lat + latStep; la += latStep) {
    const py = project(la, viewLon).y;
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(W, py); ctx.stroke();
  }
  for (let lo = Math.floor(tl.lon / lonStep) * lonStep; lo <= br.lon + lonStep; lo += lonStep) {
    const px = project(viewLat, lo).x;
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke();
  }
  ctx.restore();
  ctx.fillStyle = 'rgba(255,255,255,0.18)';
  ctx.font = '10px monospace';
  ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
  ctx.fillText('grid: ' + (gridM >= 1000 ? gridM/1000 + 'km' : gridM + 'm'), 8, H - 6);
}

function drawRoute() {
  if (waypoints.length < 2) return;
  ctx.save();
  ctx.strokeStyle = 'rgba(46,204,113,0.5)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  waypoints.forEach((wp, i) => {
    const p = project(wp.lat, wp.lon);
    i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawObstacles() {
  obstacles.forEach(poly => {
    if (poly.length < 3) return;
    ctx.beginPath();
    poly.forEach(([la, lo], i) => {
      const p = project(la, lo);
      i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y);
    });
    ctx.closePath();
    ctx.fillStyle   = 'rgba(231,76,60,0.20)';
    ctx.strokeStyle = 'rgba(231,76,60,0.70)';
    ctx.lineWidth   = 1.5;
    ctx.fill(); ctx.stroke();
  });
}

function drawObsPreview() {
  if (!obsCurPts.length) return;
  ctx.save();
  ctx.strokeStyle = 'rgba(231,76,60,0.7)';
  ctx.lineWidth   = 1.5;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  obsCurPts.forEach(([la, lo], i) => {
    const p = project(la, lo);
    i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y);
  });
  if (obsCurPts.length > 1) {
    const p0 = project(obsCurPts[0][0], obsCurPts[0][1]);
    ctx.lineTo(p0.x, p0.y);
  }
  ctx.stroke();
  ctx.restore();
  obsCurPts.forEach(([la, lo]) => {
    const p = project(la, lo);
    ctx.beginPath(); ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
    ctx.fillStyle = '#e74c3c'; ctx.fill();
  });
}

function drawSimPath() {
  if (!simResult) return;
  const path   = simResult.path    || [];
  const xteLog = simResult.xte_log || [];
  if (path.length < 2) return;
  const xteMax = Math.max(...xteLog, 0.001);
  ctx.lineWidth = 3;
  for (let i = 0; i + 1 < path.length; i++) {
    const t  = (xteLog[i] || 0) / xteMax;
    const r  = Math.round(255 * Math.min(t * 2, 1));
    const g  = Math.round(255 * Math.max(1 - t * 2 + 1, 0));
    const p1 = project(path[i][0],   path[i][1]);
    const p2 = project(path[i+1][0], path[i+1][1]);
    ctx.strokeStyle = `rgb(${r},${g},30)`;
    ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
  }
  // Rerouted path overlay
  const rerouted = simResult.rerouted_wps || [];
  if (obstacles.length && rerouted.length > 1) {
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.55)';
    ctx.lineWidth   = 1.5;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    rerouted.forEach(([la, lo], i) => {
      const p = project(la, lo);
      i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y);
    });
    ctx.stroke();
    ctx.restore();
  }
}

function drawPivotMarkers() {
  if (!simResult) return;
  (simResult.pivot_wps || []).forEach(pw => {
    const p       = project(pw.lat, pw.lon);
    const bypass  = pw.is_bypass;
    const r       = bypass ? 11 : 13;
    ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.fillStyle   = bypass ? 'rgba(180,80,200,0.5)' : 'rgba(230,126,34,0.45)';
    ctx.strokeStyle = 'rgba(255,255,255,0.7)';
    ctx.lineWidth   = 1.5;
    ctx.fill(); ctx.stroke();
    ctx.fillStyle    = '#fff';
    ctx.font         = `bold ${bypass ? 11 : 13}px sans-serif`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('\u21bb', p.x, p.y);
  });
}

function drawWaypoints() {
  // Draw waypoints 1..N first so waypoint 0 is always on top
  waypoints.forEach((wp, i) => {
    if (i === 0) return;
    const p = project(wp.lat, wp.lon);
    ctx.beginPath(); ctx.arc(p.x, p.y, 8, 0, Math.PI * 2);
    ctx.fillStyle   = 'rgba(26,122,58,0.6)';
    ctx.strokeStyle = 'rgba(255,255,255,0.7)';
    ctx.lineWidth   = 1;
    ctx.fill(); ctx.stroke();
    ctx.fillStyle    = '#fff';
    ctx.font         = 'bold 9px sans-serif';
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(i), p.x, p.y);
  });
  // Draw waypoint 0 last — larger, yellow, always on top
  if (waypoints.length > 0) {
    const wp0 = waypoints[0];
    const p = project(wp0.lat, wp0.lon);
    // Outer ring
    ctx.beginPath(); ctx.arc(p.x, p.y, 17, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(255,214,0,0.9)';
    ctx.lineWidth   = 2;
    ctx.stroke();
    // Filled circle
    ctx.beginPath(); ctx.arc(p.x, p.y, 13, 0, Math.PI * 2);
    ctx.fillStyle   = 'rgba(255,214,0,0.95)';
    ctx.strokeStyle = '#000';
    ctx.lineWidth   = 1.5;
    ctx.fill(); ctx.stroke();
    ctx.fillStyle    = '#000';
    ctx.font         = 'bold 10px sans-serif';
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('S', p.x, p.y);
  }
}

// Keep s-lat/s-lon in sync with WP[0] so sim always starts at the mission origin.
// Called after any load / import / generate that replaces the waypoint list.
function syncStartToWp0() {
  if (!waypoints.length) return;
  document.getElementById('s-lat').value = waypoints[0].lat.toFixed(7);
  document.getElementById('s-lon').value = waypoints[0].lon.toFixed(7);
}

function drawStartMarker() {
  const lat = parseFloat(document.getElementById('s-lat').value);
  const lon = parseFloat(document.getElementById('s-lon').value);
  if (isNaN(lat) || isNaN(lon)) return;
  const p = project(lat, lon);
  const r = 9;
  ctx.strokeStyle = '#FFD600';
  ctx.lineWidth   = 2.5;
  ctx.beginPath(); ctx.moveTo(p.x - r, p.y); ctx.lineTo(p.x + r, p.y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(p.x, p.y - r); ctx.lineTo(p.x, p.y + r); ctx.stroke();
}

// ── Hit testing ───────────────────────────────────────────────────
function hitWaypoint(x, y) {
  for (let i = waypoints.length - 1; i >= 0; i--) {
    const p = project(waypoints[i].lat, waypoints[i].lon);
    if (Math.hypot(p.x - x, p.y - y) <= 13) return i;
  }
  return -1;
}

function hitObstacleVertex(x, y) {
  for (let i = obstacles.length - 1; i >= 0; i--) {
    for (const [la, lo] of obstacles[i]) {
      const p = project(la, lo);
      if (Math.hypot(p.x - x, p.y - y) <= 7) return i;
    }
  }
  return -1;
}

// ── Measure mode ──────────────────────────────────────────────────
let measureMode = false;
let _measurePos = null;   // {x, y} in canvas pixels, or null
let _rafPending = false;

function toggleMeasureMode() {
  measureMode = !measureMode;
  if (measureMode) {
    addMode = false; obsMode = false;
    resetObsBtn();
    document.getElementById('btn-add').classList.remove('btn-active');
    canvas.style.cursor = 'crosshair';
  } else {
    _measurePos = null;
    canvas.style.cursor = 'default';
    redraw();
  }
  document.getElementById('btn-meas').classList.toggle('btn-active', measureMode);
  status(measureMode ? 'Hover over canvas to measure distance to planned route.' : 'Measure off.');
}

// Nearest point on the planned route (canvas pixel coords).
// Returns {px, py, distPx, segIdx} or null if no route.
function nearestOnRoute(mx, my) {
  if (waypoints.length < 2) return null;
  let best = null;
  for (let i = 0; i + 1 < waypoints.length; i++) {
    const p1 = project(waypoints[i].lat,   waypoints[i].lon);
    const p2 = project(waypoints[i+1].lat, waypoints[i+1].lon);
    const dx = p2.x - p1.x, dy = p2.y - p1.y;
    const len2 = dx*dx + dy*dy;
    let t = 0;
    if (len2 > 0) t = Math.max(0, Math.min(1, ((mx-p1.x)*dx + (my-p1.y)*dy) / len2));
    const cx = p1.x + t*dx, cy = p1.y + t*dy;
    const d  = Math.hypot(mx - cx, my - cy);
    if (!best || d < best.distPx) best = {px: cx, py: cy, distPx: d, segIdx: i};
  }
  return best;
}

function drawMeasureOverlay() {
  if (!measureMode || !_measurePos || waypoints.length < 2) return;
  const {x, y} = _measurePos;
  const near = nearestOnRoute(x, y);
  if (!near) return;

  const distM = near.distPx / scale;  // px ÷ (px/m) = metres

  // Dashed line from cursor to nearest route point
  ctx.save();
  ctx.setLineDash([5, 4]);
  ctx.strokeStyle = '#FFD600';
  ctx.lineWidth   = 1.5;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(near.px, near.py);
  ctx.stroke();
  ctx.restore();

  // Dot at cursor
  ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI*2);
  ctx.fillStyle = '#FFD600'; ctx.fill();

  // Dot at nearest route point
  ctx.beginPath(); ctx.arc(near.px, near.py, 5, 0, Math.PI*2);
  ctx.fillStyle = '#FFD600'; ctx.fill();

  // Distance label — flip side if too close to right edge
  const label  = distM < 1 ? `${(distM*100).toFixed(1)} cm` : `${distM.toFixed(3)} m`;
  const padX   = 7, padY = 4;
  ctx.font     = 'bold 12px monospace';
  const tw     = ctx.measureText(label).width;
  let lx = x + 10, ly = y - 10;
  if (lx + tw + padX*2 > canvas.width)  lx = x - tw - padX*2 - 10;
  if (ly - padY < 0)                     ly = y + 20;

  ctx.fillStyle = 'rgba(0,0,0,0.65)';
  ctx.fillRect(lx - padX, ly - 14 - padY, tw + padX*2, 20);
  ctx.fillStyle = '#FFD600';
  ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
  ctx.fillText(label, lx, ly - padY);
}

// ── Mouse interactions ─────────────────────────────────────────────
canvas.addEventListener('mousedown', e => {
  if (e.button !== 0) return;
  _didDrag = false;
  const {offsetX: x, offsetY: y} = e;
  if (!addMode && !obsMode) {
    const wi = hitWaypoint(x, y);
    if (wi >= 0) {
      _drag = {type: 'wp', idx: wi};
      return;
    }
    _drag = {type: 'pan', sx: x, sy: y, sLat: viewLat, sLon: viewLon};
    canvas.style.cursor = 'grabbing';
  }
});

canvas.addEventListener('mousemove', e => {
  const {offsetX: x, offsetY: y} = e;
  // Measure overlay — update position and schedule redraw
  if (measureMode && !_drag) {
    _measurePos = {x, y};
    if (!_rafPending) {
      _rafPending = true;
      requestAnimationFrame(() => { redraw(); _rafPending = false; });
    }
    return;
  }
  if (!_drag) return;
  _didDrag = true;
  if (_drag.type === 'pan') {
    const cosLat = Math.cos(viewLat * Math.PI / 180);
    viewLat = _drag.sLat + (_drag.sy - y) / (111320 * scale);
    viewLon = _drag.sLon - (_drag.sx - x) / (111320 * cosLat * scale);
    redraw();
  } else if (_drag.type === 'wp') {
    const ll = unproject(x, y);
    waypoints[_drag.idx].lat = ll.lat;
    waypoints[_drag.idx].lon = ll.lon;
    refreshTable();
    redraw();
  }
});

canvas.addEventListener('mouseup', () => {
  _drag = null;
  canvas.style.cursor = (addMode || obsMode) ? 'crosshair' : 'default';
});

canvas.addEventListener('mouseleave', () => {
  _drag = null;
  _measurePos = null;
  canvas.style.cursor = 'default';
  if (measureMode) redraw();
});

canvas.addEventListener('click', e => {
  if (_didDrag) return;
  const {offsetX: x, offsetY: y} = e;
  if (addMode) {
    const ll = unproject(x, y);
    addWp(ll.lat, ll.lon);
  } else if (obsMode) {
    const ll = unproject(x, y);
    obsAddVertex(ll.lat, ll.lon);
  }
});

canvas.addEventListener('contextmenu', e => {
  e.preventDefault();
  const {offsetX: x, offsetY: y} = e;
  const wi = hitWaypoint(x, y);
  if (wi >= 0) { removeWp(wi); return; }
  const oi = hitObstacleVertex(x, y);
  if (oi >= 0) { removeObstacle(oi); }
});

canvas.addEventListener('wheel', e => {
  e.preventDefault();
  const factor = e.deltaY < 0 ? 1.2 : 1 / 1.2;
  const ll = unproject(e.offsetX, e.offsetY);
  scale *= factor;
  const p = project(ll.lat, ll.lon);
  const cosLat = Math.cos(viewLat * Math.PI / 180);
  viewLat -= (p.y - e.offsetY) / (111320 * scale);
  viewLon += (p.x - e.offsetX) / (111320 * cosLat * scale);
  redraw();
}, {passive: false});

// ── Status bar ────────────────────────────────────────────────────
function status(msg, color='#27ae60') {
  const el = document.getElementById('status-bar');
  el.style.color = color;
  el.textContent = msg;
}

// ── Add mode ──────────────────────────────────────────────────────
function toggleAddMode() {
  addMode = !addMode;
  if (addMode) {
    obsMode = false; resetObsBtn();
    measureMode = false; _measurePos = null;
    document.getElementById('btn-meas').classList.remove('btn-active');
  }
  document.getElementById('btn-add').classList.toggle('btn-active', addMode);
  canvas.style.cursor = addMode ? 'crosshair' : 'default';
  status(addMode ? 'Click canvas to place waypoints. Right-click to delete.' : 'Add mode off.');
}

// ── Waypoints ─────────────────────────────────────────────────────
function addWp(lat, lon, speed=0, hold=0) {
  waypoints.push({lat, lon, speed, hold_secs: hold});
  refresh();
}

function removeWp(i) { waypoints.splice(i, 1); refresh(); }

function refresh() {
  refreshTable();
  redraw();
  status(`${waypoints.length} waypoint(s).`);
}

function refreshTable() {
  const tb = document.getElementById('wp-tbody');
  tb.innerHTML = '';
  waypoints.forEach((wp, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td style="color:#aaa">${i}</td>
      <td><input value="${wp.lat.toFixed(7)}" onchange="waypoints[${i}].lat=+this.value;redraw()"></td>
      <td><input value="${wp.lon.toFixed(7)}" onchange="waypoints[${i}].lon=+this.value;redraw()"></td>
      <td><input value="${wp.speed||0}" style="width:38px"
          onchange="waypoints[${i}].speed=+this.value||0"></td>
      <td><button onclick="removeWp(${i})"
          style="background:#7a1515;color:#fff;border:none;padding:1px 6px;border-radius:2px;cursor:pointer">&#10005;</button></td>`;
    tb.appendChild(tr);
  });
}

// ── Simulation ────────────────────────────────────────────────────
function clearSimOverlay() { simResult = null; redraw(); }

async function runSimulate() {
  if (waypoints.length < 2) { status('Need at least 2 waypoints.', '#e74c3c'); return; }
  if (obsMode) obsFinish();
  simResult = null;
  const statsEl = document.getElementById('stats');
  statsEl.style.display = 'block';
  statsEl.innerHTML = '<span style="color:#f39c12">&#9654; Simulating...</span>';
  status('Running simulation...', '#f39c12');

  const startLat = parseFloat(document.getElementById('s-lat').value) || waypoints[0].lat;
  const startLon = parseFloat(document.getElementById('s-lon').value) || waypoints[0].lon;
  const startHdg = parseFloat(document.getElementById('s-hdg').value) || 0;

  try {
    const algoSel = document.getElementById('algo-select').value;
    const navParams = algoSel ? {control_algorithm: algoSel} : {};
    const resp = await fetch('/simulate', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({waypoints, start: {lat: startLat, lon: startLon, heading: startHdg}, obstacles, nav_params: navParams}),
    });
    const d = await resp.json();
    simResult = d;
    fitAll();   // auto-center view on full path + waypoints after every simulation
    const dur = (d.total_steps / 25).toFixed(1);
    const algoName = algoSel || d.algorithm || 'yaml default';
    statsEl.innerHTML = `
      <div><span class="lbl">Algorithm:</span> <b style="color:#3498db">${algoName}</b></div>
      <div><span class="lbl">Complete:</span> ${d.complete ? '<b style="color:#2ecc71">\u2713 Yes</b>' : '<b style="color:#e74c3c">\u2717 Timeout</b>'}</div>
      <div><span class="lbl">Duration:</span> ${dur} s (${d.total_steps} steps)</div>
      <div><span class="lbl">WP reached:</span> ${(d.waypoints_reached||[]).length}/${waypoints.length}</div>
      <div><span class="lbl">XTE rms:</span> ${d.rms_xte.toFixed(3)} m</div>
      <div><span class="lbl">XTE max:</span> ${d.max_xte.toFixed(3)} m</div>
      <div><span class="lbl">Pivot WPs:</span> ${(d.pivot_wps||[]).length}</div>
      <div><span class="lbl">Obstacles:</span> ${obstacles.length}</div>
      <div style="font-size:10px;color:#666;margin-top:4px">Path: green=on-track \u2192 red=high XTE<br>Orange \u21bb = pivot &nbsp; Purple \u21bb = bypass pivot &nbsp; White dashed = reroute</div>`;
    status(`Done. XTE rms=${d.rms_xte.toFixed(3)}m max=${d.max_xte.toFixed(3)}m`);
    pbInit(d.debug_trace || []);
  } catch(e) {
    statsEl.innerHTML = `<span style="color:#e74c3c">Error: ${e}</span>`;
    status('Simulation failed.', '#e74c3c');
  }
}

// ── CSV export / import ───────────────────────────────────────────
function exportCSV() {
  fetch('/export_csv', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({waypoints}),
  }).then(r => r.blob()).then(blob => {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'mission.csv'; a.click();
  });
}

function importCSV(event) {
  const file = event.target.files[0]; if (!file) return;
  const reader = new FileReader();
  reader.onload = e => {
    const lines = e.target.result.trim().split('\n');
    const hdr = lines[0].toLowerCase().split(',');
    const li = hdr.indexOf('lat'), loi = hdr.indexOf('lon');
    const si = hdr.indexOf('speed'), hi = hdr.indexOf('hold_secs');
    if (li < 0 || loi < 0) { status('CSV must have lat,lon columns', '#e74c3c'); return; }
    waypoints = [];
    for (let i = 1; i < lines.length; i++) {
      const c = lines[i].split(','); if (c.length < 2) continue;
      waypoints.push({lat: +c[li], lon: +c[loi],
                      speed: si >= 0 ? +c[si] || 0 : 0,
                      hold_secs: hi >= 0 ? +c[hi] || 0 : 0});
    }
    refresh();
    if (waypoints.length) {
      syncStartToWp0();
      viewLat = waypoints[0].lat; viewLon = waypoints[0].lon; fitAll();
    }
    event.target.value = '';
  };
  reader.readAsText(file);
}

// ── Obstacles ─────────────────────────────────────────────────────
function resetObsBtn() {
  document.getElementById('btn-obs').textContent = '\u25a0 Obstacle';
  document.getElementById('btn-obs').style.background = '';
}

function toggleObsMode() {
  if (obsMode) { obsFinish(); return; }
  obsMode = true; addMode = false;
  measureMode = false; _measurePos = null;
  document.getElementById('btn-add').classList.remove('btn-active');
  document.getElementById('btn-meas').classList.remove('btn-active');
  document.getElementById('btn-obs').textContent = '\u2713 Done';
  document.getElementById('btn-obs').style.background = '#8b0000';
  canvas.style.cursor = 'crosshair';
  status('Click canvas to draw obstacle polygon. Click "Done" to close it.', '#e74c3c');
}

function obsAddVertex(lat, lon) {
  obsCurPts.push([lat, lon]);
  redraw();
  status(`Obstacle: ${obsCurPts.length} vertex(es). Click "Done" to close (min 3).`, '#e74c3c');
}

function obsFinish() {
  obsMode = false;
  resetObsBtn();
  canvas.style.cursor = 'default';
  if (obsCurPts.length < 3) {
    status('Need at least 3 vertices — obstacle discarded.', '#e74c3c');
    obsCurPts = []; redraw(); return;
  }
  obstacles.push([...obsCurPts]);
  obsCurPts = [];
  redraw();
  status(`Obstacle ${obstacles.length - 1} added. Total: ${obstacles.length}.`);
}

function removeObstacle(idx) {
  obstacles.splice(idx, 1);
  redraw();
  status(`Obstacle removed. Total: ${obstacles.length}.`);
}

function clearObstacles() {
  obstacles = []; obsCurPts = []; redraw();
}

// ── Clear all ─────────────────────────────────────────────────────
function clearAll() {
  waypoints = []; simResult = null;
  clearObstacles();
  refreshTable();
  document.getElementById('stats').style.display = 'none';
  redraw();
  status('Cleared.');
}

// ── Mission file save / load ──────────────────────────────────────
async function saveMission() {
  const name = document.getElementById('m-name').value.trim();
  if (!name) { status('Enter a mission name first.', '#e74c3c'); return; }
  await fetch('/save_mission', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({name, waypoints, obstacles}),
  });
  status(`Saved "${name}".`);
  refreshMissionList();
}

async function loadMission() {
  const sel = document.getElementById('m-select');
  const name = sel.value;
  if (!name) { status('Select a mission first.', '#e74c3c'); return; }
  const resp = await fetch(`/load_mission?name=${encodeURIComponent(name)}`);
  const d = await resp.json();
  waypoints = d.waypoints || [];
  obstacles = []; obsCurPts = [];
  (d.obstacles || []).forEach(poly => obstacles.push(poly));
  refresh();
  if (waypoints.length) {
    syncStartToWp0();
    viewLat = waypoints[0].lat; viewLon = waypoints[0].lon; fitAll();
  }
  status(`Loaded "${name}" — ${waypoints.length} wps, ${obstacles.length} obstacles.`);
}

async function refreshMissionList() {
  const resp = await fetch('/missions');
  const list = await resp.json();
  const sel = document.getElementById('m-select');
  const cur = sel.value;
  sel.innerHTML = '<option value="">— saved missions —</option>' +
    list.map(m => `<option value="${m.name}"${m.name === cur ? ' selected' : ''}>${m.name} (${m.wp_count}wp${m.obs_count ? ' ' + m.obs_count + 'obs' : ''})</option>`).join('');
}
refreshMissionList();

// ── Rover upload / network import ─────────────────────────────────
async function uploadRover(roverId) {
  // Auto-load from dropdown if canvas is empty and a mission is selected
  if (!waypoints.length) {
    const sel = document.getElementById('m-select');
    if (sel.value) { await loadMission(); }
    if (!waypoints.length) { status('No waypoints to upload.', '#e74c3c'); return; }
  }
  const rover_ip = document.getElementById(`r-ip-rv${roverId}`).value.trim();
  if (!rover_ip) { status(`Enter RV${roverId} IP.`, '#e74c3c'); return; }
  status(`Uploading to RV${roverId}...`, '#f39c12');
  if (obsMode) obsFinish();
  try {
    const resp = await fetch('/upload_rover', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({waypoints, obstacles, rover_ip, rover_port: 14550, rover_sysid: roverId}),
    });
    const d = await resp.json();
    status(d.message, d.ok ? '#27ae60' : '#e74c3c');
  } catch(e) {
    status(`Upload failed: ${e.message}`, '#e74c3c');
  }
}

async function pollSnooped() {
  try {
    const resp = await fetch('/snooped_mission');
    const d = await resp.json();
    const el = document.getElementById('snoop-status');
    const btn = document.getElementById('btn-net');
    if (d.waypoints && d.waypoints.length > 0) {
      const age = d.ts ? Math.round(Date.now() / 1000 - d.ts) : '?';
      el.textContent = `RV${d.rover}: ${d.waypoints.length}wp ${d.obstacles?.length||0}obs (${age}s ago)`;
      el.style.color = '#f39c12';
      btn.style.background = '#a05010';
    } else {
      el.textContent = 'no network mission';
      el.style.color = '#888';
      btn.style.background = '';
    }
  } catch(e) {}
}
setInterval(pollSnooped, 3000);
pollSnooped();

// ── Mission generator ─────────────────────────────────────────────
function toggleGenPanel() {
  const p = document.getElementById('gen-panel');
  p.style.display = p.style.display === 'block' ? 'none' : 'block';
}

function updateGenFields() {
  const pat = document.getElementById('gen-pattern').value;
  ['grid', 'zigzag', 'scatter', 'spiral'].forEach(t =>
    document.getElementById(`gen-${t}-fields`).style.display = t === pat ? 'block' : 'none');
}

function offsetLatLon(lat0, lon0, dNorth, dEast) {
  return [lat0 + dNorth / 111320,
          lon0 + dEast  / (111320 * Math.cos(lat0 * Math.PI / 180))];
}

function applyGenerate(append) {
  const pat   = document.getElementById('gen-pattern').value;
  const spd   = parseFloat(document.getElementById('gen-speed').value) || 0;
  const cMode = document.getElementById('gen-center').value;
  let cLat, cLon;
  if (cMode === 'start') {
    cLat = parseFloat(document.getElementById('s-lat').value);
    cLon = parseFloat(document.getElementById('s-lon').value);
  } else { cLat = viewLat; cLon = viewLon; }

  let newWps = [];

  if (pat === 'grid') {
    const rows  = parseInt(document.getElementById('gen-rows').value)     || 4;
    const cols  = parseInt(document.getElementById('gen-cols').value)     || 5;
    const rowSp = parseFloat(document.getElementById('gen-row-sp').value) || 5;
    const colSp = parseFloat(document.getElementById('gen-col-sp').value) || 5;
    const hdg   = (parseFloat(document.getElementById('gen-hdg').value) || 0) * Math.PI / 180;
    const tA = colSp * (cols - 1), tP = rowSp * (rows - 1);
    for (let r = 0; r < rows; r++) {
      const pts = [];
      for (let c = 0; c < cols; c++) {
        const a = c * colSp - tA / 2, p = r * rowSp - tP / 2;
        const [lat, lon] = offsetLatLon(cLat, cLon,
          a * Math.cos(hdg) - p * Math.sin(hdg),
          a * Math.sin(hdg) + p * Math.cos(hdg));
        pts.push({lat, lon, speed: spd, hold_secs: 0});
      }
      if (r % 2 === 1) pts.reverse();
      newWps.push(...pts);
    }

  } else if (pat === 'zigzag') {
    const legs   = parseInt(document.getElementById('gen-legs').value)      || 6;
    const legLen = parseFloat(document.getElementById('gen-leg-len').value) || 10;
    const legOff = parseFloat(document.getElementById('gen-leg-off').value) || 5;
    const hdg    = (parseFloat(document.getElementById('gen-zhdg').value) || 0) * Math.PI / 180;
    const tP     = legOff * (legs - 1);
    for (let i = 0; i < legs; i++) {
      const perp  = i * legOff - tP / 2;
      const along = (i % 2 === 0) ? -legLen / 2 : legLen / 2;
      const [lat, lon] = offsetLatLon(cLat, cLon,
        along * Math.cos(hdg) - perp * Math.sin(hdg),
        along * Math.sin(hdg) + perp * Math.cos(hdg));
      newWps.push({lat, lon, speed: spd, hold_secs: 0});
    }

  } else if (pat === 'scatter') {
    const nPts   = parseInt(document.getElementById('gen-npts').value)     || 10;
    const radius = parseFloat(document.getElementById('gen-radius').value) || 20;
    let   s      = parseInt(document.getElementById('gen-seed').value) || Date.now();
    const rnd = () => { s |= 0; s = s + 0x6D2B79F5 | 0;
      let t = Math.imul(s ^ s >>> 15, 1 | s); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
      return ((t ^ t >>> 14) >>> 0) / 4294967296; };
    for (let i = 0; i < nPts; i++) {
      const r = radius * Math.sqrt(rnd()), ang = rnd() * 2 * Math.PI;
      const [lat, lon] = offsetLatLon(cLat, cLon, r * Math.cos(ang), r * Math.sin(ang));
      newWps.push({lat, lon, speed: spd, hold_secs: 0});
    }

  } else if (pat === 'spiral') {
    const turns   = parseFloat(document.getElementById('gen-turns').value)    || 3;
    const armSp   = parseFloat(document.getElementById('gen-arm-sp').value)   || 5;
    const ptsTurn = parseInt(document.getElementById('gen-pts-turn').value)   || 10;
    const total   = Math.round(turns * ptsTurn);
    for (let i = 0; i < total; i++) {
      const theta = (i / ptsTurn) * 2 * Math.PI;
      const r     = armSp * theta / (2 * Math.PI);
      const [lat, lon] = offsetLatLon(cLat, cLon,
        r * Math.cos(theta - Math.PI / 2), r * Math.sin(theta - Math.PI / 2));
      newWps.push({lat, lon, speed: spd, hold_secs: 0});
    }
  }

  if (document.getElementById('gen-anchor').checked && newWps.length > 0) {
    const startLat = parseFloat(document.getElementById('s-lat').value);
    const startLon = parseFloat(document.getElementById('s-lon').value);
    if (!isNaN(startLat) && !isNaN(startLon)) {
      const dLat = startLat - newWps[0].lat;
      const dLon = startLon - newWps[0].lon;
      newWps.forEach(wp => { wp.lat += dLat; wp.lon += dLon; });
    }
  }

  if (!append) { waypoints = []; simResult = null; }
  waypoints.push(...newWps);
  // If anchor is off, WP[0] is not at the start input — sync start to WP[0]
  if (!document.getElementById('gen-anchor').checked) syncStartToWp0();
  refresh();
  fitAll();
  toggleGenPanel();
  status(`Generated ${newWps.length} waypoints (${pat}).`);
}

// ── Bulk speed ────────────────────────────────────────────────────
function applyBulkSpeed() {
  const spd = parseFloat(document.getElementById('bulk-speed').value) || 0;
  waypoints.forEach(wp => wp.speed = spd);
  refresh();
  status(`Speed set to ${spd} m/s on all ${waypoints.length} waypoints.`);
}

async function importSnooped() {
  const resp = await fetch('/snooped_mission');
  const d = await resp.json();
  if (!d.waypoints || !d.waypoints.length) {
    status('No network mission captured yet — arm GQC and upload a mission.', '#e74c3c');
    return;
  }
  waypoints = d.waypoints;
  obstacles = []; obsCurPts = [];
  (d.obstacles || []).forEach(poly => obstacles.push(poly));
  refresh();
  if (waypoints.length) {
    syncStartToWp0();
    viewLat = waypoints[0].lat; viewLon = waypoints[0].lon; fitAll();
  }
  status(`Imported from RV${d.rover}: ${waypoints.length} wps, ${obstacles.length} obstacles.`);
}

// ── Playback / rover debug ─────────────────────────────────────────
let pbTrace = [];       // debug_trace frames from simulate response
let pbIdx   = 0;        // current frame index
let pbTimer = null;     // setInterval id
let pbRate  = 1;        // playback speed multiplier
let pbPlaying = false;

function pbInit(trace) {
  pbTrace = trace || [];
  pbIdx = 0;
  pbPlaying = false;
  if (pbTimer) { clearInterval(pbTimer); pbTimer = null; }
  const panel = document.getElementById('playback');
  if (!pbTrace.length) { panel.style.display = 'none'; return; }
  panel.style.display = 'block';
  const slider = document.getElementById('pb-slider');
  slider.max = pbTrace.length - 1;
  slider.value = 0;
  document.getElementById('pb-play').innerHTML = '&#9654;';
  pbUpdateInfo();
  redraw();
}

function pbToggle() {
  if (!pbTrace.length) return;
  pbPlaying = !pbPlaying;
  document.getElementById('pb-play').innerHTML = pbPlaying ? '&#9646;&#9646;' : '&#9654;';
  if (pbPlaying) {
    const fps = 10;
    if (pbTimer) clearInterval(pbTimer);
    pbTimer = setInterval(() => {
      pbIdx += Math.max(1, Math.round(pbRate));
      if (pbIdx >= pbTrace.length) { pbIdx = pbTrace.length - 1; pbToggle(); return; }
      document.getElementById('pb-slider').value = pbIdx;
      pbUpdateInfo();
      redraw();
    }, 1000 / fps);
  } else {
    if (pbTimer) { clearInterval(pbTimer); pbTimer = null; }
  }
}

function pbSeek(v) {
  pbIdx = parseInt(v);
  pbUpdateInfo();
  redraw();
}

function pbSetSpeed(v) { pbRate = parseFloat(v); }

function pbDownload() {
  if (!pbTrace.length) return;
  const blob = new Blob([JSON.stringify(pbTrace, null, 1)], {type:'application/json'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'sim_debug_trace.json';
  a.click();
  URL.revokeObjectURL(a.href);
}

function pbUpdateInfo() {
  if (!pbTrace.length) return;
  const f = pbTrace[Math.min(pbIdx, pbTrace.length - 1)];
  document.getElementById('pb-time').textContent =
    `t=${f.t.toFixed(2)}s  frame ${pbIdx+1}/${pbTrace.length}`;
  const info = document.getElementById('pb-info');
  const modeColor = f.m === 'ttr' ? '#e67e22' : (f.m === 'mpc' ? '#9b59b6' : '#2ecc71');
  info.innerHTML =
    `<span style="color:${modeColor}"><b>${f.m || (f.pv ? 'PIVOT' : '—')}</b></span>` +
    ` &nbsp;WP: <b>${f.wp}</b>  dist: ${f.dw.toFixed(2)}m` +
    `<br>hdg: <b>${f.h.toFixed(1)}&deg;</b>  hdg_err: <span style="color:${Math.abs(f.he)>10?'#e74c3c':'#2ecc71'}">${f.he>0?'+':''}${f.he.toFixed(1)}&deg;</span>` +
    `  CTE: <span style="color:${Math.abs(f.cte)>0.3?'#e74c3c':'#2ecc71'}">${f.cte>0?'+':''}${f.cte.toFixed(3)}m</span>` +
    `<br>steer: <span style="color:#3498db">${f.sf>0?'+':''}${f.sf.toFixed(3)}</span>` +
    `  spd: ${f.v.toFixed(2)}m/s` +
    `  PPM: T=${f.tp} S=${f.sp}` +
    `  seg: ${f.cs!=null?f.cs:'?'}` + (f.cs !== f.bs ? `<span style="color:#e67e22">/${f.bs}</span>` : '') +
    (f.pv ? ' <b style="color:#e67e22">PIVOT</b>' : '');
}

function drawRover() {
  if (!pbTrace.length) return;
  const f = pbTrace[Math.min(pbIdx, pbTrace.length - 1)];
  const rear   = project(f.rl, f.ro);
  const front  = project(f.fl, f.fo);
  const center = project(f.cl, f.co);

  // Line connecting antennas
  ctx.save();
  ctx.strokeStyle = 'rgba(255,255,255,0.5)';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(rear.x, rear.y); ctx.lineTo(front.x, front.y); ctx.stroke();

  // Heading arrow from center
  const hRad = f.h * Math.PI / 180;
  const arrowLen = 20;
  const ax = center.x + Math.sin(hRad) * arrowLen;
  const ay = center.y - Math.cos(hRad) * arrowLen;
  ctx.strokeStyle = '#f1c40f';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(center.x, center.y); ctx.lineTo(ax, ay); ctx.stroke();
  // Arrowhead
  const aRad = 0.4;
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(ax - 6*Math.sin(hRad-aRad), ay + 6*Math.cos(hRad-aRad));
  ctx.moveTo(ax, ay);
  ctx.lineTo(ax - 6*Math.sin(hRad+aRad), ay + 6*Math.cos(hRad+aRad));
  ctx.stroke();

  // Rear antenna (red)
  ctx.fillStyle = '#e74c3c';
  ctx.beginPath(); ctx.arc(rear.x, rear.y, 5, 0, 2*Math.PI); ctx.fill();
  ctx.fillStyle = '#fff'; ctx.font = '9px sans-serif'; ctx.textAlign = 'left';
  ctx.fillText('R', rear.x + 7, rear.y + 3);

  // Front antenna (cyan)
  ctx.fillStyle = '#00bcd4';
  ctx.beginPath(); ctx.arc(front.x, front.y, 5, 0, 2*Math.PI); ctx.fill();
  ctx.fillStyle = '#fff';
  ctx.fillText('F', front.x + 7, front.y + 3);

  // Center (yellow)
  ctx.fillStyle = '#f1c40f';
  ctx.beginPath(); ctx.arc(center.x, center.y, 3, 0, 2*Math.PI); ctx.fill();

  // CTE bar visualization: small perpendicular line from center showing CTE magnitude
  if (f.cte !== 0) {
    const perpRad = hRad + (f.cte > 0 ? -Math.PI/2 : Math.PI/2);
    const cteScale = Math.min(Math.abs(f.cte) * 50, 30);  // pixels, capped
    const cx2 = center.x + Math.sin(perpRad) * cteScale;
    const cy2 = center.y - Math.cos(perpRad) * cteScale;
    ctx.strokeStyle = Math.abs(f.cte) > 0.3 ? '#e74c3c' : '#f39c12';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(center.x, center.y); ctx.lineTo(cx2, cy2); ctx.stroke();
  }

  ctx.restore();
}

// ── Live rover overlay ────────────────────────────────────────────
const ROVER_COLORS = {1: '#ff7700', 2: '#00e5ff'};  // orange=RV1, cyan=RV2

function drawLiveRovers() {
  const now = Date.now() / 1000;
  for (const [sid, r] of Object.entries(liveRovers)) {
    if (now - r.ts > 5) continue;   // stale > 5 s — hide
    const id = parseInt(sid);
    const color = ROVER_COLORS[id] || '#aaa';
    const p = project(r.lat, r.lon);
    ctx.save();
    ctx.translate(p.x, p.y);
    if (r.hdg != null) ctx.rotate(r.hdg * Math.PI / 180);
    // Chevron arrow (same shape as Android GQC)
    const dp = 10;
    ctx.beginPath();
    ctx.moveTo(0, -dp * 1.4);
    ctx.lineTo(dp, dp * 0.7);
    ctx.lineTo(0, dp * 0.2);
    ctx.lineTo(-dp, dp * 0.7);
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.9;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.restore();
    // Label
    ctx.fillStyle = color;
    ctx.font = 'bold 11px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText('RV' + id, p.x + 13, p.y + 4);
  }
}

async function pollLiveRovers() {
  try {
    const resp = await fetch('/rover_live');
    const d = await resp.json();
    liveRovers = d;
    redraw();
  } catch(e) {}
}
setInterval(pollLiveRovers, 2000);
pollLiveRovers();

// ── Init ──────────────────────────────────────────────────────────
resizeCanvas();
</script>
</body>
</html>
"""


def _build_page(default_lat: float, default_lon: float) -> str:
    return (_HTML_TEMPLATE
            .replace('START_LAT', str(default_lat))
            .replace('START_LON', str(default_lon))
            .replace('SERVER_VER', _SERVER_VER))


# ── HTTP handler ──────────────────────────────────────────────────────────────

class _Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == '/':
            # Redirect to versioned URL — browser can't serve this from cache
            self.send_response(302)
            self.send_header('Location', f'/?v={_SERVER_VER}')
            self.end_headers()
            return
        if self.path == f'/?v={_SERVER_VER}':
            body = _build_page(self.server.default_lat,
                               self.server.default_lon).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.end_headers()
            self.wfile.write(body)

        elif self.path == '/missions':
            self._json(_list_mission_files())

        elif self.path.startswith('/load_mission?name='):
            name = self.path.split('=', 1)[1]
            try:
                self._json(_load_mission_file(name))
            except FileNotFoundError:
                self.send_error(404, f'Mission not found: {name}')

        elif self.path == '/snooped_mission':
            with _snooped_lock:
                data = dict(_snooped)
            self._json(data)

        elif self.path == '/rover_live':
            with _rover_live_lock:
                data = {str(k): v for k, v in _rover_live.items()}
            self._json(data)

        else:
            self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        raw = self.rfile.read(length)

        if self.path == '/simulate':
            data = json.loads(raw)
            wps_raw = data.get('waypoints', [])
            if len(wps_raw) < 2:
                self._json({'error': 'need at least 2 waypoints'})
                return

            wps = [SimWaypoint(seq=i, lat=float(w['lat']), lon=float(w['lon']),
                               speed=float(w.get('speed') or 0),
                               hold_secs=float(w.get('hold_secs') or 0))
                   for i, w in enumerate(wps_raw)]

            start = data.get('start', {})
            start_lat = float(start.get('lat') or wps[0].lat)
            start_lon = float(start.get('lon') or wps[0].lon)
            start_hdg = float(start.get('heading')) if start.get('heading') else None

            nav_p      = {**DEFAULT_NAV,  **(data.get('nav_params',  {}) or {})}
            phys_p     = {**DEFAULT_PHYS, **(data.get('phys_params', {}) or {})}
            obstacles  = data.get('obstacles') or []

            print(f'[simulate] wps={len(wps)}  obstacles={len(obstacles)}', flush=True)
            # Auto-save current mission so it can be reloaded after restart
            try:
                _save_mission_file('_autosave',
                                   [{'lat': w.lat, 'lon': w.lon,
                                     'speed': w.speed, 'hold_secs': w.hold_secs}
                                    for w in wps],
                                   obstacles)
            except Exception:
                pass

            result = simulate(wps, start_lat, start_lon, start_hdg,
                              nav_params=nav_p, phys_params=phys_p,
                              obstacles=obstacles if obstacles else None)

            # Compute pivot waypoints for the frontend (all chunks, not just first)
            pivot_wps = compute_pivot_wps(
                _rerouted_as_wps(result.rerouted_wps, wps),
                start_lat, start_lon,
                nav_p.get('pivot_threshold', 25.0))

            self._json({
                'complete':          result.complete,
                'rms_xte':           result.rms_xte,
                'max_xte':           result.max_xte,
                'avg_xte':           result.avg_xte,
                'total_steps':       result.total_steps,
                'path':              result.path,
                'xte_log':           result.xte_log,
                'waypoints_reached': result.waypoints_reached,
                'rerouted_wps':      result.rerouted_wps,
                'obstacle_polygons': result.obstacle_polygons,
                'pivot_wps':         pivot_wps,
                'algorithm':         nav_p.get('control_algorithm', 'stanley'),
                'debug_trace':       result.debug_trace,
            })

        elif self.path == '/save_mission':
            data = json.loads(raw)
            name = data.get('name', '').strip().replace('/', '_').replace('\\', '_')
            if not name:
                self._json({'error': 'name required'}); return
            path = _save_mission_file(name, data.get('waypoints', []),
                                      data.get('obstacles', []))
            print(f'[save] {path}', flush=True)
            self._json({'ok': True, 'name': name})

        elif self.path == '/upload_rover':
            try:
                data      = json.loads(raw)
                rover_ip  = data.get('rover_ip', '192.168.100.19')
                rover_port = int(data.get('rover_port', 14550))
                rover_sysid = int(data.get('rover_sysid', 1))
                wps = data.get('waypoints', [])
                obs = data.get('obstacles', [])
                print(f'[upload] -> RV{rover_sysid} @ {rover_ip}:{rover_port}  '
                      f'wps={len(wps)} obs={len(obs)}', flush=True)
                result = _mavlink_upload(wps, obs, rover_ip, rover_port, rover_sysid)
                print(f'[upload] {result["message"]}', flush=True)
                self._json(result)
            except Exception as e:
                print(f'[upload] handler error: {e}', flush=True)
                self._json({'ok': False, 'message': f'Server error: {e}'})

        elif self.path == '/export_csv':
            data = json.loads(raw)
            buf = io.StringIO()
            w = csv.DictWriter(buf, fieldnames=['lat', 'lon', 'speed', 'hold_secs'])
            w.writeheader()
            for wp in data.get('waypoints', []):
                w.writerow({'lat': wp['lat'], 'lon': wp['lon'],
                            'speed': wp.get('speed', 0) or 0,
                            'hold_secs': wp.get('hold_secs', 0) or 0})
            body = buf.getvalue().encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/csv')
            self.send_header('Content-Disposition',
                             'attachment; filename="mission.csv"')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def _ok(self, ct: str, body: bytes):
        self.send_response(200)
        self.send_header('Content-Type', ct)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self._ok('application/json', body)

    def log_message(self, fmt, *args):
        pass   # suppress access log noise


def _rerouted_as_wps(rerouted_wps: list, originals: list[SimWaypoint]) -> list[SimWaypoint]:
    """Reconstruct SimWaypoint list from rerouted_wps pairs."""
    orig_set = {(round(w.lat, 7), round(w.lon, 7)) for w in originals}
    result = []
    for i, (lat, lon) in enumerate(rerouted_wps):
        is_bypass = (round(lat, 7), round(lon, 7)) not in orig_set
        result.append(SimWaypoint(seq=i, lat=lat, lon=lon, is_bypass=is_bypass))
    return result


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='AgriRover mission planner (web UI)')
    ap.add_argument('--port', type=int, default=DEFAULT_PORT,
                    help=f'HTTP port (default {DEFAULT_PORT})')
    ap.add_argument('--lat',  type=float, default=DEFAULT_LAT,
                    help='Default map centre latitude')
    ap.add_argument('--lon',  type=float, default=DEFAULT_LON,
                    help='Default map centre longitude')
    args = ap.parse_args()

    try:
        import socket as _s
        _sock = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
        _sock.connect(('8.8.8.8', 80))
        local_ip = _sock.getsockname()[0]
        _sock.close()
    except Exception:
        local_ip = 'localhost'

    server = ThreadingHTTPServer(('', args.port), _Handler)
    server.default_lat = args.lat
    server.default_lon = args.lon

    url = f'http://{local_ip}:{args.port}'
    print(f'Mission Planner ->  {url}')
    print('Ctrl+C to stop')

    _start_snooper()

    import threading, webbrowser
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
