"""
tools/mission_planner.py — Web-based mission route editor + SIL simulator.

Usage:
  python tools/mission_planner.py [--port 8089] [--lat LAT] [--lon LON]

Then open:  http://localhost:8089
  - Click map to add waypoints (enable with "+ Add WP" button)
  - Right-click a waypoint marker to delete it
  - Drag markers to reposition
  - Import / Export CSV (same format as mission_uploader.py)
  - Press Simulate to run sim_navigator SIL and see the path on the map
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
    DEFAULT_NAV, DEFAULT_PHYS, PathNavigator, SimWaypoint, simulate,
)

DEFAULT_LAT  = 20.727715
DEFAULT_LON  = -103.566782
DEFAULT_PORT = 8089

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
                if t == 'MISSION_COUNT':
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
    # Build item list: fence vertices first, then nav waypoints
    items = []
    for poly in (obstacles or []):
        n = len(poly)
        for v in poly:
            items.append({'cmd': 5003, 'p1': float(n), 'p2': 0.0, 'p3': 0.0, 'p4': 0.0,
                          'lat': float(v[0]), 'lon': float(v[1])})
    for wp in waypoints:
        items.append({'cmd': 16, 'p1': 0.0,
                      'p2': float(wp.get('acceptance_radius') or DEFAULT_NAV['default_acceptance_radius']),
                      'p3': 0.0, 'p4': float(wp.get('speed') or 0.0),
                      'lat': float(wp['lat']), 'lon': float(wp['lon'])})
    if not items:
        return {'ok': False, 'message': 'No items to upload'}
    try:
        sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        mav  = _mav_def.MAVLink(None)
        mav.srcSystem    = 255   # GCS sysid
        mav.srcComponent = 0
        def _send(pkt):
            sock.sendto(pkt.pack(mav), (rover_ip, rover_port))
        _send(mav.mission_count_encode(rover_sysid, 0, len(items),
                                       mavtype=0))
        _time.sleep(0.15)   # GQC streaming delay
        for seq, item in enumerate(items):
            _send(mav.mission_item_int_encode(
                rover_sysid, 0, seq,
                6,             # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                item['cmd'],
                0, 1,          # current=0, autocontinue=1
                item['p1'], item['p2'], item['p3'], item['p4'],
                int(item['lat'] * 1e7), int(item['lon'] * 1e7), 0,
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
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{display:flex;height:100vh;font-family:Arial,sans-serif;font-size:13px;background:#111}
#sidebar{width:310px;background:#1a1a2e;color:#ddd;display:flex;flex-direction:column;overflow:hidden;flex-shrink:0}
#map{flex:1}
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
</style>
</head>
<body>
<div id="sidebar">
  <div class="toolbar">
    <button class="btn-blue" id="btn-add" onclick="toggleAddMode()">+ Add WP</button>
    <button id="btn-obs" class="btn-red" onclick="toggleObsMode()">&#9632; Obstacle</button>
    <button class="btn-green" onclick="runSimulate()">&#9654; Simulate</button>
    <button class="btn-orange" onclick="exportCSV()">&#8595; CSV</button>
    <button class="btn-blue" onclick="document.getElementById('file-import').click()">&#8593; Import</button>
    <button class="btn-red" onclick="clearAll()">&#10005; Clear</button>
    <button class="btn-orange" onclick="toggleGenPanel()">&#9881; Gen</button>
    <input type="file" id="file-import" accept=".csv" style="display:none" onchange="importCSV(event)">
  </div>
  <div style="padding:3px 6px;background:#0d0d1a;border-bottom:1px solid #333;display:flex;align-items:center;gap:4px">
    <span style="color:#888;font-size:10px">All spd:</span>
    <input id="bulk-speed" type="number" value="1.0" min="0" max="1.5" step="0.1"
           style="width:42px;background:#0a1020;color:#eee;border:1px solid #446;padding:2px 3px;border-radius:2px;font-size:11px">
    <span style="color:#888;font-size:10px">m/s</span>
    <button onclick="applyBulkSpeed()" style="padding:2px 7px;font-size:11px;background:#0f3460;color:#fff;border:none;border-radius:2px;cursor:pointer">&#10003;</button>
  </div>
  <div id="status-bar">Click "+ Add WP" then click the map to place waypoints.</div>
  <div class="section">
    <label>Start lat</label><input id="s-lat" size="11" value="START_LAT">
    <label>lon</label><input id="s-lon" size="12" value="START_LON">
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
      <button id="btn-rv1" class="btn-blue btn-active" style="padding:3px 8px;font-size:11px" onclick="selectRover(1)">RV1</button>
      <button id="btn-rv2" class="btn-blue" style="padding:3px 8px;font-size:11px" onclick="selectRover(2)">RV2</button>
      <input id="r-ip" size="14" placeholder="192.168.100.19" style="background:#0a1020;color:#eee;border:1px solid #446;padding:2px 4px;border-radius:2px;flex:1">
    </div>
    <div style="display:flex;gap:3px;align-items:center">
      <button class="btn-green" style="padding:3px 9px;font-size:11px" onclick="uploadRover()">&#9650; Upload</button>
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
</div>
<!-- ── Mission generator panel ──────────────────────────────────────── -->
<div id="gen-panel">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:7px">
    <b style="color:#fff;font-size:12px">&#9881; Generate Mission</b>
    <button onclick="toggleGenPanel()" style="background:none;border:none;color:#aaa;font-size:15px;cursor:pointer;padding:0 3px">&#10005;</button>
  </div>
  <label>Pattern</label>
  <select id="gen-pattern" onchange="updateGenFields()">
    <option value="grid">Grid — serpentine rows (180° U-turns)</option>
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
    <option value="map">Map centre</option>
    <option value="start">Start lat/lon</option>
  </select>
  <div class="grow">
    <button class="btn-red" style="flex:1;padding:4px" onclick="applyGenerate(false)">&#9654; Replace</button>
    <button class="btn-blue" style="flex:1;padding:4px" onclick="applyGenerate(true)">&#43; Append</button>
  </div>
</div>
<div id="map"></div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
// ── Map ──────────────────────────────────────────────────────────
const map = L.map('map').setView([START_LAT, START_LON], 18);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {attribution:'ESRI',maxZoom:23,maxNativeZoom:19}).addTo(map);

// ── State ────────────────────────────────────────────────────────
let waypoints = [];
let wpMarkers = [], routeLine = null;
let simLayers = [];
let addMode = false;

// Obstacle drawing state
let obstacles = [];          // completed polygons: [[[lat,lon],...], ...]
let obsMode = false;         // currently drawing an obstacle
let obsCurPts = [];          // vertices of polygon being drawn
let obsCurMarkers = [];      // temporary vertex markers
let obsCurLine = null;       // preview polyline while drawing
let obsLayers = [];          // rendered completed obstacle layers

function status(msg, color='#27ae60') {
  const el = document.getElementById('status-bar');
  el.style.color = color;
  el.textContent = msg;
}

// ── Add mode ─────────────────────────────────────────────────────
function toggleAddMode() {
  addMode = !addMode;
  const btn = document.getElementById('btn-add');
  btn.classList.toggle('btn-active', addMode);
  map.getContainer().style.cursor = addMode ? 'crosshair' : '';
  status(addMode ? 'Click map to place waypoints. Right-click marker to delete.' : 'Add mode off.');
}

map.on('click', e => {
  if (addMode) addWp(e.latlng.lat, e.latlng.lng);
  else if (obsMode) obsAddVertex(e.latlng.lat, e.latlng.lng);
});

// ── Waypoints ────────────────────────────────────────────────────
function addWp(lat, lon, speed=0, hold=0) {
  waypoints.push({lat, lon, speed, hold_secs: hold});
  refresh();
}

function removeWp(i) { waypoints.splice(i,1); refresh(); }

function refresh() {
  // Remove old markers + route
  wpMarkers.forEach(m => map.removeLayer(m)); wpMarkers = [];
  if (routeLine) { map.removeLayer(routeLine); routeLine = null; }

  waypoints.forEach((wp, i) => {
    const icon = L.divIcon({
      html: `<div style="background:rgba(26,122,58,0.45);color:#fff;border-radius:50%;width:22px;height:22px;
             display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:bold;
             border:1.5px solid rgba(255,255,255,0.7);box-shadow:0 1px 3px rgba(0,0,0,.4)">${i}</div>`,
      className:'', iconAnchor:[11,11]
    });
    const m = L.marker([wp.lat, wp.lon], {icon, draggable:true})
      .addTo(map)
      .bindTooltip(`WP${i} (${wp.lat.toFixed(6)}, ${wp.lon.toFixed(6)})<br>speed:${wp.speed} m/s`);
    m.on('dragend', e => {
      const ll = e.target.getLatLng();
      waypoints[i].lat = ll.lat; waypoints[i].lon = ll.lng;
      refresh();
    });
    m.on('contextmenu', () => removeWp(i));
    wpMarkers.push(m);
  });

  if (waypoints.length > 1)
    routeLine = L.polyline(waypoints.map(w=>[w.lat,w.lon]),
      {color:'#2ecc71',weight:1.5,dashArray:'6,4',opacity:.5}).addTo(map);

  refreshTable();
  status(`${waypoints.length} waypoint(s).`);
}

function refreshTable() {
  const tb = document.getElementById('wp-tbody');
  tb.innerHTML = '';
  waypoints.forEach((wp, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td style="color:#aaa">${i}</td>
      <td><input value="${wp.lat.toFixed(7)}" onchange="waypoints[${i}].lat=+this.value;refresh()"></td>
      <td><input value="${wp.lon.toFixed(7)}" onchange="waypoints[${i}].lon=+this.value;refresh()"></td>
      <td><input value="${wp.speed||0}" style="width:38px"
          onchange="waypoints[${i}].speed=+this.value||0"></td>
      <td><button onclick="removeWp(${i})"
          style="background:#7a1515;color:#fff;border:none;padding:1px 6px;border-radius:2px;cursor:pointer">&#10005;</button></td>`;
    tb.appendChild(tr);
  });
}

// ── Simulation ───────────────────────────────────────────────────
function clearSimOverlay() {
  simLayers.forEach(l => map.removeLayer(l)); simLayers = [];
}

async function runSimulate() {
  if (waypoints.length < 2) { status('Need at least 2 waypoints.','#e74c3c'); return; }
  // Auto-finalize any in-progress obstacle polygon before running
  if (obsMode) obsFinish();
  clearSimOverlay();
  const statsEl = document.getElementById('stats');
  statsEl.style.display = 'block';
  statsEl.innerHTML = '<span style="color:#f39c12">&#9654; Simulating...</span>';
  status('Running simulation...','#f39c12');

  const startLat = parseFloat(document.getElementById('s-lat').value) || waypoints[0].lat;
  const startLon = parseFloat(document.getElementById('s-lon').value) || waypoints[0].lon;
  const startHdg = parseFloat(document.getElementById('s-hdg').value) || 0;

  try {
    const resp = await fetch('/simulate', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({waypoints, start:{lat:startLat, lon:startLon, heading:startHdg}, obstacles})
    });
    const d = await resp.json();

    // Draw path colored green→red by XTE
    const xteMax = Math.max(...(d.xte_log||[0.001]), 0.001);
    for (let i = 0; i+1 < d.path.length && i < (d.xte_log||[]).length; i++) {
      const t = d.xte_log[i] / xteMax;
      const r2 = Math.round(255 * Math.min(t*2, 1));
      const g2 = Math.round(255 * Math.max(1 - t*2 + 1, 0));
      const seg = L.polyline([d.path[i], d.path[i+1]],
        {color:`rgb(${r2},${g2},30)`, weight:3, opacity:.85}).addTo(map);
      simLayers.push(seg);
    }

    // Pivot waypoint markers — rendered after path so they appear on top
    (d.pivot_wps||[]).forEach(pw => {
      const isBypass = pw.is_bypass;
      const sz = isBypass ? 22 : 26;
      const fs = isBypass ? 11 : 13;
      const bg = isBypass ? 'rgba(180,80,200,0.5)' : 'rgba(230,126,34,0.45)';
      const icon = L.divIcon({
        html: `<div style="background:${bg};color:#fff;border-radius:50%;width:${sz}px;height:${sz}px;
               display:flex;align-items:center;justify-content:center;font-size:${fs}px;font-weight:bold;
               border:1.5px solid rgba(255,255,255,0.7);box-shadow:0 1px 3px rgba(0,0,0,.4)">&#8635;</div>`,
        className:'', iconAnchor:[sz/2,sz/2]
      });
      const label = (isBypass ? 'Bypass pivot ' : 'Pivot ') + pw.turn_angle.toFixed(0) + '\u00b0';
      const m = L.marker([pw.lat, pw.lon], {icon, zIndexOffset:1000})
        .addTo(map).bindTooltip(label);
      simLayers.push(m);
    });

    // Rerouted path (bypass around obstacles) — white dashed overlay
    if (obstacles.length && (d.rerouted_wps||[]).length > 1) {
      const rl = L.polyline(d.rerouted_wps, {
        color:'#fff', weight:1.5, dashArray:'5,4', opacity:.55
      }).addTo(map);
      simLayers.push(rl);
    }

    const dur = (d.total_steps / 25).toFixed(1);
    statsEl.innerHTML = `
      <div><span class="lbl">Complete:</span> ${d.complete?'<b style="color:#2ecc71">\u2713 Yes</b>':'<b style="color:#e74c3c">\u2717 Timeout</b>'}</div>
      <div><span class="lbl">Duration:</span> ${dur} s (${d.total_steps} steps)</div>
      <div><span class="lbl">WP reached:</span> ${(d.waypoints_reached||[]).length}/${waypoints.length}</div>
      <div><span class="lbl">XTE rms:</span> ${d.rms_xte.toFixed(3)} m</div>
      <div><span class="lbl">XTE max:</span> ${d.max_xte.toFixed(3)} m</div>
      <div><span class="lbl">Pivot WPs:</span> ${(d.pivot_wps||[]).length}</div>
      <div><span class="lbl">Obstacles:</span> ${obstacles.length}</div>
      <div style="font-size:10px;color:#666;margin-top:4px">Path: green=on-track \u2192 red=high XTE<br>Orange \u21bb = pivot &nbsp; Purple \u21bb = bypass pivot &nbsp; White dashed = reroute</div>`;
    status(`Done. XTE rms=${d.rms_xte.toFixed(3)}m max=${d.max_xte.toFixed(3)}m`);
  } catch(e) {
    statsEl.innerHTML = `<span style="color:#e74c3c">Error: ${e}</span>`;
    status('Simulation failed.','#e74c3c');
  }
}

// ── CSV export ───────────────────────────────────────────────────
function exportCSV() {
  fetch('/export_csv', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({waypoints})
  }).then(r=>r.blob()).then(blob => {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'mission.csv'; a.click();
  });
}

// ── CSV import ───────────────────────────────────────────────────
function importCSV(event) {
  const file = event.target.files[0]; if (!file) return;
  const reader = new FileReader();
  reader.onload = e => {
    const lines = e.target.result.trim().split('\n');
    const hdr = lines[0].toLowerCase().split(',');
    const li=hdr.indexOf('lat'), loi=hdr.indexOf('lon');
    const si=hdr.indexOf('speed'), hi=hdr.indexOf('hold_secs');
    if (li<0||loi<0){status('CSV must have lat,lon columns','#e74c3c');return;}
    waypoints = [];
    for (let i=1;i<lines.length;i++){
      const c=lines[i].split(','); if(c.length<2)continue;
      waypoints.push({lat:+c[li],lon:+c[loi],speed:si>=0?+c[si]||0:0,hold_secs:hi>=0?+c[hi]||0:0});
    }
    refresh();
    if (waypoints.length) map.setView([waypoints[0].lat,waypoints[0].lon],18);
    event.target.value='';
  };
  reader.readAsText(file);
}

// ── Obstacles ────────────────────────────────────────────────────
function toggleObsMode() {
  if (obsMode) {
    obsFinish();
  } else {
    obsMode = true;
    addMode = false;
    document.getElementById('btn-add').classList.remove('btn-active');
    document.getElementById('btn-obs').textContent = '\u2713 Done';
    document.getElementById('btn-obs').style.background = '#8b0000';
    map.getContainer().style.cursor = 'crosshair';
    status('Click map to draw obstacle polygon. Click "Done" to close it.', '#e74c3c');
  }
}

function obsAddVertex(lat, lon) {
  obsCurPts.push([lat, lon]);
  const m = L.circleMarker([lat, lon], {
    radius:4, color:'#e74c3c', fillColor:'#e74c3c', fillOpacity:1, weight:1
  }).addTo(map);
  obsCurMarkers.push(m);
  if (obsCurLine) map.removeLayer(obsCurLine);
  if (obsCurPts.length > 1) {
    const pts = [...obsCurPts, obsCurPts[0]];
    obsCurLine = L.polyline(pts, {color:'#e74c3c', weight:1.5, dashArray:'4,3', opacity:.7}).addTo(map);
  }
  status(`Obstacle: ${obsCurPts.length} vertex(es). Click "Done" to close (min 3).`, '#e74c3c');
}

function obsFinish() {
  obsMode = false;
  document.getElementById('btn-obs').textContent = '\u25a0 Obstacle';
  document.getElementById('btn-obs').style.background = '';
  map.getContainer().style.cursor = '';
  // Clear preview markers and line
  obsCurMarkers.forEach(m => map.removeLayer(m)); obsCurMarkers = [];
  if (obsCurLine) { map.removeLayer(obsCurLine); obsCurLine = null; }
  if (obsCurPts.length < 3) {
    status('Need at least 3 vertices — obstacle discarded.', '#e74c3c');
    obsCurPts = [];
    return;
  }
  const poly = [...obsCurPts];
  obsCurPts = [];
  const idx = obstacles.length;
  obstacles.push(poly);
  renderObstacle(poly, idx);
  status(`Obstacle ${idx} added (${poly.length} vertices). Total: ${obstacles.length}.`);
}

function renderObstacle(poly, idx) {
  const layer = L.polygon(poly, {
    color:'#e74c3c', weight:1.5,
    fillColor:'#e74c3c', fillOpacity:0.2, opacity:0.7
  }).addTo(map);
  layer.bindTooltip(`Obstacle ${idx} — right-click to delete`);
  layer.on('contextmenu', () => removeObstacle(idx));
  obsLayers.push({layer, idx});
}

function removeObstacle(idx) {
  obstacles.splice(idx, 1);
  obsLayers.forEach(o => map.removeLayer(o.layer)); obsLayers = [];
  obstacles.forEach((poly, i) => renderObstacle(poly, i));
  status(`Obstacle removed. Total: ${obstacles.length}.`);
}

function clearObstacles() {
  obstacles = []; obsCurPts = [];
  obsCurMarkers.forEach(m => map.removeLayer(m)); obsCurMarkers = [];
  if (obsCurLine) { map.removeLayer(obsCurLine); obsCurLine = null; }
  obsLayers.forEach(o => map.removeLayer(o.layer)); obsLayers = [];
}

// ── Clear ────────────────────────────────────────────────────────
function clearAll() {
  waypoints=[];refresh();clearSimOverlay();clearObstacles();
  document.getElementById('stats').style.display='none';
  status('Cleared.');
}

// ── Mission file save / load ──────────────────────────────────────
async function saveMission() {
  const name = document.getElementById('m-name').value.trim();
  if (!name) { status('Enter a mission name first.', '#e74c3c'); return; }
  await fetch('/save_mission', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({name, waypoints, obstacles})
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
  obstacles = [];
  clearObstacles();
  (d.obstacles || []).forEach((poly, i) => { obstacles.push(poly); renderObstacle(poly, i); });
  refresh();
  if (waypoints.length) map.setView([waypoints[0].lat, waypoints[0].lon], 18);
  status(`Loaded "${name}" — ${waypoints.length} wps, ${obstacles.length} obstacles.`);
}

async function refreshMissionList() {
  const resp = await fetch('/missions');
  const list = await resp.json();
  const sel = document.getElementById('m-select');
  const cur = sel.value;
  sel.innerHTML = '<option value="">— saved missions —</option>' +
    list.map(m => `<option value="${m.name}"${m.name===cur?' selected':''}>${m.name} (${m.wp_count}wp${m.obs_count?' '+m.obs_count+'obs':''})</option>`).join('');
}
refreshMissionList();

// ── Rover upload / network import ────────────────────────────────
let selectedRover = 1;
const roverIPs = {1:'192.168.100.19', 2:'192.168.100.20'};

function selectRover(n) {
  selectedRover = n;
  document.getElementById('btn-rv1').classList.toggle('btn-active', n===1);
  document.getElementById('btn-rv2').classList.toggle('btn-active', n===2);
  document.getElementById('r-ip').value = roverIPs[n] || '';
}
document.getElementById('r-ip').value = roverIPs[1];

async function uploadRover() {
  if (waypoints.length < 1) { status('No waypoints to upload.', '#e74c3c'); return; }
  const rover_ip = document.getElementById('r-ip').value.trim();
  if (!rover_ip) { status('Enter rover IP.', '#e74c3c'); return; }
  status(`Uploading to RV${selectedRover}...`, '#f39c12');
  // Auto-finalize obstacle if drawing
  if (obsMode) obsFinish();
  const resp = await fetch('/upload_rover', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({waypoints, obstacles, rover_ip, rover_port:14550, rover_sysid:selectedRover})
  });
  const d = await resp.json();
  status(d.message, d.ok ? '#27ae60' : '#e74c3c');
}

// Poll for snooped network mission every 3 s
async function pollSnooped() {
  try {
    const resp = await fetch('/snooped_mission');
    const d = await resp.json();
    const el = document.getElementById('snoop-status');
    const btn = document.getElementById('btn-net');
    if (d.waypoints && d.waypoints.length > 0) {
      const age = d.ts ? Math.round((Date.now()/1000 - d.ts)) : '?';
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
  ['grid','zigzag','scatter','spiral'].forEach(t =>
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
  } else { const c = map.getCenter(); cLat = c.lat; cLon = c.lng; }

  let newWps = [];

  if (pat === 'grid') {
    const rows  = parseInt(document.getElementById('gen-rows').value)   || 4;
    const cols  = parseInt(document.getElementById('gen-cols').value)   || 5;
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
    const legs   = parseInt(document.getElementById('gen-legs').value)    || 6;
    const legLen = parseFloat(document.getElementById('gen-leg-len').value) || 10;
    const legOff = parseFloat(document.getElementById('gen-leg-off').value) || 5;
    const hdg    = (parseFloat(document.getElementById('gen-zhdg').value) || 0) * Math.PI / 180;
    const tP = legOff * (legs - 1);
    for (let i = 0; i < legs; i++) {
      const perp  = i * legOff - tP / 2;
      const along = (i % 2 === 0) ? -legLen / 2 : legLen / 2;
      const [lat, lon] = offsetLatLon(cLat, cLon,
        along * Math.cos(hdg) - perp * Math.sin(hdg),
        along * Math.sin(hdg) + perp * Math.cos(hdg));
      newWps.push({lat, lon, speed: spd, hold_secs: 0});
    }

  } else if (pat === 'scatter') {
    const nPts   = parseInt(document.getElementById('gen-npts').value)   || 10;
    const radius = parseFloat(document.getElementById('gen-radius').value) || 20;
    let   s      = parseInt(document.getElementById('gen-seed').value) || Date.now();
    // Mulberry32 seeded PRNG
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
      const [lat, lon] = offsetLatLon(cLat, cLon, r * Math.cos(theta - Math.PI/2), r * Math.sin(theta - Math.PI/2));
      newWps.push({lat, lon, speed: spd, hold_secs: 0});
    }
  }

  if (!append) { waypoints = []; clearSimOverlay(); }
  waypoints.push(...newWps);
  refresh();
  if (waypoints.length) map.fitBounds(waypoints.map(w => [w.lat, w.lon]), {padding:[50,50]});
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
  if (!d.waypoints || d.waypoints.length === 0) {
    status('No network mission captured yet — arm GQC and upload a mission.', '#e74c3c');
    return;
  }
  waypoints = d.waypoints;
  obstacles = [];
  clearObstacles();
  (d.obstacles || []).forEach((poly, i) => { obstacles.push(poly); renderObstacle(poly, i); });
  refresh();
  if (waypoints.length) map.setView([waypoints[0].lat, waypoints[0].lon], 18);
  status(`Imported from RV${d.rover}: ${waypoints.length} wps, ${obstacles.length} obstacles.`);
}
</script>
</body>
</html>
"""


def _build_page(default_lat: float, default_lon: float) -> str:
    return (_HTML_TEMPLATE
            .replace('START_LAT', str(default_lat))
            .replace('START_LON', str(default_lon)))


# ── HTTP handler ──────────────────────────────────────────────────────────────

class _Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            body = _build_page(self.server.default_lat,
                               self.server.default_lon).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache, no-store')
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
            start_hdg = float(start.get('heading') or 0)

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

            # Compute pivot waypoints for the frontend
            path_n = PathNavigator(
                _rerouted_as_wps(result.rerouted_wps, wps),
                start_lat, start_lon, nav_p)
            pivot_wps = []
            for i in range(len(path_n._wps)):
                ta = path_n._turn_angle_at(i)
                if ta >= nav_p.get('pivot_threshold', 25.0):
                    pivot_wps.append({'lat': path_n._wps[i].lat,
                                      'lon': path_n._wps[i].lon,
                                      'turn_angle': round(ta, 1),
                                      'is_bypass': path_n._wps[i].is_bypass})

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
            data      = json.loads(raw)
            rover_ip  = data.get('rover_ip', '192.168.100.19')
            rover_port = int(data.get('rover_port', 14550))
            rover_sysid = int(data.get('rover_sysid', 1))
            wps = data.get('waypoints', [])
            obs = data.get('obstacles', [])
            print(f'[upload] → RV{rover_sysid} @ {rover_ip}:{rover_port}  '
                  f'wps={len(wps)} obs={len(obs)}', flush=True)
            result = _mavlink_upload(wps, obs, rover_ip, rover_port, rover_sysid)
            print(f'[upload] {result["message"]}', flush=True)
            self._json(result)

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
