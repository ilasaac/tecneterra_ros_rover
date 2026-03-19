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
"""

from __future__ import annotations

import csv
import io
import json
import math
import os
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_navigator import (  # noqa: E402
    DEFAULT_NAV, DEFAULT_PHYS, PathNavigator, SimWaypoint, simulate,
)

DEFAULT_LAT  = 20.727715
DEFAULT_LON  = -103.566782
DEFAULT_PORT = 8089

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
</style>
</head>
<body>
<div id="sidebar">
  <div class="toolbar">
    <button class="btn-blue" id="btn-add" onclick="toggleAddMode()">+ Add WP</button>
    <button class="btn-green" onclick="runSimulate()">&#9654; Simulate</button>
    <button class="btn-orange" onclick="exportCSV()">&#8595; CSV</button>
    <button class="btn-blue" onclick="document.getElementById('file-import').click()">&#8593; Import</button>
    <button class="btn-red" onclick="clearAll()">&#10005; Clear</button>
    <input type="file" id="file-import" accept=".csv" style="display:none" onchange="importCSV(event)">
  </div>
  <div id="status-bar">Click "+ Add WP" then click the map to place waypoints.</div>
  <div class="section">
    <label>Start lat</label><input id="s-lat" size="11" value="START_LAT">
    <label>lon</label><input id="s-lon" size="12" value="START_LON">
    <label>hdg&deg;</label><input id="s-hdg" size="4" value="0">
  </div>
  <div id="wp-list">
    <table>
      <thead><tr><th>#</th><th>Lat</th><th>Lon</th><th>Spd</th><th></th></tr></thead>
      <tbody id="wp-tbody"></tbody>
    </table>
  </div>
  <div id="stats"></div>
</div>
<div id="map"></div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
// ── Map ──────────────────────────────────────────────────────────
const map = L.map('map').setView([START_LAT, START_LON], 18);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {attribution:'ESRI',maxZoom:21,maxNativeZoom:19}).addTo(map);

// ── State ────────────────────────────────────────────────────────
let waypoints = [];
let wpMarkers = [], routeLine = null;
let simLayers = [];
let addMode = false;

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

map.on('click', e => { if (addMode) addWp(e.latlng.lat, e.latlng.lng); });

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
      html: `<div style="background:#1a7a3a;color:#fff;border-radius:50%;width:24px;height:24px;
             display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:bold;
             border:2px solid #fff;box-shadow:0 1px 4px rgba(0,0,0,.7)">${i}</div>`,
      className:'', iconAnchor:[12,12]
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
      {color:'#2ecc71',weight:2,dashArray:'8,5',opacity:.7}).addTo(map);

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
      body: JSON.stringify({waypoints, start:{lat:startLat, lon:startLon, heading:startHdg}})
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
      const icon = L.divIcon({
        html: `<div style="background:#e67e22;color:#fff;border-radius:50%;width:28px;height:28px;
               display:flex;align-items:center;justify-content:center;font-size:10px;font-weight:bold;
               border:3px solid #fff;box-shadow:0 1px 6px rgba(0,0,0,.8);
               white-space:nowrap">&#8635;</div>`,
        className:'', iconAnchor:[14,14]
      });
      const m = L.marker([pw.lat, pw.lon], {icon, zIndexOffset:1000})
        .addTo(map).bindTooltip(`Pivot ${pw.turn_angle.toFixed(0)}\u00b0`);
      simLayers.push(m);
    });

    const dur = (d.total_steps / 25).toFixed(1);
    statsEl.innerHTML = `
      <div><span class="lbl">Complete:</span> ${d.complete?'<b style="color:#2ecc71">\u2713 Yes</b>':'<b style="color:#e74c3c">\u2717 Timeout</b>'}</div>
      <div><span class="lbl">Duration:</span> ${dur} s (${d.total_steps} steps)</div>
      <div><span class="lbl">WP reached:</span> ${(d.waypoints_reached||[]).length}/${waypoints.length}</div>
      <div><span class="lbl">XTE rms:</span> ${d.rms_xte.toFixed(3)} m</div>
      <div><span class="lbl">XTE max:</span> ${d.max_xte.toFixed(3)} m</div>
      <div><span class="lbl">Pivot WPs:</span> ${(d.pivot_wps||[]).length}</div>
      <div style="font-size:10px;color:#666;margin-top:4px">Path: green=on-track \u2192 red=off-track<br>Orange circles = pivot turns</div>`;
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

// ── Clear ────────────────────────────────────────────────────────
function clearAll() {
  waypoints=[];refresh();clearSimOverlay();
  document.getElementById('stats').style.display='none';
  status('Cleared.');
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
            self._ok('text/html', body)
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

            nav_p   = {**DEFAULT_NAV,  **(data.get('nav_params',  {}) or {})}
            phys_p  = {**DEFAULT_PHYS, **(data.get('phys_params', {}) or {})}

            result = simulate(wps, start_lat, start_lon, start_hdg,
                              nav_params=nav_p, phys_params=phys_p)

            # Compute pivot waypoints for the frontend
            path_n = PathNavigator(
                _rerouted_as_wps(result.rerouted_wps, wps),
                start_lat, start_lon, nav_p)
            pivot_wps = []
            for i in range(len(path_n._wps)):
                if not path_n._wps[i].is_bypass:
                    ta = path_n._turn_angle_at(i)
                    if ta >= nav_p.get('pivot_threshold', 25.0):
                        pivot_wps.append({'lat': path_n._wps[i].lat,
                                          'lon': path_n._wps[i].lon,
                                          'turn_angle': round(ta, 1)})

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

    server = HTTPServer(('', args.port), _Handler)
    server.default_lat = args.lat
    server.default_lon = args.lon

    url = f'http://{local_ip}:{args.port}'
    print(f'Mission Planner ->  {url}')
    print('Ctrl+C to stop')

    import threading, webbrowser
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
