"""
tools/monitor.py — Live terminal dashboard for both rovers.

Listens on UDP :14550 — both rovers broadcast to this port (different Jetsons,
no conflict). Rovers are identified by MAVLink sysid in HEARTBEAT.

Also snoops GQC (sysid=255) mission uploads, runs a software-in-the-loop
simulation (via sim_navigator.py) as soon as a mission is fully uploaded, and
writes a Leaflet.js map to monitor_map.html that refreshes every 5 s.

The map is served over HTTP on MAP_PORT (default 8088) so it can be opened
from any machine on the network:  http://<this-host-ip>:8088/monitor_map.html

Usage:
  python tools/monitor.py [--map-port PORT]
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from http.server import HTTPServer, SimpleHTTPRequestHandler

os.environ['MAVLINK20'] = '1'
try:
    from pymavlink import mavutil
except ImportError:
    raise ImportError("pip install pymavlink")

# Simulation engine (same directory)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from sim_navigator import simulate as _sim_run, SimWaypoint   # noqa: E402
    _SIM_AVAILABLE = True
except ImportError:
    _SIM_AVAILABLE = False


@dataclass
class RoverState:
    rv_id:         int   = 0
    ip:            str   = '?'
    lat:           float = 0.0
    lon:           float = 0.0
    fix_type:      str   = 'NO_FIX'
    armed:         bool  = False
    mode:          str   = 'UNKNOWN'
    heading:       float = -1.0   # degrees, -1 = unknown
    battery_v:     float = 0.0
    battery_pct:   float = -1.0
    cmd_thr:       int   = 1500   # navigator throttle PPM
    cmd_str:       int   = 1500   # navigator steering PPM
    wp_active:     int   = -1    # active waypoint seq (-1 = none)
    wp_total:      int   = 0     # total waypoints in mission
    xte_m:         float = 0.0   # latest cross-track error (metres)
    xte_max:       float = 0.0   # max XTE observed this session
    xte_sum:       float = 0.0   # cumulative sum for average
    xte_count:     int   = 0     # number of XTE samples collected
    sbus_ch:       list  = field(default_factory=lambda: [1500] * 16)
    last_hb:       float = 0.0
    sensors:       dict  = field(default_factory=dict)
    log:           list  = field(default_factory=list)
    # Mission tracking (populated from snooped GQC packets)
    mission_raw:       dict  = field(default_factory=dict)  # seq -> (lat, lon), nav wps only
    mission_count:     int   = 0   # total items in current upload (incl. DO_SET_SERVO)
    mission_fence_buf: list  = field(default_factory=list)  # raw fence vertices (lat, lon, count)
    obstacle_polygons: list  = field(default_factory=list)  # parsed [[lat,lon],...] polygons
    sim_result:        object = None   # SimResult when simulation completes
    sim_running:       bool  = False
    actual_path:       list  = field(default_factory=list)  # [(lat, lon)] from GLOBAL_POSITION_INT


RV    = {1: RoverState(rv_id=1), 2: RoverState(rv_id=2)}
_lock = threading.Lock()

MAV_MODE_ARMED = 128
_MAP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'monitor_map.html')
_MAP_PORT = 8088   # overridden by --map-port arg


def _sbus_to_ppm(ch: list) -> list:
    """
    Mirror firmware apply_ppm_map() — converts raw SBUS (0-indexed) to the 8
    PPM channels that the motor controllers physically receive.

    PPM CH1 = SBUS CH3          (throttle)
    PPM CH2 = ~SBUS CH1         (steering,       inverted)
    PPM CH3 = ~SBUS CH5         (SWA emergency,  inverted)
    PPM CH4 = ~SBUS CH6         (SWB autonomous, inverted)
    PPM CH5 = SBUS CH11
    PPM CH6 = SBUS CH12
    PPM CH7 = ~SBUS CH7         (inverted)
    PPM CH8 = ~SBUS CH8         (inverted)

    Inversion: 3000 − value  (maps 1000↔2000, keeps 1500 centred)
    """
    inv = lambda v: 3000 - v
    return [
        ch[2],        # PPM1 = SBUS3
        inv(ch[0]),   # PPM2 = ~SBUS1
        inv(ch[4]),   # PPM3 = ~SBUS5
        inv(ch[5]),   # PPM4 = ~SBUS6
        ch[10],       # PPM5 = SBUS11
        ch[11],       # PPM6 = SBUS12
        inv(ch[6]),   # PPM7 = ~SBUS7
        inv(ch[7]),   # PPM8 = ~SBUS8
    ]


# ── Fence parsing ─────────────────────────────────────────────────────────────

def _parse_fence_buf(fence_buf: list) -> list:
    """Parse raw fence vertex tuples (lat, lon, vertex_count) into polygon list."""
    polygons = []
    current  = []
    expected = 0
    for (lat, lon, vertex_count) in fence_buf:
        if expected == 0:
            expected = vertex_count
        current.append([lat, lon])
        if len(current) >= expected:
            polygons.append(current)
            current  = []
            expected = 0
    if current:
        polygons.append(current)
    return polygons


# ── Simulation ────────────────────────────────────────────────────────────────

def _trigger_simulation(rv_id: int):
    """Build waypoints from mission_raw and run sim; store result in rv.sim_result."""
    if not _SIM_AVAILABLE:
        with _lock:
            RV[rv_id].sim_running = False
        return

    with _lock:
        rv = RV[rv_id]
        if not rv.mission_raw:
            rv.sim_running = False
            return
        waypoints = [
            SimWaypoint(seq=seq, lat=lat, lon=lon)
            for seq, (lat, lon) in sorted(rv.mission_raw.items())
        ]
        start_lat     = rv.lat
        start_lon     = rv.lon
        start_heading = rv.heading if rv.heading >= 0 else 0.0
        obstacles     = list(rv.obstacle_polygons)   # snapshot under lock

    try:
        result = _sim_run(waypoints, start_lat, start_lon, start_heading,
                          obstacles=obstacles if obstacles else None)
    except Exception:
        result = None

    with _lock:
        RV[rv_id].sim_result  = result
        RV[rv_id].sim_running = False


# ── Map generation ─────────────────────────────────────────────────────────────

def _build_map_html() -> str:
    """Generate a Leaflet.js HTML page with mission, simulated, and actual paths."""
    with _lock:
        rovers_snapshot = []
        for rv_id, rv in RV.items():
            # Build ordered mission path from nav waypoints only
            mission_path = [
                [lat, lon]
                for _, (lat, lon) in sorted(rv.mission_raw.items())
            ]
            sim_path = (
                [[lat, lon] for lat, lon in rv.sim_result.path]
                if rv.sim_result else []
            )
            actual_path = [[lat, lon] for lat, lon in rv.actual_path]
            rovers_snapshot.append({
                'id':               rv_id,
                'lat':              rv.lat,
                'lon':              rv.lon,
                'armed':            rv.armed,
                'mode':             rv.mode,
                'mission_path':     mission_path,
                'sim_path':         sim_path,
                'actual_path':      actual_path,
                'obstacle_polygons': list(rv.obstacle_polygons),
                'sim_rms':          rv.sim_result.rms_xte if rv.sim_result else None,
                'sim_max':          rv.sim_result.max_xte if rv.sim_result else None,
                'sim_complete':     rv.sim_result.complete if rv.sim_result else None,
            })

    # Map center: first rover with a valid fix, else default field
    center_lat, center_lon = 20.727715, -103.566782
    for rd in rovers_snapshot:
        if rd['lat'] != 0.0:
            center_lat, center_lon = rd['lat'], rd['lon']
            break

    data_json = json.dumps(rovers_snapshot)

    return f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>AgriRover Monitor Map</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  body  {{ margin:0; background:#111; color:#eee; font-family:monospace; font-size:13px; }}
  #map  {{ height:82vh; }}
  #info {{ padding:6px 12px; line-height:1.9; }}
  .dot  {{ display:inline-block; width:11px; height:11px; border-radius:50%; margin-right:5px;
            vertical-align:middle; }}
  .swatch {{ display:inline-block; width:28px; height:4px; vertical-align:middle;
              margin-right:5px; }}
  .controls {{ float: right; background: #222; padding: 2px 8px; border-radius: 4px; border: 1px solid #444; }}
</style>
</head>
<body>
<div id="map"></div>
<div id="info"></div>
<script>
var data   = {data_json};
var center = [{center_lat}, {center_lon}];
var roverColor = {{1: '#ef5350', 2: '#42a5f5'}};

// Restore state from localStorage
var savedCenter = localStorage.getItem('mapCenter');
var savedZoom   = localStorage.getItem('mapZoom');
var autoFit     = localStorage.getItem('autoFit') !== 'false'; // default true

if (savedCenter && !autoFit) {{
  center = JSON.parse(savedCenter);
}}
var zoom = (savedZoom && !autoFit) ? parseInt(savedZoom) : 18;

var map = L.map('map').setView(center, zoom);

// Save state on move/zoom
map.on('moveend', function() {{
  localStorage.setItem('mapCenter', JSON.stringify(map.getCenter()));
}});
map.on('zoomend', function() {{
  localStorage.setItem('mapZoom', map.getZoom());
}});

var googleSat = L.tileLayer(
  'https://{{s}}.google.com/vt/lyrs=s&x={{x}}&y={{y}}&z={{z}}',
  {{maxZoom: 22, subdomains: ['mt0','mt1','mt2','mt3'], attribution: '© Google'}}
).addTo(map);
var osm = L.tileLayer(
  'https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
  {{maxZoom: 19, attribution: '© OpenStreetMap contributors'}}
);
L.control.layers({{'Satellite (Google)': googleSat, 'Street (OSM)': osm}}).addTo(map);

var infoHtml = '<div class="controls">' +
               '<label><input type="checkbox" id="autofit" ' + (autoFit ? 'checked' : '') + '> Auto-fit</label>' +
               '</div>';

data.forEach(function(rv) {{
  var col = roverColor[rv.id] || '#fff';

  // — Mission path (solid, rover colour) with waypoint dots
  if (rv.mission_path && rv.mission_path.length > 1) {{
    L.polyline(rv.mission_path, {{color: col, weight: 3, opacity: 0.95}}).addTo(map);
  }}
  if (rv.mission_path) {{
    rv.mission_path.forEach(function(pt, i) {{
      L.circleMarker(pt, {{radius: 5, color: col, fillColor: col, fillOpacity: 0.85, weight: 1}})
       .bindTooltip('WP' + i).addTo(map);
    }});
  }}

  // — Simulated path (dashed orange)
  if (rv.sim_path && rv.sim_path.length > 1) {{
    L.polyline(rv.sim_path, {{color: '#ff9800', weight: 2, opacity: 0.85,
                              dashArray: '7 5'}}).addTo(map);
  }}

  // — Actual path (green)
  if (rv.actual_path && rv.actual_path.length > 1) {{
    L.polyline(rv.actual_path, {{color: '#66bb6a', weight: 2, opacity: 0.9}}).addTo(map);
  }}

  // — Obstacle polygons (red semi-transparent)
  if (rv.obstacle_polygons && rv.obstacle_polygons.length > 0) {{
    rv.obstacle_polygons.forEach(function(poly) {{
      if (poly.length >= 3) {{
        L.polygon(poly, {{
          color: '#f44336', weight: 2, opacity: 0.9,
          fillColor: '#f44336', fillOpacity: 0.25
        }}).bindTooltip('Obstacle').addTo(map);
      }}
    }});
  }}

  // — Current rover marker
  if (rv.lat !== 0.0) {{
    var fill = rv.mode === 'AUTONOMOUS' ? '#ffee58' : (rv.armed ? '#ff7043' : '#66bb6a');
    L.circleMarker([rv.lat, rv.lon], {{
      radius: 9, color: col, weight: 2, fillColor: fill, fillOpacity: 1.0
    }}).bindTooltip('RV' + rv.id + ' ' + rv.mode,
                    {{permanent: true, direction: 'top', offset: [0, -10]}}).addTo(map);
  }}

  // — Legend row
  infoHtml += '<span class="dot" style="background:' + col + '"></span><b>RV' + rv.id + '</b> &nbsp;';
  if (rv.sim_rms !== null) {{
    var ok = rv.sim_complete ? '&#10003;' : '&#9888;';
    infoHtml += ok + ' Sim XTE  rms=' + rv.sim_rms.toFixed(3) + ' m &nbsp; max='
             + rv.sim_max.toFixed(3) + ' m';
  }} else {{
    infoHtml += '(upload a mission to simulate)';
  }}
  infoHtml += ' &nbsp;&nbsp;&nbsp;';
}});

infoHtml += '<br>';
infoHtml += '<span class="swatch" style="background:#ff9800;"></span>Simulated path &nbsp;';
infoHtml += '<span class="swatch" style="background:#66bb6a;"></span>Actual path &nbsp;';
infoHtml += '<span class="swatch" style="background:#f44336; opacity:0.6;"></span>Obstacle &nbsp;';
infoHtml += '<span style="color:#aaa">&nbsp; Marker: yellow=AUTO, orange=armed, green=disarmed</span>';

document.getElementById('info').innerHTML = infoHtml;

document.getElementById('autofit').onchange = function(e) {{
  localStorage.setItem('autoFit', e.target.checked);
  if (e.target.checked) location.reload();
}};

// Auto-fit map to all visible data points
if (autoFit) {{
  var allPts = [];
  data.forEach(function(rv) {{
    if (rv.lat !== 0.0) allPts.push([rv.lat, rv.lon]);
    (rv.mission_path || []).forEach(function(p) {{ allPts.push(p); }});
    (rv.sim_path     || []).forEach(function(p) {{ allPts.push(p); }});
    // Sample actual_path to avoid fitting to thousands of points
    var ap = rv.actual_path || [];
    if (ap.length > 0) {{ allPts.push(ap[0]); allPts.push(ap[ap.length - 1]); }}
    (rv.obstacle_polygons || []).forEach(function(poly) {{
      poly.forEach(function(v) {{ allPts.push(v); }});
    }});
  }});
  if (allPts.length > 1) {{
    map.fitBounds(allPts, {{padding: [40, 40], maxZoom: 20}});
  }} else if (allPts.length === 1) {{
    map.setView(allPts[0], 18);
  }}
}}

// Refresh page every 5 seconds
setTimeout(function() {{ location.reload(); }}, 5000);
</script>
</body>
</html>"""



def map_gen_loop():
    """Write monitor_map.html every 5 s."""
    while True:
        time.sleep(5)
        try:
            html = _build_map_html()
            with open(_MAP_PATH, 'w', encoding='utf-8') as f:
                f.write(html)
        except Exception:
            pass


class _QuietHandler(SimpleHTTPRequestHandler):
    """Serve tools/ directory; suppress access log noise."""
    def log_message(self, *_):
        pass


def map_http_loop():
    """Serve the tools/ directory over HTTP so the map is accessible from any machine."""
    os.chdir(os.path.dirname(_MAP_PATH))
    server = HTTPServer(('', _MAP_PORT), _QuietHandler)
    server.serve_forever()


# ── MAVLink listener ──────────────────────────────────────────────────────────

def listen():
    """
    Receive MAVLink UDP on :14550 — rovers (sysid 1/2) and GQC (sysid 255).
    Source IP comes directly from recvfrom().
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(2.0)
    sock.bind(('', 14550))

    # One MAVLink parser per rover sysid
    parsers = {sysid: mavutil.mavlink.MAVLink(None) for sysid in RV}
    for p in parsers.values():
        p.robust_parsing = True

    while True:
        try:
            data, (src_ip, _) = sock.recvfrom(512)
        except socket.timeout:
            continue
        except OSError:
            continue

        if not data:
            continue

        # Fast sysid from raw bytes — avoid a full parse just to route
        if data[0] == 0xFD and len(data) >= 6:    # MAVLink v2
            sysid = data[5]
        elif data[0] == 0xFE and len(data) >= 4:  # MAVLink v1
            sysid = data[3]
        else:
            continue

        if sysid not in parsers:
            continue

        msgs = parsers[sysid].parse_buffer(data)
        if not msgs:
            continue

        t_now = time.time()
        with _lock:
            if sysid in RV:
                # ── Rover telemetry ───────────────────────────────────────
                rv = RV[sysid]
                rv.ip = src_ip
                for msg in msgs:
                    t = msg.get_type()
                    if t == 'HEARTBEAT':
                        rv.last_hb  = t_now
                        rv.armed    = bool(msg.base_mode & MAV_MODE_ARMED)
                        rv.mode     = {0: 'MANUAL', 4: 'AUTONOMOUS', 16: 'EMERGENCY'}.get(
                            msg.custom_mode, str(msg.custom_mode))
                    elif t == 'GLOBAL_POSITION_INT':
                        rv.lat = msg.lat / 1e7
                        rv.lon = msg.lon / 1e7
                        if msg.hdg != 65535:
                            rv.heading = msg.hdg / 100.0
                        if rv.lat != 0.0:
                            rv.actual_path.append((rv.lat, rv.lon))
                            if len(rv.actual_path) > 10_000:
                                rv.actual_path.pop(0)
                    elif t == 'RC_CHANNELS':
                        rv.sbus_ch = [getattr(msg, f'chan{i}_raw', 1500) for i in range(1, 17)]
                    elif t == 'SYS_STATUS':
                        rv.battery_v   = msg.voltage_battery / 1000.0
                        rv.battery_pct = msg.battery_remaining
                    elif t == 'NAMED_VALUE_FLOAT':
                        name = (msg.name.rstrip('\x00') if isinstance(msg.name, str)
                                else msg.name.rstrip(b'\x00').decode())
                        if name == 'CMD_T':
                            rv.cmd_thr = int(msg.value)
                        elif name == 'CMD_S':
                            rv.cmd_str = int(msg.value)
                        elif name == 'WP_ACT':
                            rv.wp_active = int(msg.value)
                        elif name == 'WP_TOT':
                            rv.wp_total = int(msg.value)
                        elif name == 'XTE':
                            rv.xte_m = msg.value
                            if msg.value > rv.xte_max:
                                rv.xte_max = msg.value
                            rv.xte_sum   += msg.value
                            rv.xte_count += 1
                        else:
                            rv.sensors[name] = round(msg.value, 1)
                    elif t == 'MISSION_COUNT':
                        # Re-broadcast by mavlink_bridge so we see it here from rover sysid
                        rv.mission_raw       = {}
                        rv.mission_count     = msg.count
                        rv.sim_result        = None
                        rv.sim_running       = False
                        rv.actual_path       = []   # fresh path for this mission
                        rv.mission_fence_buf = []
                        rv.obstacle_polygons = []
                    elif t == 'MISSION_ITEM_INT':
                        # Re-broadcast by mavlink_bridge from rover sysid
                        if msg.command == 16:   # NAV_WAYPOINT only
                            rv.mission_raw[msg.seq] = (msg.x / 1e7, msg.y / 1e7)
                        elif msg.command == 5003:  # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
                            rv.mission_fence_buf.append(
                                (msg.x / 1e7, msg.y / 1e7, int(msg.param1)))
                        # Trigger simulation when last item arrives
                        if rv.mission_count > 0 and msg.seq == rv.mission_count - 1 \
                                and not rv.sim_running:
                            rv.obstacle_polygons = _parse_fence_buf(rv.mission_fence_buf)
                            rv.sim_running = True
                            rv_id_copy = sysid
                            threading.Thread(
                                target=_trigger_simulation,
                                args=(rv_id_copy,),
                                daemon=True,
                            ).start()
                    elif t == 'STATUSTEXT':
                        sev = {0: 'EMERG', 1: 'ALERT', 2: 'CRIT', 3: 'ERR',
                               4: 'WARN',  5: 'NOTE',  6: 'INFO', 7: 'DBG'}.get(msg.severity, '?')
                        rv.log.append(f'[{sev}] {msg.text}')
                        if len(rv.log) > 20:
                            rv.log.pop(0)


# ── Terminal render ───────────────────────────────────────────────────────────

def render():
    while True:
        time.sleep(0.5)
        os.system('cls' if os.name == 'nt' else 'clear')
        now = time.time()
        print('═' * 72)
        print('  AGRIROVER MONITOR')
        print('═' * 72)
        with _lock:
            for rv_id, rv in RV.items():
                age    = now - rv.last_hb
                status = 'ONLINE' if age < 3 else f'LOST ({age:.0f}s)'
                hdg_str = f'{rv.heading:.1f}°' if rv.heading >= 0 else '---.-°'
                print(f'\n  RV{rv_id}  {status}  {"ARMED" if rv.armed else "DISARMED"}  {rv.mode}  ip={rv.ip}')
                print(f'  GPS  {rv.fix_type:10s}  {rv.lat:.6f}, {rv.lon:.6f}  hdg={hdg_str}')
                print(f'  BAT  {rv.battery_v:.2f}V  {rv.battery_pct:.0f}%')
                if rv.mode == 'AUTONOMOUS':
                    thr_delta = rv.cmd_thr - 1500
                    str_delta = rv.cmd_str - 1500
                    thr_bar = '█' * min(10, abs(thr_delta) // 50)
                    str_bar = ('◄' * (max(0, -str_delta) // 50)
                               + '►' * (max(0, str_delta) // 50))
                    wp_str = f'wp={rv.wp_active}/{rv.wp_total - 1}' if rv.wp_total > 0 else 'wp=--'
                    xte_avg = rv.xte_sum / rv.xte_count if rv.xte_count > 0 else 0.0
                    print(f'  CMD  thr={rv.cmd_thr} ({thr_delta:+d}) {thr_bar}'
                          f'   str={rv.cmd_str} ({str_delta:+d}) {str_bar or "─"}'
                          f'   {wp_str}')
                    print(f'  XTE  now={rv.xte_m:.3f}m  max={rv.xte_max:.3f}m'
                          f'  avg={xte_avg:.3f}m  n={rv.xte_count}')

                sbus = rv.sbus_ch
                sbus8 = ' '.join(f'{c:4d}' for c in sbus[:8])
                print(f'  SBUS {sbus8}  sel={sbus[8]:4d}')

                ppm  = _sbus_to_ppm(sbus)
                ppm8 = ' '.join(f'{c:4d}' for c in ppm)
                print(f'  PPM  {ppm8}')
                print(f'       thr  str  SWA  SWB  ch5  ch6  ch7  ch8')

                if rv.sensors:
                    s = rv.sensors
                    print(f'  TANK {s.get("TANK", "?")}%  '
                          f'TEMP {s.get("TEMP", "?")}°C  '
                          f'HUM {s.get("HUMID", "?")}%  '
                          f'PRES {s.get("PRESSURE", "?")}hPa')

                # Simulation XTE summary
                if rv.sim_running:
                    print(f'  SIM  running...')
                elif rv.sim_result is not None:
                    sr = rv.sim_result
                    ok = 'OK' if sr.complete else 'INCOMPLETE'
                    print(f'  SIM  {ok}  rms={sr.rms_xte:.3f}m  max={sr.max_xte:.3f}m'
                          f'  avg={sr.avg_xte:.3f}m  '
                          f'wps={len(sr.waypoints_reached)}/{rv.mission_count}')
                elif rv.mission_count > 0:
                    wps = len(rv.mission_raw)
                    print(f'  SIM  waiting for mission ({wps}/{rv.mission_count} items received)')

                if rv.log:
                    print(f'  LOG  {rv.log[-1]}')

        print('\n' + '─' * 72)
        print(f'  map: http://<this-ip>:{_MAP_PORT}/monitor_map.html  (auto-refresh 5 s)')
        print('  Ctrl+C to exit')


# ── XTE CSV export ────────────────────────────────────────────────────────────

def save_xte_csv():
    """Write per-rover XTE statistics to a timestamped CSV file."""
    ts  = time.strftime('%Y%m%d_%H%M%S')
    out = f'xte_{ts}.csv'
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['rover', 'xte_max_m', 'xte_avg_m', 'samples',
                    'sim_rms_m', 'sim_max_m'])
        with _lock:
            for rv_id, rv in RV.items():
                avg    = rv.xte_sum / rv.xte_count if rv.xte_count > 0 else 0.0
                sr     = rv.sim_result
                w.writerow([f'RV{rv_id}',
                             round(rv.xte_max, 4),
                             round(avg, 4),
                             rv.xte_count,
                             round(sr.rms_xte, 4) if sr else '',
                             round(sr.max_xte, 4) if sr else ''])
    print(f'\nXTE stats saved to {out}')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='AgriRover live monitor')
    ap.add_argument('--map-port', type=int, default=8088,
                    help='HTTP port for map server (default 8088)')
    args = ap.parse_args()
    _MAP_PORT = args.map_port

    threading.Thread(target=listen,        daemon=True).start()
    threading.Thread(target=map_gen_loop,  daemon=True).start()
    threading.Thread(target=map_http_loop, daemon=True).start()
    try:
        render()
    except KeyboardInterrupt:
        pass
    finally:
        save_xte_csv()
