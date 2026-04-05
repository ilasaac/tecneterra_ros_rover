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
from urllib.parse import unquote as _unquote

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_navigator import (  # noqa: E402
    DEFAULT_NAV, DEFAULT_PHYS, SimWaypoint, simulate, compute_pivot_wps,
)

# ── u-blox GPS survey module ─────────────────────────────────────────────────
_gps_serial = None          # serial.Serial instance (or None)
_gps_lock   = _threading.Lock()
_gps_fix: dict = {}         # {lat, lon, alt, fix, sats, hdop, ts, port}
_gps_thread: _threading.Thread | None = None
_gps_stop   = _threading.Event()


def _parse_nmea_coord(raw: str, hemi: str) -> float | None:
    """Convert NMEA ddmm.mmmm(m) string to decimal degrees."""
    if not raw:
        return None
    try:
        dot = raw.index('.')
        deg = int(raw[:dot - 2])
        minutes = float(raw[dot - 2:])
        val = deg + minutes / 60.0
        if hemi in ('S', 'W'):
            val = -val
        return val
    except (ValueError, IndexError):
        return None


def _gps_reader(port: str, baud: int):
    """Background thread: read NMEA from u-blox, update _gps_fix."""
    import serial
    global _gps_serial
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f'[gps] Cannot open {port}: {e}', flush=True)
        with _gps_lock:
            _gps_fix.clear()
            _gps_fix['error'] = str(e)
        return
    with _gps_lock:
        _gps_serial = ser
    print(f'[gps] Opened {port} @ {baud}', flush=True)
    buf = b''
    while not _gps_stop.is_set():
        try:
            chunk = ser.read(max(1, ser.in_waiting or 1))
        except Exception:
            break
        if not chunk:
            continue
        buf += chunk
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            try:
                sentence = line.decode('ascii', errors='ignore').strip()
            except Exception:
                continue
            if sentence.startswith('$') and 'GGA' in sentence[:7]:
                parts = sentence.split(',')
                if len(parts) < 15:
                    continue
                lat = _parse_nmea_coord(parts[2], parts[3])
                lon = _parse_nmea_coord(parts[4], parts[5])
                try:
                    fix = int(parts[6]) if parts[6] else 0
                except ValueError:
                    fix = 0
                try:
                    sats = int(parts[7]) if parts[7] else 0
                except ValueError:
                    sats = 0
                try:
                    hdop = float(parts[8]) if parts[8] else 99.9
                except ValueError:
                    hdop = 99.9
                try:
                    alt = float(parts[9]) if parts[9] else 0.0
                except ValueError:
                    alt = 0.0
                with _gps_lock:
                    _gps_fix.update({
                        'lat': lat, 'lon': lon, 'alt': alt,
                        'fix': fix, 'sats': sats, 'hdop': hdop,
                        'ts': _time.time(), 'port': port,
                    })
                    _gps_fix.pop('error', None)
    ser.close()
    with _gps_lock:
        _gps_serial = None
    print(f'[gps] Closed {port}', flush=True)


def _gps_list_ports() -> list[dict]:
    """List available serial ports on this PC."""
    try:
        from serial.tools.list_ports import comports
        return [{'port': p.device, 'desc': p.description} for p in sorted(comports())]
    except ImportError:
        return []


def _gps_connect(port: str, baud: int = 9600) -> dict:
    global _gps_thread
    _gps_disconnect()
    _gps_stop.clear()
    _gps_thread = _threading.Thread(target=_gps_reader, args=(port, baud), daemon=True)
    _gps_thread.start()
    return {'ok': True, 'port': port, 'baud': baud}


def _gps_disconnect() -> dict:
    global _gps_thread, _gps_serial
    _gps_stop.set()
    with _gps_lock:
        if _gps_serial:
            try:
                _gps_serial.close()
            except Exception:
                pass
            _gps_serial = None
    if _gps_thread and _gps_thread.is_alive():
        _gps_thread.join(timeout=2)
    _gps_thread = None
    with _gps_lock:
        _gps_fix.clear()
    return {'ok': True}

DEFAULT_LAT  = 20.727715
DEFAULT_LON  = -103.566782
DEFAULT_PORT = 8089

# Unique per-launch token — forces browser cache miss on every server restart
_SERVER_VER  = str(int(_time.time()))

# ── Mission file storage ───────────────────────────────────────────────────────
MISSIONS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mission_logs'))
os.makedirs(MISSIONS_DIR, exist_ok=True)

def _save_mission_file(name: str, waypoints: list, obstacles: list,
                       servos: dict | None = None,
                       original_corridors: dict | None = None,
                       optimized_path: list | None = None,
                       real_track: list | None = None):
    path = os.path.join(MISSIONS_DIR, f'{name}.json')
    data = {'waypoints': waypoints, 'obstacles': obstacles, 'servos': servos or {}}
    if original_corridors:
        data['original_corridors'] = original_corridors
    if optimized_path:
        data['optimized_path'] = optimized_path
    if real_track:
        data['real_track'] = real_track
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    return path

def _load_mission_file(name: str) -> dict:
    path = os.path.join(MISSIONS_DIR, f'{name}.json')
    with open(path) as f:
        return json.load(f)

def _list_mission_files() -> list[dict]:
    # List both CSV and JSON mission files
    csv_files = sorted(_glob.glob(os.path.join(MISSIONS_DIR, '*.csv')),
                       key=os.path.getmtime, reverse=True)
    json_files = sorted(_glob.glob(os.path.join(MISSIONS_DIR, '*.json')),
                        key=os.path.getmtime, reverse=True)
    result = []
    for p in csv_files:
        name = os.path.basename(p)
        try:
            with open(p) as f:
                wp_count = sum(1 for line in f) - 1  # minus header
        except Exception:
            wp_count = 0
        result.append({'name': name, 'wp_count': wp_count, 'obs_count': 0,
                       'mtime': int(os.path.getmtime(p))})
    for p in json_files:
        name = os.path.basename(p)
        try:
            d = json.loads(open(p).read())
            wp_count  = len(d.get('waypoints', []))
            obs_count = len(d.get('obstacles', []))
        except Exception:
            wp_count = obs_count = 0
        result.append({'name': name, 'wp_count': wp_count, 'obs_count': obs_count,
                       'mtime': int(os.path.getmtime(p))})
    result.sort(key=lambda x: x['mtime'], reverse=True)
    return result

# ── Mission analyzer ──────────────────────────────────────────────────────────

def _compute_analysis(log_csv: str, mission_name: str) -> dict:
    """Parse navigator_diag.csv + optional mission JSON → per-segment stats."""
    import io as _io
    reader = csv.DictReader(_io.StringIO(log_csv))
    rows = []
    for row in reader:
        try:
            rows.append({
                't':           float(row['t']),
                'lat':         float(row['lat']),
                'lon':         float(row['lon']),
                'heading':     float(row.get('heading') or 0),
                'target_brg':  float(row.get('target_brg') or 0),
                'hdg_err':     float(row.get('hdg_err') or 0),
                'cte':         float(row['cte']),
                'steer_ppm':   int(float(row.get('steer_ppm') or 1500)),
                'throttle_ppm': int(float(row.get('throttle_ppm') or 1500)),
                'wp_idx':      int(row['wp_idx']),
                'speed_tgt':   float(row.get('speed_tgt') or 0),
                'algo':        row.get('algo', ''),
                'fix_quality': int(row.get('fix_quality') or -1),
            })
        except (ValueError, KeyError):
            continue

    if not rows:
        return {'error': 'No valid rows in log (need t,lat,lon,cte,wp_idx columns)'}

    track = [{'lat': r['lat'], 'lon': r['lon'], 'heading': r['heading'],
              'target_brg': r['target_brg'], 'hdg_err': r['hdg_err'],
              'cte': r['cte'], 'steer_ppm': r['steer_ppm'],
              'throttle_ppm': r['throttle_ppm'], 'wp_idx': r['wp_idx'],
              'algo': r['algo']} for r in rows]

    # Per-segment stats grouped by wp_idx
    from itertools import groupby as _groupby
    segments = []
    for wp_idx, grp in _groupby(rows, key=lambda r: r['wp_idx']):
        g = list(grp)
        ctes = [abs(r['cte']) for r in g]
        dur = g[-1]['t'] - g[0]['t'] if len(g) > 1 else 0.0
        rtk_pct = sum(1 for r in g if r['fix_quality'] == 2) / len(g) * 100
        segments.append({
            'wp_idx':    wp_idx,
            'n_points':  len(g),
            'cte_mean':  sum(ctes) / len(ctes),
            'cte_max':   max(ctes),
            'cte_rms':   math.sqrt(sum(c**2 for c in ctes) / len(ctes)),
            'duration_s': round(dur, 2),
            'speed_mean': round(sum(r['speed_tgt'] for r in g) / len(g), 3),
            'rtk_pct':   round(rtk_pct, 1),
        })

    # Overall stats
    all_ctes = [abs(r['cte']) for r in rows]
    overall = {
        'n_points':  len(rows),
        'duration_s': round(rows[-1]['t'] - rows[0]['t'] if len(rows) > 1 else 0, 2),
        'cte_mean':  round(sum(all_ctes) / len(all_ctes), 4),
        'cte_max':   round(max(all_ctes), 4),
        'cte_rms':   round(math.sqrt(sum(c**2 for c in all_ctes) / len(all_ctes)), 4),
        'rtk_pct':   round(sum(1 for r in rows if r['fix_quality'] == 2) / len(rows) * 100, 1),
    }

    mission_wps = []
    if mission_name:
        try:
            mission_wps = _load_mission_file(mission_name).get('waypoints', [])
        except Exception:
            pass

    # Detect algorithm from log (most common 'algo' value)
    algo_detected = ''
    try:
        reader2 = csv.DictReader(_io.StringIO(log_csv))
        algo_counts: dict = {}
        for row in reader2:
            a = row.get('algo', '')
            if a:
                algo_counts[a] = algo_counts.get(a, 0) + 1
        if algo_counts:
            algo_detected = max(algo_counts, key=algo_counts.get)
    except Exception:
        pass

    return {'track': track, 'segments': segments,
            'overall': overall, 'mission_wps': mission_wps,
            'algorithm': algo_detected}


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
                elif t == 'RC_CHANNELS':
                    with _rover_live_lock:
                        live = _rover_live.get(sysid)
                        if live:
                            live['servos'] = {
                                5: msg.chan5_raw, 6: msg.chan6_raw,
                                7: msg.chan7_raw, 8: msg.chan8_raw,
                            }
                elif t == 'MISSION_COUNT':
                    buf[sysid] = {'count': msg.count, 'nav': {}, 'fence': [],
                                  'servos': {}, 'last_nav_seq': None}
                elif t == 'MISSION_ITEM_INT' and sysid in buf:
                    b = buf[sysid]
                    if msg.command == 5003:
                        b['fence'].append((msg.x / 1e7, msg.y / 1e7, int(msg.param1)))
                    elif msg.command == 183:
                        snum = int(msg.param1)
                        if snum in (5, 6, 7, 8):
                            pwm = int(msg.param2)
                            if b['last_nav_seq'] is None:
                                # Initial servo state (before any waypoint)
                                b['servos'][snum] = pwm
                            else:
                                # Per-waypoint servo: attach to the last nav waypoint
                                wp = b['nav'].get(b['last_nav_seq'])
                                if wp:
                                    if 'servos' not in wp:
                                        wp['servos'] = {}
                                    wp['servos'][snum] = pwm
                    elif msg.command == 16:
                        nav_seq = len(b['nav'])
                        if msg.z < 0:
                            print(f'[snoop] TURN MARKER seq={msg.seq} z={msg.z}', flush=True)
                        b['nav'][nav_seq] = {
                            'lat': msg.x / 1e7, 'lon': msg.y / 1e7,
                            'speed': float(msg.z),  # <0 = turn marker, 0 = default, >0 = m/s
                            'hold_secs': float(msg.param1) if msg.param1 else 0.0,
                        }
                        b['last_nav_seq'] = nav_seq
                    if b['count'] > 0 and msg.seq >= b['count'] - 1:
                        wps = [b['nav'][k] for k in sorted(b['nav'])]
                        obs = _parse_fence_buf(b['fence'])
                        with _snooped_lock:
                            _snooped.clear()
                            _snooped.update({'waypoints': wps, 'obstacles': obs,
                                             'servos': b.get('servos', {}),
                                             'rover': sysid, 'ts': _time.time()})
                        print(f'[snoop] RV{sysid}: {len(wps)} wps, '
                              f'{len(obs)} obstacles captured', flush=True)
    t = _threading.Thread(target=_run, daemon=True, name='mavlink-snooper')
    t.start()

# ── MAVLink upload to rover ────────────────────────────────────────────────────
def _mavlink_upload(waypoints: list, obstacles: list,
                    rover_ip: str, rover_port: int, rover_sysid: int,
                    servos: dict | None = None) -> dict:
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
        svs = {int(k): int(v) for k, v in (servos or {}).items()}
        for servo_num in (5, 6, 7, 8):
            pwm = max(1000, min(2000, svs.get(servo_num, 1500)))
            items.append({'cmd': 183, 'p1': float(servo_num), 'p2': float(pwm),
                          'p3': 0.0, 'p4': 0.0, 'lat': 0.0, 'lon': 0.0, 'z': 0.0})
        for wp in waypoints:
            items.append({'cmd': 16, 'p1': 0.0,
                          'p2': float(wp.get('acceptance_radius') or DEFAULT_NAV['default_acceptance_radius']),
                          'p3': 0.0, 'p4': 0.0,
                          'lat': float(wp['lat']), 'lon': float(wp['lon']),
                          'z': float(wp.get('speed') or 0.0)})
            # Per-waypoint servo changes — placed AFTER the NAV_WAYPOINT so mavlink_bridge
            # defers them and applies them when the navigator reaches this waypoint.
            wp_svs = wp.get('servos') or {}
            for servo_num in (5, 6, 7, 8):
                pwm = wp_svs.get(servo_num) or wp_svs.get(str(servo_num))
                if pwm is not None:
                    items.append({'cmd': 183, 'p1': float(servo_num), 'p2': float(int(pwm)),
                                  'p3': 0.0, 'p4': 0.0, 'lat': 0.0, 'lon': 0.0, 'z': 0.0})
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
        def _send_item(seq):
            item = items[seq]
            _send(mav.mission_item_int_encode(
                rover_sysid, 0, seq,
                6,             # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                item['cmd'],
                0, 1,          # current=0, autocontinue=1
                item['p1'], item['p2'], item['p3'], item['p4'],
                int(item['lat'] * 1e7), int(item['lon'] * 1e7), item['z'],
            ))
        _send(mav.mission_count_encode(rover_sysid, 0, len(items)))
        _time.sleep(0.15)   # GQC streaming delay
        for seq in range(len(items)):
            _send_item(seq)
            _time.sleep(0.02)
        # Listen for REQUEST_INT retries (handles packet loss over WiFi)
        sock.settimeout(0.5)
        retry_deadline = _time.monotonic() + 5.0  # wait up to 5s for retries
        while _time.monotonic() < retry_deadline:
            try:
                data, _ = sock.recvfrom(1024)
                msgs = mav.parse_buffer(data)
                for m in (msgs or []):
                    if m.get_type() == 'MISSION_REQUEST_INT':
                        rseq = m.seq
                        if 0 <= rseq < len(items):
                            _send_item(rseq)
                    elif m.get_type() == 'MISSION_ACK':
                        break  # rover confirmed receipt
                else:
                    continue
                break  # got MISSION_ACK
            except OSError:
                pass  # timeout — no retry requested
        sock.close()
        n_wps = len(waypoints)
        n_obs = len(obstacles or [])
        return {'ok': True, 'message': f'Uploaded {len(items)} items ({n_wps} wps + {n_obs} obs) to RV{rover_sysid}'}
    except Exception as e:
        return {'ok': False, 'message': str(e)}

def _mavlink_upload_corridor(corridor_json: str,
                             rover_ip: str, rover_port: int, rover_sysid: int) -> dict:
    """Upload corridor mission to rover using cmd=50100 vertices."""
    os.environ.setdefault('MAVLINK20', '1')
    try:
        from pymavlink.dialects.v20 import ardupilotmega as _mav_def
    except ImportError:
        return {'ok': False, 'message': 'pymavlink not installed'}
    try:
        from corridor import corridor_mission_from_json
        mission = corridor_mission_from_json(corridor_json)
        items = []
        for c in mission.corridors:
            n_verts = len(c.centerline)
            for lat, lon in c.centerline:
                items.append({
                    'cmd': 50100,
                    'p1': float(n_verts),          # vertex_count
                    'p2': float(c.width),           # corridor half-width
                    'p3': float(c.speed),           # speed
                    'p4': float(c.next_corridor_id if c.next_corridor_id >= 0 else 65535),
                    'lat': lat,
                    'lon': lon,
                    'z': float(c.corridor_id),      # corridor_id
                })
        if not items:
            return {'ok': False, 'message': 'No corridor vertices'}
    except Exception as e:
        return {'ok': False, 'message': f'Build error: {e}'}
    try:
        sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        mav  = _mav_def.MAVLink(None)
        mav.srcSystem    = 255
        mav.srcComponent = 0
        def _send(pkt):
            sock.sendto(pkt.pack(mav), (rover_ip, rover_port))
        # DISARM first (same as regular upload)
        for _ in range(3):
            _send(mav.command_long_encode(rover_sysid, 0, 400, 0, 0, 0, 0, 0, 0, 0, 0))
            _time.sleep(0.1)
        def _send_item(seq):
            item = items[seq]
            _send(mav.mission_item_int_encode(
                rover_sysid, 0, seq,
                6, item['cmd'], 0, 1,
                item['p1'], item['p2'], item['p3'], item['p4'],
                int(item['lat'] * 1e7), int(item['lon'] * 1e7), item['z'],
            ))
        _send(mav.mission_count_encode(rover_sysid, 0, len(items)))
        _time.sleep(0.15)
        for seq in range(len(items)):
            _send_item(seq)
            _time.sleep(0.02)
        # Listen for REQUEST_INT retries (handles packet loss over WiFi)
        sock.settimeout(0.5)
        retry_deadline = _time.monotonic() + 5.0
        while _time.monotonic() < retry_deadline:
            try:
                data, _ = sock.recvfrom(1024)
                msgs = mav.parse_buffer(data)
                for m in (msgs or []):
                    if m.get_type() == 'MISSION_REQUEST_INT':
                        rseq = m.seq
                        if 0 <= rseq < len(items):
                            _send_item(rseq)
                    elif m.get_type() == 'MISSION_ACK':
                        break
                else:
                    continue
                break
            except OSError:
                pass
        sock.close()
        return {'ok': True, 'message': f'Uploaded {len(items)} corridor vertices ({len(mission.corridors)} corridors) to RV{rover_sysid}'}
    except Exception as e:
        return {'ok': False, 'message': str(e)}


# ── MAVLink test sensor injection ─────────────────────────────────────────────
def _mavlink_send_test_sensors(rover_ip: str, rover_port: int, rover_sysid: int,
                                tank_level: float, batt_pct: float) -> dict:
    """Send PARAM_SET TANK_LEVEL + BATT_PCT to mavlink_bridge for test injection."""
    os.environ.setdefault('MAVLINK20', '1')
    try:
        from pymavlink.dialects.v20 import ardupilotmega as _mav_def
    except ImportError:
        return {'ok': False, 'message': 'pymavlink not installed'}
    try:
        sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        mav  = _mav_def.MAVLink(None)
        mav.srcSystem    = 255
        mav.srcComponent = 0
        MAV_PARAM_TYPE_REAL32 = 9
        def _send(pkt):
            sock.sendto(pkt.pack(mav), (rover_ip, rover_port))
        _send(mav.param_set_encode(
            rover_sysid, 0, b'TANK_LEVEL',
            float(tank_level), MAV_PARAM_TYPE_REAL32))
        _time.sleep(0.05)
        _send(mav.param_set_encode(
            rover_sysid, 0, b'BATT_PCT',
            float(batt_pct), MAV_PARAM_TYPE_REAL32))
        sock.close()
        return {'ok': True,
                'message': f'Test sensors → RV{rover_sysid}: tank={tank_level:.0f}%  batt={batt_pct:.0f}%'}
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
#analyze-panel{flex-direction:column}
#az-seg-table{display:none}
#az-overall{display:none;padding:5px 8px;font-size:11px;color:#ccc;background:#0d0d1a;border-bottom:1px solid #222;line-height:1.7;font-family:monospace}
.az-legend{padding:3px 8px;font-size:10px;color:#aaa;background:#0d0d1a;border-bottom:1px solid #222;display:flex;gap:8px;align-items:center}
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
    <!-- analyze is now a permanent section, no toggle button -->
    <input type="file" id="file-import" accept=".csv" style="display:none" onchange="importCSV(event)">
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
      <button class="btn-orange" style="padding:3px 9px;font-size:11px" onclick="downloadRover(1)">&#9660; RV1</button>
    </div>
    <div style="display:flex;gap:3px;align-items:center;margin-bottom:3px">
      <label style="color:#b7a">&#128225;</label>
      <input id="r-ip-rv2" size="13" value="192.168.100.20" style="background:#0a1020;color:#eee;border:1px solid #446;padding:2px 4px;border-radius:2px;flex:1">
      <button class="btn-blue" style="padding:3px 9px;font-size:11px" onclick="uploadRover(2)">&#9650; RV2</button>
      <button class="btn-orange" style="padding:3px 9px;font-size:11px" onclick="downloadRover(2)">&#9660; RV2</button>
    </div>
    <!-- Net import removed — use Analyze section Runs/Compare instead -->
  </div>
  <div style="padding:3px 6px;background:#0d0d1a;border-bottom:1px solid #333;display:flex;align-items:center;gap:4px;flex-wrap:wrap">
    <span style="color:#888;font-size:10px">All spd:</span>
    <input id="bulk-speed" type="number" value="1.0" min="0" max="1.5" step="0.1"
           style="width:42px;background:#0a1020;color:#eee;border:1px solid #446;padding:2px 3px;border-radius:2px;font-size:11px">
    <button onclick="applyBulkSpeed()" style="padding:2px 5px;font-size:10px;background:#0f3460;color:#fff;border:none;border-radius:2px;cursor:pointer">&#10003;</button>
    <span style="color:#888;font-size:9px;margin-left:3px">5:</span>
    <input id="bulk-ch5" type="number" value="1500" min="1000" max="2000" step="50"
           style="width:38px;background:#0a1020;color:#eee;border:1px solid #446;padding:2px 1px;border-radius:2px;font-size:10px">
    <span style="color:#888;font-size:9px">6:</span>
    <input id="bulk-ch6" type="number" value="1500" min="1000" max="2000" step="50"
           style="width:38px;background:#0a1020;color:#eee;border:1px solid #446;padding:2px 1px;border-radius:2px;font-size:10px">
    <span style="color:#888;font-size:9px">7:</span>
    <input id="bulk-ch7" type="number" value="1500" min="1000" max="2000" step="50"
           style="width:38px;background:#0a1020;color:#eee;border:1px solid #446;padding:2px 1px;border-radius:2px;font-size:10px">
    <span style="color:#888;font-size:9px">8:</span>
    <input id="bulk-ch8" type="number" value="1500" min="1000" max="2000" step="50"
           style="width:38px;background:#0a1020;color:#eee;border:1px solid #446;padding:2px 1px;border-radius:2px;font-size:10px">
    <button onclick="applyBulkServo()" style="padding:2px 5px;font-size:10px;background:#0f3460;color:#fff;border:none;border-radius:2px;cursor:pointer" title="Set CH5-CH8 for all points">Srv&#10003;</button>
  </div>
  <div id="wp-list" style="flex:1;overflow-y:auto;min-height:60px;max-height:35vh">
    <table>
      <thead><tr><th style="width:18px">#</th><th style="width:28px">Spd</th><th style="width:12px">T</th><th style="width:30px">5</th><th style="width:30px">6</th><th style="width:30px">7</th><th style="width:30px">8</th></tr></thead>
      <tbody id="wp-tbody"></tbody>
    </table>
  </div>
  <!-- ── Analyze section (permanent) ── -->
  <div id="analyze-panel" style="display:flex;flex-direction:column;max-height:30vh;overflow-y:auto">
    <div style="padding:4px 8px;background:#0a1a3a;border-top:2px solid #1a5aaa;border-bottom:1px solid #333">
      <b style="color:#8cf;font-size:12px">ANALYSIS</b>
    </div>
    <div class="section" style="background:#0d1a2e">
      <div style="display:flex;gap:3px;align-items:center;margin-bottom:4px">
        <span style="color:#8cf;font-size:10px">Rover</span>
        <input id="az-rover-ip" size="13" value="192.168.100.19"
               style="background:#0a1020;color:#eee;border:1px solid #446;padding:2px 4px;border-radius:2px;font-size:11px;flex:1">
        <button class="btn-orange" style="padding:2px 8px;font-size:11px" onclick="listRoverRuns()" title="List available runs from rover">&#128269; Runs</button>
      </div>
      <div style="display:flex;gap:3px;align-items:center;margin-bottom:4px">
        <span style="color:#8cf;font-size:10px">user</span>
        <input id="az-ssh-user" value="ilasa1" size="5"
               style="background:#0a1020;color:#eee;border:1px solid #446;padding:2px 3px;border-radius:2px;font-size:11px;width:50px">
        <span style="color:#8cf;font-size:10px">key</span>
        <input id="az-ssh-key" value="~/.ssh/agri_rover" size="14"
               style="background:#0a1020;color:#eee;border:1px solid #446;padding:2px 3px;border-radius:2px;font-size:11px;flex:1">
      </div>
      <div style="display:flex;gap:3px;align-items:center;margin-bottom:4px">
        <select id="az-run-select" style="flex:1;background:#0a1020;color:#eee;border:1px solid #446;padding:2px;border-radius:2px;font-size:11px">
          <option value="">— select a run —</option>
        </select>
        <button class="btn-green" style="padding:2px 9px;font-size:11px" onclick="fetchAndCompare()" title="Fetch run data + simulate + compare">&#8644; Compare</button>
        <button class="btn-blue" style="padding:2px 7px;font-size:11px" onclick="runSimOnRover()" title="Upload mission + run sim on rover + fetch results">&#9654; Sim</button>
      </div>
      <div style="display:flex;gap:3px;align-items:center;margin-bottom:4px">
        <span style="color:#888;font-size:10px">or:</span>
        <button class="btn-blue" style="padding:2px 7px;font-size:11px" onclick="document.getElementById('az-log-input').click()">&#128194; Browse CSV</button>
        <button id="btn-save-log" class="btn-green" style="padding:2px 7px;font-size:11px;display:none" onclick="saveLog()" title="Save loaded log to disk">&#128190; Save</button>
        <span id="az-log-name" style="font-size:10px;color:#888;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;flex:1">no file</span>
        <input type="file" id="az-log-input" accept=".csv" style="display:none" onchange="onLogFileSelect(event)">
      </div>
      <div style="display:flex;gap:3px;align-items:center">
        <select id="az-m-select" style="flex:1;background:#0a1020;color:#eee;border:1px solid #446;padding:2px;border-radius:2px;font-size:11px">
          <option value="">— mission (manual) —</option>
        </select>
        <button class="btn-green" style="padding:2px 9px;font-size:11px" onclick="runAnalysis()">&#9654; Run</button>
        <button class="btn-orange" style="padding:2px 9px;font-size:11px" onclick="runComparison()" title="Analyze + Simulate with same mission">&#8644; Compare</button>
      </div>
    </div>
    <div class="az-legend">
      <span>Real:</span>
      <span style="color:#00e676">&#9632;&nbsp;&lt;0.10m</span>
      <span style="color:#ffee00">&#9632;&nbsp;&lt;0.20m</span>
      <span style="color:#ff9100">&#9632;&nbsp;&lt;0.40m</span>
      <span style="color:#ff1744">&#9632;&nbsp;&ge;0.40m</span>
      <span style="margin-left:6px;color:#4af">|</span>
      <span style="color:#4af">&#9473; Sim</span>
    </div>
    <div style="padding:3px 8px;background:#0d0d1a;border-bottom:1px solid #222;display:flex;gap:8px;align-items:center;flex-wrap:wrap;font-size:10px">
      <span style="color:#888">Layers:</span>
      <label style="color:#fff"><input type="checkbox" id="layer-raw" checked onchange="redraw()" style="width:auto;vertical-align:middle"> Raw</label>
      <label style="color:#2c6"><input type="checkbox" id="layer-opt" checked onchange="redraw()" style="width:auto;vertical-align:middle"> Optimized</label>
      <label style="color:#0e6"><input type="checkbox" id="layer-real" checked onchange="redraw()" style="width:auto;vertical-align:middle"> Real</label>
      <label style="color:#4af"><input type="checkbox" id="layer-sim" checked onchange="redraw()" style="width:auto;vertical-align:middle"> Sim</label>
    </div>
    <div id="az-overall"></div>
    <div style="max-height:150px;overflow-y:auto">
      <table id="az-seg-table">
        <thead><tr><th>Seg</th><th>Mean CTE</th><th>Max CTE</th><th>RMS CTE</th><th>Dur(s)</th><th>RTK%</th></tr></thead>
        <tbody id="az-seg-tbody"></tbody>
      </table>
    </div>
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
  <!-- ── GPS Survey ── -->
  <div>
    <div style="padding:4px 8px;background:#0a2a1a;border-top:2px solid #2a8a4a;border-bottom:1px solid #333">
      <b style="color:#6f8;font-size:12px">GPS SURVEY</b>
    </div>
    <div class="section" style="background:#0a1a0a">
      <div style="display:flex;gap:3px;align-items:center;margin-bottom:4px">
        <select id="gps-port" style="flex:1;background:#0a1020;color:#eee;border:1px solid #446;padding:2px;border-radius:2px;font-size:11px">
          <option value="">-- select port --</option>
        </select>
        <button class="btn-blue" style="padding:2px 6px;font-size:10px" onclick="gpsRefreshPorts()" title="Refresh port list">&#8635;</button>
        <button id="gps-conn-btn" class="btn-green" style="padding:2px 8px;font-size:10px" onclick="gpsToggleConnect()">Connect</button>
      </div>
      <div id="gps-fix-info" style="font-size:10px;color:#888;margin-bottom:4px;font-family:monospace;line-height:1.5">
        Not connected
      </div>
      <div style="display:flex;gap:3px;align-items:center;margin-bottom:4px">
        <span style="color:#aaa;font-size:10px">Mode:</span>
        <select id="gps-survey-mode" style="flex:1;background:#0a1020;color:#eee;border:1px solid #446;padding:2px;border-radius:2px;font-size:11px">
          <option value="perimeter">Perimeter (fence)</option>
          <option value="obstacle">Obstacle</option>
        </select>
      </div>
      <div style="display:flex;gap:3px;align-items:center;margin-bottom:4px">
        <span style="color:#aaa;font-size:10px">Avg:</span>
        <input id="gps-avg-count" type="number" value="5" min="1" max="60"
               style="width:36px;background:#0a1020;color:#eee;border:1px solid #446;padding:1px 3px;border-radius:2px;font-size:10px"
               title="Number of fixes to average per capture">
        <span style="color:#888;font-size:9px">fixes</span>
        <span style="color:#aaa;font-size:10px;margin-left:4px">Wall:</span>
        <input id="gps-wall-width" type="number" value="3.0" min="0.5" max="20" step="0.5"
               style="width:36px;background:#0a1020;color:#eee;border:1px solid #446;padding:1px 3px;border-radius:2px;font-size:10px"
               title="Perimeter wall strip width in metres (outward from fence)">
        <span style="color:#888;font-size:9px">m</span>
      </div>
      <div style="display:flex;gap:3px;align-items:center;margin-bottom:4px">
        <button id="gps-capture-btn" class="btn-green" style="flex:1;padding:4px;font-size:11px;font-weight:bold" onclick="gpsCapture()" disabled>&#9678; Capture Point</button>
      </div>
      <div style="display:flex;gap:3px;align-items:center">
        <button id="gps-close-btn" class="btn-orange" style="flex:1;padding:3px;font-size:10px" onclick="gpsClosePoly()" disabled>Close Polygon</button>
        <button class="btn-red" style="padding:3px 8px;font-size:10px" onclick="gpsDiscardPoly()">Discard</button>
      </div>
      <div id="gps-poly-info" style="font-size:10px;color:#888;margin-top:3px"></div>
    </div>
    <!-- ── Sensor Control ── -->
    <div style="padding:4px 8px;background:#2a0a0a;border-top:2px solid #aa3a3a;border-bottom:1px solid #333">
      <b style="color:#f88;font-size:12px">SENSOR CONTROL</b>
    </div>
    <div class="section" style="background:#1a0808">
      <div style="display:grid;grid-template-columns:auto 1fr;gap:2px 6px;align-items:center">
        <span style="color:#aaa;font-size:10px">Tank (%)</span>
        <input id="ts-tank" type="number" min="0" max="100" step="1" value="80"
               style="background:#0a0a1e;color:#eee;border:1px solid #444;padding:1px 3px;border-radius:2px;font-size:10px">
        <span style="color:#aaa;font-size:10px">Battery (%)</span>
        <input id="ts-batt" type="number" min="0" max="100" step="1" value="100"
               style="background:#0a0a1e;color:#eee;border:1px solid #444;padding:1px 3px;border-radius:2px;font-size:10px">
      </div>
      <div style="display:flex;gap:3px;margin-top:4px">
        <button class="btn-orange" style="flex:1;padding:3px;font-size:10px" onclick="sendTestSensors(1)">&#9650; RV1</button>
        <button class="btn-blue"   style="flex:1;padding:3px;font-size:10px" onclick="sendTestSensors(2)">&#9650; RV2</button>
      </div>
    </div>
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
    <option value="corridor">Corridor Grid — rows with arc/spin turns</option>
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
  <div id="gen-corridor-fields" style="display:none">
    <label>Rows</label><input id="gen-c-rows" type="number" value="4" min="2" max="30">
    <label>Row length (m)</label><input id="gen-c-len" type="number" value="20" min="5">
    <label>Row spacing (m)</label><input id="gen-c-sp" type="number" value="3" min="1" step="0.5">
    <label>Corridor width (m)</label><input id="gen-c-w" type="number" value="1.0" min="0.3" max="3" step="0.1">
    <label>Min turn radius (m)</label><input id="gen-c-rad" type="number" value="2.0" min="0.5" step="0.5">
    <label>Heading &deg;</label><input id="gen-c-hdg" type="number" value="0" min="0" max="359">
    <label>Turn type</label>
    <select id="gen-c-turn">
      <option value="auto">Auto (spin + cross)</option>
      <option value="arc">Arc (smooth curve)</option>
      <option value="spin">Spin only (no crossing)</option>
    </select>
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
let waypoints     = [];
let addMode       = false;
let simResult     = null;
let obstacles     = [];
let obsMode       = false;
let obsCurPts     = [];
let liveRovers    = {};   // sysid → {lat, lon, hdg, ts}
let analyzeResult = null;
let logFileData   = null;
let fetchedMission = null;  // mission metadata from rover run (corridor_mode, algorithm)
let originalCorridors = null;  // raw corridor vertices from rover (with speed markers)
let optimizedPath = null;      // optimized path from rover (what it actually follows)
let simPreflight = null;       // preflight sim result from navigator (sim_preflight.json)
let simDiagCsv = null;         // sim_diag.csv — same format as navigator_diag.csv

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
  if (analyzeResult) {
    (analyzeResult.track || []).forEach(p => pts.push([p.lat, p.lon]));
    (analyzeResult.mission_wps || []).forEach(w => pts.push([w.lat, w.lon]));
  }
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
  // Layer toggles
  const showRaw  = document.getElementById('layer-raw')?.checked ?? true;
  const showOpt  = document.getElementById('layer-opt')?.checked ?? true;
  const showReal = document.getElementById('layer-real')?.checked ?? true;
  const showSim  = document.getElementById('layer-sim')?.checked ?? true;
  if (showRaw)  drawRawCorridors();
  if (showOpt)  drawOptimizedPath();
  if (showSim)  { drawSimDiagTrack(); drawRoute(); drawSimPath(); drawPivotMarkers(); drawWaypoints(); drawStartMarker(); }
  drawRover();
  if (showReal) drawAnalyzeTrack();
  drawLiveRovers();
  drawGpsSurvey();
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
  const comparing = !!analyzeResult;
  const xteMax = Math.max(...xteLog, 0.001);
  ctx.lineWidth = comparing ? 2 : 3;
  if (comparing) { ctx.save(); ctx.setLineDash([6, 4]); }
  for (let i = 0; i + 1 < path.length; i++) {
    const p1 = project(path[i][0],   path[i][1]);
    const p2 = project(path[i+1][0], path[i+1][1]);
    if (comparing) {
      ctx.strokeStyle = '#44aaff';
    } else {
      const t = (xteLog[i] || 0) / xteMax;
      const r = Math.round(255 * Math.min(t * 2, 1));
      const g = Math.round(255 * Math.max(1 - t * 2 + 1, 0));
      ctx.strokeStyle = `rgb(${r},${g},30)`;
    }
    ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
  }
  if (comparing) ctx.restore();
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
  // Check optimizedPath first (what's shown in table and on map)
  if (optimizedPath && optimizedPath.length) {
    for (let i = optimizedPath.length - 1; i >= 0; i--) {
      const p = project(optimizedPath[i].lat, optimizedPath[i].lon);
      if (Math.hypot(p.x - x, p.y - y) <= 13) return i;
    }
    return -1;
  }
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
    if (optimizedPath && optimizedPath.length) {
      optimizedPath[_drag.idx].lat = ll.lat;
      optimizedPath[_drag.idx].lon = ll.lon;
    } else {
      waypoints[_drag.idx].lat = ll.lat;
      waypoints[_drag.idx].lon = ll.lon;
    }
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

// ── Servo state ───────────────────────────────────────────────────
function getServos() {
  const r = {};
  [5,6,7,8].forEach(ch => {
    const el = document.getElementById(`svo-${ch}`);
    const v = parseInt(el ? el.value : 1500);
    r[String(ch)] = isNaN(v) ? 1500 : Math.max(1000, Math.min(2000, v));
  });
  return r;
}

function setServos(s) {
  if (!s) return;
  [5,6,7,8].forEach(ch => {
    const val = s[ch] ?? s[String(ch)];
    if (val != null) {
      const el = document.getElementById(`svo-${ch}`);
      if (el) el.value = val;
    }
  });
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
  waypoints.push({lat, lon, speed, hold_secs: hold, servos: {}});
  refresh();
}

function removeWp(i) { waypoints.splice(i, 1); refresh(); }

function refresh() {
  refreshTable();
  redraw();
  status(`${waypoints.length} waypoint(s).`);
}

function _wpHasServos(wp) {
  const s = wp.servos || {};
  return [5,6,7,8].some(ch => s[ch] != null || s[String(ch)] != null);
}

function refreshTable() {
  const tb = document.getElementById('wp-tbody');
  tb.innerHTML = '';
  // Servo data — from optimized_path (per-point full state) or original_corridors

  // Show optimized path from fetched run — cells are editable
  if (optimizedPath && optimizedPath.length) {
    optimizedPath.forEach((pt, i) => {
      const tr = document.createElement('tr');
      const isTurn = pt.turn === true;
      const spdColor = pt.speed < 0 ? '#ffeb3b' : pt.speed <= 0.5 ? '#ff9800' : '#ccc';
      const tl = isTurn
        ? `<span style="color:#f52;cursor:pointer" onclick="toggleTurn(${i})">T</span>`
        : `<span style="color:#333;cursor:pointer" onclick="toggleTurn(${i})">·</span>`;
      const s = pt.servo || {};
      const c5 = s[5]||s['5']||''; const c6 = s[6]||s['6']||'';
      const c7 = s[7]||s['7']||''; const c8 = s[8]||s['8']||'';
      const ss = 'font-size:8px;cursor:pointer;color:#5a8;padding:0 1px';
      tr.innerHTML = `
        <td style="color:#666;font-size:9px;padding:0 2px">${i}</td>
        <td style="color:${spdColor};cursor:pointer;padding:0 2px" onclick="editCell(this,${i},'speed')">${pt.speed.toFixed(1)}</td>
        <td style="padding:0 1px">${tl}</td>
        <td style="${ss}" onclick="editCell(this,${i},'ch5')">${c5||'-'}</td>
        <td style="${ss}" onclick="editCell(this,${i},'ch6')">${c6||'-'}</td>
        <td style="${ss}" onclick="editCell(this,${i},'ch7')">${c7||'-'}</td>
        <td style="${ss}" onclick="editCell(this,${i},'ch8')">${c8||'-'}</td>`;
      if (isTurn) tr.style.background = '#2a1a0a';
      tb.appendChild(tr);
    });
    return;
  }
  // Fallback: show raw corridor speeds
  if (originalCorridors) {
    const corrs = originalCorridors.corridors || [];
    let gi = 0;
    corrs.forEach(c => {
      (c.speeds || []).forEach((spd, i) => {
        const tr = document.createElement('tr');
        const isTurn = spd < 0;
        const spdColor = isTurn ? '#ffeb3b' : '#ccc';
        const srvArr = c.servos || [];
        const srv = srvArr[i];
        const srvText = srv ? Object.entries(srv).map(([ch,pw]) => `${ch}:${pw}`).join(' ') : '';
        tr.innerHTML = `
          <td style="color:#aaa">${gi}</td>
          <td style="color:${spdColor}">${spd.toFixed(1)}</td>
          <td>${isTurn ? '<span style="color:#ffeb3b">T</span>' : ''}</td>
          <td style="color:#5a8;font-size:9px">${srvText}</td>`;
        if (isTurn) tr.style.background = '#2a2a0a';
        if (srvText) tr.style.background = '#0a1a2a';
        tb.appendChild(tr);
        gi++;
      });
    });
  }
}

function toggleWpServo(i) {
  const row = document.getElementById(`svo-row-${i}`);
  if (row) row.style.display = row.style.display === 'none' ? '' : 'none';
}

function wpServoSet(i, ch, val) {
  if (!waypoints[i].servos) waypoints[i].servos = {};
  const v = val === '' ? null : Math.max(1000, Math.min(2000, parseInt(val)));
  if (v === null || isNaN(v)) {
    delete waypoints[i].servos[ch];
    delete waypoints[i].servos[String(ch)];
  } else {
    waypoints[i].servos[String(ch)] = v;
  }
  const hasSrv = _wpHasServos(waypoints[i]);
  const btn = document.getElementById(`svo-btn-${i}`);
  if (btn) {
    btn.style.background = hasSrv ? '#1a7a3a' : '#1a1a3a';
    btn.style.color      = hasSrv ? '#5d9'    : '#556';
    btn.style.border     = hasSrv ? '1px solid #2a9a4a' : '1px solid #334';
  }
}

function wpServoClear(i) {
  waypoints[i].servos = {};
  refreshTable();
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

// ── GPS Survey ───────────────────────────────────────────────────
let _gpsConnected = false;
let _gpsFix = null;           // {lat, lon, fix, sats, hdop, ts}
let _gpsSurveyPts = [];       // current polygon being surveyed
let _gpsAvgBuf = [];          // accumulator for averaging
let _gpsAvgTarget = 0;        // how many fixes to collect
let _gpsCapturing = false;    // currently averaging?
let _gpsPerimeter = null;     // [[lat,lon], ...] — single perimeter polygon
let _gpsPollTimer = null;

async function gpsRefreshPorts() {
  const resp = await fetch('/gps_list_ports');
  const ports = await resp.json();
  const sel = document.getElementById('gps-port');
  const prev = sel.value;
  sel.innerHTML = '<option value="">-- select port --</option>';
  ports.forEach(p => {
    const opt = document.createElement('option');
    opt.value = p.port;
    opt.textContent = `${p.port} — ${p.desc}`;
    sel.appendChild(opt);
  });
  if (prev) sel.value = prev;
  status(`Found ${ports.length} serial port(s).`);
}

async function gpsToggleConnect() {
  if (_gpsConnected) {
    await fetch('/gps_disconnect', {method:'POST', headers:{'Content-Type':'application/json'}, body:'{}'});
    _gpsConnected = false;
    _gpsFix = null;
    if (_gpsPollTimer) { clearInterval(_gpsPollTimer); _gpsPollTimer = null; }
    document.getElementById('gps-conn-btn').textContent = 'Connect';
    document.getElementById('gps-conn-btn').className = 'btn-green';
    document.getElementById('gps-fix-info').textContent = 'Not connected';
    document.getElementById('gps-capture-btn').disabled = true;
    redraw();
    return;
  }
  const port = document.getElementById('gps-port').value;
  if (!port) { status('Select a serial port first.', '#e74c3c'); return; }
  const resp = await fetch('/gps_connect', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({port, baud: 9600}),
  });
  const r = await resp.json();
  if (r.error) { status(`GPS: ${r.error}`, '#e74c3c'); return; }
  _gpsConnected = true;
  document.getElementById('gps-conn-btn').textContent = 'Disconnect';
  document.getElementById('gps-conn-btn').className = 'btn-red';
  _gpsPollTimer = setInterval(gpsPollStatus, 500);
  status(`GPS connected on ${port}.`);
}

async function gpsPollStatus() {
  try {
    const resp = await fetch('/gps_status');
    const d = await resp.json();
    if (!d.connected) {
      _gpsConnected = false;
      if (_gpsPollTimer) { clearInterval(_gpsPollTimer); _gpsPollTimer = null; }
      document.getElementById('gps-conn-btn').textContent = 'Connect';
      document.getElementById('gps-conn-btn').className = 'btn-green';
      document.getElementById('gps-fix-info').textContent = d.error ? `Error: ${d.error}` : 'Disconnected';
      document.getElementById('gps-capture-btn').disabled = true;
      _gpsFix = null;
      redraw();
      return;
    }
    if (d.error) {
      document.getElementById('gps-fix-info').innerHTML = `<span style="color:#f44">${d.error}</span>`;
      _gpsFix = null;
      document.getElementById('gps-capture-btn').disabled = true;
      redraw();
      return;
    }
    _gpsFix = d;
    const fixNames = {0:'No fix', 1:'GPS', 2:'DGPS', 4:'RTK Fix', 5:'RTK Float', 6:'Dead reck'};
    const fixName = fixNames[d.fix] || `Fix ${d.fix}`;
    const fixColor = d.fix >= 4 ? '#0f0' : d.fix >= 1 ? '#ff0' : '#f44';
    const age = d.ts ? Math.max(0, (Date.now()/1000 - d.ts)).toFixed(0) : '?';
    const capBtn = document.getElementById('gps-capture-btn');
    capBtn.disabled = !d.lat || d.fix < 1;
    let info = `<span style="color:${fixColor}">${fixName}</span> Sats:${d.sats} HDOP:${(d.hdop||0).toFixed(1)}`;
    if (d.lat) info += `\n${d.lat.toFixed(8)}, ${d.lon.toFixed(8)}`;
    if (_gpsCapturing) info += `\n<span style="color:#ff0">Averaging: ${_gpsAvgBuf.length}/${_gpsAvgTarget}</span>`;
    document.getElementById('gps-fix-info').innerHTML = info.replace(/\n/g, '<br>');
    // Accumulate for averaging if capturing
    if (_gpsCapturing && d.lat && d.fix >= 1) {
      _gpsAvgBuf.push([d.lat, d.lon]);
      if (_gpsAvgBuf.length >= _gpsAvgTarget) {
        _gpsFinishCapture();
      }
    }
    redraw();
  } catch(e) {}
}

function gpsCapture() {
  if (!_gpsFix || !_gpsFix.lat) return;
  _gpsAvgTarget = parseInt(document.getElementById('gps-avg-count').value) || 5;
  if (_gpsAvgTarget <= 1) {
    // Instant capture, no averaging
    _gpsSurveyPts.push([_gpsFix.lat, _gpsFix.lon]);
    _gpsUpdatePolyInfo();
    redraw();
    status(`GPS vertex ${_gpsSurveyPts.length} captured.`);
    return;
  }
  _gpsAvgBuf = [];
  _gpsCapturing = true;
  document.getElementById('gps-capture-btn').disabled = true;
  document.getElementById('gps-capture-btn').textContent = 'Averaging...';
  status(`Collecting ${_gpsAvgTarget} fixes for average...`);
}

function _gpsFinishCapture() {
  _gpsCapturing = false;
  document.getElementById('gps-capture-btn').disabled = false;
  document.getElementById('gps-capture-btn').innerHTML = '&#9678; Capture Point';
  if (!_gpsAvgBuf.length) return;
  const avgLat = _gpsAvgBuf.reduce((s, p) => s + p[0], 0) / _gpsAvgBuf.length;
  const avgLon = _gpsAvgBuf.reduce((s, p) => s + p[1], 0) / _gpsAvgBuf.length;
  _gpsSurveyPts.push([avgLat, avgLon]);
  _gpsUpdatePolyInfo();
  redraw();
  status(`GPS vertex ${_gpsSurveyPts.length} captured (avg of ${_gpsAvgBuf.length}).`);
}

function _perimeterToWalls(verts, wallWidth) {
  // Convert a closed perimeter polygon into thin wall strips along each edge,
  // offset outward. The rover stays inside; walls block exit.
  // wallWidth in metres.
  const walls = [];
  const n = verts.length;
  const DEG_PER_M = 1.0 / 111320;
  // Compute polygon winding (positive = CCW in lat/lon)
  let area2 = 0;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    area2 += (verts[j][1] - verts[i][1]) * (verts[j][0] + verts[i][0]);
  }
  const ccw = area2 > 0;  // if true, outward normal is to the right of edge direction
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    const lat0 = verts[i][0], lon0 = verts[i][1];
    const lat1 = verts[j][0], lon1 = verts[j][1];
    const cosLat = Math.cos(((lat0 + lat1) / 2) * Math.PI / 180);
    // Edge vector in metres
    const dx = (lon1 - lon0) * 111320 * cosLat;
    const dy = (lat1 - lat0) * 111320;
    const edgeLen = Math.sqrt(dx * dx + dy * dy);
    if (edgeLen < 0.01) continue;
    // Outward normal (perpendicular, pointing outside polygon)
    let nx, ny;
    if (ccw) { nx = dy / edgeLen; ny = -dx / edgeLen; }
    else     { nx = -dy / edgeLen; ny = dx / edgeLen; }
    // Offset in degrees
    const offLat = ny * wallWidth * DEG_PER_M;
    const offLon = nx * wallWidth * DEG_PER_M / cosLat;
    // Wall = thin rectangle: inner edge = original, outer edge = offset outward
    walls.push([
      [lat0, lon0],
      [lat1, lon1],
      [lat1 + offLat, lon1 + offLon],
      [lat0 + offLat, lon0 + offLon],
    ]);
  }
  return walls;
}

function gpsClosePoly() {
  if (_gpsSurveyPts.length < 3) {
    status('Need at least 3 vertices to close polygon.', '#e74c3c'); return;
  }
  const mode = document.getElementById('gps-survey-mode').value;
  if (mode === 'perimeter') {
    _gpsPerimeter = [..._gpsSurveyPts];
    // Generate thin wall strips along each fence edge, offset outward
    const wallW = parseFloat(document.getElementById('gps-wall-width').value) || 3.0;
    const walls = _perimeterToWalls(_gpsSurveyPts, wallW);
    walls.forEach(w => obstacles.push(w));
    status(`Perimeter: ${_gpsSurveyPts.length} vertices → ${walls.length} wall strips added.`);
  } else {
    obstacles.push([..._gpsSurveyPts]);
    status(`Obstacle ${obstacles.length} saved (${_gpsSurveyPts.length} vertices).`);
  }
  _gpsSurveyPts = [];
  _gpsUpdatePolyInfo();
  redraw();
}

function gpsDiscardPoly() {
  _gpsSurveyPts = [];
  _gpsCapturing = false;
  _gpsAvgBuf = [];
  document.getElementById('gps-capture-btn').innerHTML = '&#9678; Capture Point';
  _gpsUpdatePolyInfo();
  redraw();
  status('Survey polygon discarded.');
}

function _gpsUpdatePolyInfo() {
  const mode = document.getElementById('gps-survey-mode').value;
  const label = mode === 'perimeter' ? 'Perimeter' : 'Obstacle';
  const el = document.getElementById('gps-poly-info');
  const closeBtn = document.getElementById('gps-close-btn');
  if (_gpsSurveyPts.length === 0) {
    el.textContent = '';
    closeBtn.disabled = true;
  } else {
    el.textContent = `${label}: ${_gpsSurveyPts.length} vertex(es)`;
    closeBtn.disabled = _gpsSurveyPts.length < 3;
  }
}

// GPS survey dot + in-progress polygon + perimeter overlay on canvas
function drawGpsSurvey() {
  // Draw saved perimeter outline (green solid boundary)
  if (_gpsPerimeter && _gpsPerimeter.length >= 3) {
    ctx.save();
    ctx.strokeStyle = 'rgba(46,204,113,0.7)';
    ctx.lineWidth = 2.5;
    ctx.setLineDash([10, 5]);
    ctx.beginPath();
    _gpsPerimeter.forEach(([la, lo], i) => {
      const p = project(la, lo);
      i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y);
    });
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
    // Label vertices
    _gpsPerimeter.forEach(([la, lo], i) => {
      const p = project(la, lo);
      ctx.beginPath(); ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(46,204,113,0.8)'; ctx.fill();
    });
  }
  // Draw in-progress survey polygon (green dashed for perimeter, red dashed for obstacle)
  if (_gpsSurveyPts.length > 0) {
    const mode = document.getElementById('gps-survey-mode').value;
    const color = mode === 'perimeter' ? 'rgba(46,204,113,0.8)' : 'rgba(231,76,60,0.7)';
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    _gpsSurveyPts.forEach(([la, lo], i) => {
      const p = project(la, lo);
      i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y);
    });
    if (_gpsSurveyPts.length > 1) {
      const p0 = project(_gpsSurveyPts[0][0], _gpsSurveyPts[0][1]);
      ctx.lineTo(p0.x, p0.y);
    }
    ctx.stroke();
    ctx.restore();
    // Vertex dots
    _gpsSurveyPts.forEach(([la, lo], i) => {
      const p = project(la, lo);
      ctx.beginPath(); ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
      ctx.fillStyle = color; ctx.fill();
      ctx.fillStyle = '#fff'; ctx.font = '9px monospace';
      ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
      ctx.fillText(String(i + 1), p.x, p.y - 6);
    });
  }
  // Draw live GPS position as pulsing dot
  if (_gpsFix && _gpsFix.lat) {
    const p = project(_gpsFix.lat, _gpsFix.lon);
    const pulse = 6 + 3 * Math.sin(Date.now() / 200);
    const fixColor = _gpsFix.fix >= 4 ? '#0f0' : _gpsFix.fix >= 1 ? '#ff0' : '#f44';
    // Outer glow
    ctx.beginPath(); ctx.arc(p.x, p.y, pulse + 4, 0, Math.PI * 2);
    ctx.fillStyle = _gpsFix.fix >= 4 ? 'rgba(0,255,0,0.15)' : _gpsFix.fix >= 1 ? 'rgba(255,255,0,0.15)' : 'rgba(255,68,68,0.15)';
    ctx.fill();
    // Core dot
    ctx.beginPath(); ctx.arc(p.x, p.y, pulse, 0, Math.PI * 2);
    ctx.fillStyle = fixColor;
    ctx.globalAlpha = 0.8;
    ctx.fill();
    ctx.globalAlpha = 1.0;
    // Crosshair
    ctx.strokeStyle = fixColor; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(p.x - 12, p.y); ctx.lineTo(p.x + 12, p.y); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p.x, p.y - 12); ctx.lineTo(p.x, p.y + 12); ctx.stroke();
  }
}

// Auto-refresh port list on page load
gpsRefreshPorts();

// ── Clear all ─────────────────────────────────────────────────────
function clearAll() {
  waypoints = []; simResult = null;
  originalCorridors = null; optimizedPath = null;
  simPreflight = null; simDiagCsv = null;
  analyzeResult = null; logFileData = null; fetchedMission = null;
  clearObstacles();
  _gpsSurveyPts = []; _gpsPerimeter = null;
  _gpsUpdatePolyInfo();
  refreshTable();
  document.getElementById('stats').style.display = 'none';
  document.getElementById('az-log-name').textContent = 'no file';
  const saveBtn = document.getElementById('btn-save-log');
  if (saveBtn) saveBtn.style.display = 'none';
  redraw();
  status('Cleared.');
}

// ── Mission file save / load (CSV format) ────────────────────────
async function saveMission() {
  const name = document.getElementById('m-name').value.trim();
  if (!name) { status('Enter a mission name first.', '#e74c3c'); return; }
  const payload = {name, waypoints, obstacles, servos: getServos()};
  if (originalCorridors) payload.original_corridors = originalCorridors;
  if (optimizedPath) payload.optimized_path = optimizedPath;
  if (analyzeResult && analyzeResult.track) payload.real_track = analyzeResult.track;
  await fetch('/save_mission', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload),
  });
  status(`Saved "${name}".`);
  refreshMissionList();
  loadAnalyzeMissionList();
}

async function loadMission() {
  const sel = document.getElementById('m-select');
  const name = sel.value;
  if (!name) { status('Select a mission first.', '#e74c3c'); return; }
  const cleanName = name.replace(/\.json$/, '');
  const resp = await fetch(`/load_mission?name=${encodeURIComponent(cleanName)}`);
  const d = await resp.json();
  clearAll();
  waypoints = (d.waypoints || []).map(wp => ({servos: {}, ...wp}));
  if (d.obstacles) d.obstacles.forEach(poly => obstacles.push(poly));
  originalCorridors = d.original_corridors || null;
  optimizedPath = d.optimized_path || null;
  if (d.real_track) analyzeResult = {track: d.real_track};
  if (d.servos) setServos(d.servos);
  refreshTable(); renderAll();
  if (waypoints.length) { viewLat = waypoints[0].lat; viewLon = waypoints[0].lon; fitAll(); }
  document.getElementById('m-name').value = cleanName;
  const parts = [`${waypoints.length} wps`, `${obstacles.length} obs`];
  if (optimizedPath) parts.push(`${optimizedPath.length} opt pts`);
  status(`Loaded "${cleanName}" — ${parts.join(', ')}.`);
}

async function downloadRover(roverId) {
  const ip = document.getElementById(`r-ip-rv${roverId}`).value.trim();
  const user = document.getElementById('az-ssh-user').value.trim() || 'ilasa1';
  const key  = document.getElementById('az-ssh-key').value.trim();
  if (!ip) { status(`Enter RV${roverId} IP.`, '#e74c3c'); return; }
  status(`Downloading mission from RV${roverId}...`, '#f39c12');
  try {
    // Get latest run from rover
    const listResp = await fetch('/list_rover_runs', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ip, user, key}),
    });
    const listData = await listResp.json();
    if (!listData.runs || !listData.runs.length) { status('No runs on rover.', '#e74c3c'); return; }
    const runDir = listData.runs[0].dir;
    // Fetch run data
    const fetchResp = await fetch('/fetch_rover_run', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ip, user, key, run_dir: runDir}),
    });
    const d = await fetchResp.json();
    if (d.error) { status('Fetch error: ' + d.error, '#e74c3c'); return; }
    // Clear and load
    clearAll();
    optimizedPath = d.optimized_path || null;
    originalCorridors = d.original_corridors || null;
    simPreflight = d.sim_preflight || null;
    simDiagCsv = d.sim_diag_csv || null;
    logFileData = d.log_csv || null;
    fetchedMission = d.mission || null;
    if (d.mission && d.mission.waypoints && d.mission.waypoints.length) {
      waypoints = d.mission.waypoints.map((w, i) => ({servos: {}, ...w, idx: i}));
      if (d.mission.obstacles) obstacles = d.mission.obstacles;
    }
    refreshTable(); renderAll();
    if (waypoints.length) { viewLat = waypoints[0].lat; viewLon = waypoints[0].lon; fitAll(); }
    else if (optimizedPath && optimizedPath.length) { viewLat = optimizedPath[0].lat; viewLon = optimizedPath[0].lon; fitAll(); }
    const parts = [];
    if (waypoints.length) parts.push(`${waypoints.length} wps`);
    if (optimizedPath) parts.push(`${optimizedPath.length} opt pts`);
    if (simPreflight) parts.push(`sim CTE ${simPreflight.max_cte.toFixed(3)}m`);
    if (logFileData) parts.push('real log');
    status(`Downloaded RV${roverId} ${runDir} — ${parts.join(', ')}.`, '#27ae60');
  } catch(e) { status('Download error: ' + e, '#e74c3c'); }
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
  const wsPort = roverId === 1 ? 9090 : 9091;
  const ns = `/rv${roverId}`;
  status(`Uploading to RV${roverId} via rosbridge...`, '#f39c12');
  if (obsMode) obsFinish();
  try {
    const ws = new WebSocket(`ws://${rover_ip}:${wsPort}`);
    await new Promise((resolve, reject) => {
      ws.onopen = resolve;
      ws.onerror = () => reject(new Error('WebSocket connection failed'));
      setTimeout(() => reject(new Error('WebSocket timeout')), 5000);
    });
    // Build corridor JSON from optimizedPath (edited table) or waypoints
    let corridorJson;
    let pointCount;
    if (optimizedPath && optimizedPath.length) {
      // Build single-corridor from edited optimized path (what the table shows)
      const cl = optimizedPath.map(pt => [pt.lat, pt.lon]);
      const speeds = optimizedPath.map(pt => pt.speed);
      const ch5 = optimizedPath.map(pt => (pt.servo||{})[5]||(pt.servo||{})['5']||1500);
      const ch6 = optimizedPath.map(pt => (pt.servo||{})[6]||(pt.servo||{})['6']||1500);
      const ch7 = optimizedPath.map(pt => (pt.servo||{})[7]||(pt.servo||{})['7']||1500);
      const ch8 = optimizedPath.map(pt => (pt.servo||{})[8]||(pt.servo||{})['8']||1500);
      corridorJson = JSON.stringify({corridors:[{
        corridor_id:0, centerline:cl, width:1.5, speed:0,
        speeds, ch5, ch6, ch7, ch8,
        next_corridor_id:-1, turn_type:'auto', headland_width:0
      }], min_turn_radius:3.0, headland_width:0});
      pointCount = optimizedPath.length;
    } else if (waypoints.length) {
      const cl = waypoints.map(w => [w.lat, w.lon]);
      const speeds = waypoints.map(w => w.speed || 0);
      const ch5 = waypoints.map(w => (w.servos||{})['5']||1500);
      const ch6 = waypoints.map(w => (w.servos||{})['6']||1500);
      const ch7 = waypoints.map(w => (w.servos||{})['7']||1500);
      const ch8 = waypoints.map(w => (w.servos||{})['8']||1500);
      corridorJson = JSON.stringify({corridors:[{
        corridor_id:0, centerline:cl, width:1.5, speed:0,
        speeds, ch5, ch6, ch7, ch8,
        next_corridor_id:-1, turn_type:'auto', headland_width:0
      }], min_turn_radius:3.0, headland_width:0});
      pointCount = waypoints.length;
    } else {
      ws.close(); status('No data to upload.', '#e74c3c'); return;
    }
    // Publish as corridor mission (navigator auto-splits at turn markers)
    ws.send(JSON.stringify({
      op: 'publish', topic: `${ns}/corridor_mission`,
      type: 'std_msgs/msg/String',
      msg: {data: corridorJson},
    }));
    // Publish obstacles as fence if present
    if (obstacles.length) {
      ws.send(JSON.stringify({
        op: 'publish', topic: `${ns}/mission_fence`,
        type: 'std_msgs/msg/String',
        msg: {data: JSON.stringify({polygons: obstacles})},
      }));
    }
    ws.close();
    status(`Uploaded ${pointCount} points to RV${roverId} via rosbridge`, '#27ae60');
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
  ['grid', 'zigzag', 'scatter', 'spiral', 'corridor'].forEach(t =>
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

  } else if (pat === 'corridor') {
    // Corridor generation is server-side (arc geometry in Python)
    const body = {
      lat: cLat, lon: cLon,
      heading: parseFloat(document.getElementById('gen-c-hdg').value) || 0,
      rows:       parseInt(document.getElementById('gen-c-rows').value) || 4,
      row_length: parseFloat(document.getElementById('gen-c-len').value) || 20,
      row_spacing: parseFloat(document.getElementById('gen-c-sp').value) || 3,
      width:      parseFloat(document.getElementById('gen-c-w').value) || 1.0,
      min_turn_radius: parseFloat(document.getElementById('gen-c-rad').value) || 2.0,
      speed: spd,
      turn_type: document.getElementById('gen-c-turn').value,
    };
    fetch('/generate_corridor', {method:'POST', body:JSON.stringify(body)})
      .then(r => r.json())
      .then(d => {
        if (!d.ok) { alert('Corridor error: ' + d.error); return; }
        if (!append) { waypoints = []; simResult = null; }
        waypoints.push(...d.waypoints);
        window._lastCorridorJson = d.corridor_json;
        redraw();
        document.getElementById('gen-panel').style.display = 'none';
      });
    return; // async — don't fall through to synchronous append below
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
  window._lastCorridorJson = null;  // non-corridor generation clears corridor state
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
  // Also update optimizedPath if available
  if (optimizedPath) optimizedPath.forEach(pt => { pt.speed = spd; });
  refresh();
  status(`Speed set to ${spd} m/s on all ${waypoints.length} waypoints.`);
}

function applyBulkServo() {
  const ch5 = parseInt(document.getElementById('bulk-ch5').value) || 1500;
  const ch6 = parseInt(document.getElementById('bulk-ch6').value) || 1500;
  const ch7 = parseInt(document.getElementById('bulk-ch7').value) || 1500;
  const ch8 = parseInt(document.getElementById('bulk-ch8').value) || 1500;
  waypoints.forEach(wp => {
    if (!wp.servos) wp.servos = {};
    wp.servos['5'] = ch5; wp.servos['6'] = ch6;
    wp.servos['7'] = ch7; wp.servos['8'] = ch8;
  });
  if (optimizedPath) optimizedPath.forEach(pt => {
    if (!pt.servo) pt.servo = {};
    pt.servo[5] = ch5; pt.servo[6] = ch6;
    pt.servo[7] = ch7; pt.servo[8] = ch8;
  });
  refreshTable();
  status(`CH5=${ch5} CH6=${ch6} CH7=${ch7} CH8=${ch8} set on all points.`);
}

function editCell(td, idx, field) {
  const old = td.textContent.trim();
  const input = document.createElement('input');
  input.type = 'number'; input.value = old;
  input.step = field === 'speed' ? '0.1' : '50';
  input.style.cssText = 'width:50px;background:#0a1020;color:#fff;border:1px solid #5af;padding:1px 3px;font-size:11px';
  td.textContent = ''; td.appendChild(input);
  input.focus(); input.select();
  function commit() {
    const val = parseFloat(input.value);
    if (isNaN(val)) { td.textContent = old; return; }
    if (optimizedPath && idx < optimizedPath.length) {
      if (field === 'speed') optimizedPath[idx].speed = val;
      else if (field.startsWith('ch')) {
        if (!optimizedPath[idx].servo) optimizedPath[idx].servo = {};
        optimizedPath[idx].servo[parseInt(field.slice(2))] = Math.round(val);
      }
    }
    refreshTable(); renderAll();
  }
  input.onblur = commit;
  input.onkeydown = e => { if (e.key === 'Enter') commit(); if (e.key === 'Escape') { td.textContent = old; } };
}

function toggleTurn(idx) {
  // Works on optimizedPath if available, otherwise waypoints
  if (optimizedPath && idx < optimizedPath.length) {
    optimizedPath[idx].turn = !optimizedPath[idx].turn;
    if (!optimizedPath[idx].turn && optimizedPath[idx].speed <= 0) optimizedPath[idx].speed = 0.5;
  } else if (idx < waypoints.length) {
    if (waypoints[idx].speed < 0) waypoints[idx].speed = 0.5;
    else waypoints[idx].speed = -1;
  }
  refreshTable(); renderAll();
}

async function sendTestSensors(roverId) {
  const tank = parseFloat(document.getElementById('ts-tank').value);
  const batt = parseFloat(document.getElementById('ts-batt').value);
  const rover_ip = document.getElementById(`r-ip-rv${roverId}`).value.trim();
  status(`Sending test sensors to RV${roverId}…`, '#f39c12');
  try {
    const resp = await fetch('/send_test_sensors', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({rover_ip, rover_port: 14550, rover_sysid: roverId,
                            tank_level: tank, batt_pct: batt}),
    });
    const d = await resp.json();
    status(d.message || (d.ok ? 'Sent' : 'Error'), d.ok ? '#27ae60' : '#e74c3c');
  } catch(e) { status('Network error: ' + e, '#e74c3c'); }
}

async function importSnooped() {
  const resp = await fetch('/snooped_mission');
  const d = await resp.json();
  if (!d.waypoints || !d.waypoints.length) {
    status('No network mission captured yet — arm GQC and upload a mission.', '#e74c3c');
    return;
  }
  waypoints = d.waypoints.map(wp => ({servos: {}, ...wp}));
  obstacles = []; obsCurPts = [];
  (d.obstacles || []).forEach(poly => obstacles.push(poly));
  setServos(d.servos);
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
    const now = Date.now() / 1000;
    let servoHtml = 'Rover: —';
    for (const [sid, r] of Object.entries(d)) {
      if (now - r.ts > 5 || !r.servos) continue;
      const s = r.servos;
      servoHtml = `RV${sid}: <span style="color:#5d9">CH5=${s[5]??s['5']??'?'} CH6=${s[6]??s['6']??'?'} CH7=${s[7]??s['7']??'?'} CH8=${s[8]??s['8']??'?'}</span>`;
      break;
    }
    const sl = document.getElementById('servo-live');
    if (sl) sl.innerHTML = servoHtml;
    redraw();
  } catch(e) {}
}
setInterval(pollLiveRovers, 2000);
pollLiveRovers();

// ── Mission analyzer ──────────────────────────────────────────────

function cteColor(v) {
  const a = Math.abs(v);
  if (a < 0.10) return '#00e676';
  if (a < 0.20) return '#ffee00';
  if (a < 0.40) return '#ff9100';
  return '#ff1744';
}

function loadAnalyzeMissionList() {
  fetch('/missions').then(r => r.json()).then(list => {
    const sel = document.getElementById('az-m-select');
    sel.innerHTML = '<option value="">— none —</option>' +
      list.map(m => `<option value="${m.name}">${m.name} (${m.wp_count} wps)</option>`).join('');
  });
}

function onLogFileSelect(event) {
  const f = event.target.files[0];
  if (!f) return;
  document.getElementById('az-log-name').textContent = f.name;
  const reader = new FileReader();
  reader.onload = e => { logFileData = e.target.result; _showSaveBtn(f.name); };
  reader.readAsText(f);
}

function _showSaveBtn(filename) {
  const btn = document.getElementById('btn-save-log');
  btn.style.display = '';
  btn._filename = filename;
}

function saveLog() {
  if (!logFileData) return;
  const btn = document.getElementById('btn-save-log');
  const name = btn._filename || 'navigator_diag.csv';
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([logFileData], {type: 'text/csv'}));
  a.download = name;
  a.click();
  URL.revokeObjectURL(a.href);
}

async function fetchRoverLog(roverNum) {
  const ip   = document.getElementById(`r-ip-rv${roverNum}`).value.trim();
  const user = document.getElementById('az-ssh-user').value.trim() || 'rover';
  if (!ip) { status(`Set RV${roverNum} IP in the upload panel first.`, '#e74c3c'); return; }
  status(`Fetching navigator_diag.csv from RV${roverNum} (${ip})…`, '#888');
  try {
    const resp = await fetch('/fetch_rover_log', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ip, user,
      key: document.getElementById('az-ssh-key').value.trim(),
      path: '/tmp/navigator_diag.csv'}),
    });
    const d = await resp.json();
    if (d.error) { status(`RV${roverNum} fetch error: ${d.error}`, '#e74c3c'); return; }
    logFileData = d.content;
    const fname = `navigator_diag_rv${roverNum}.csv`;
    document.getElementById('az-log-name').textContent = `RV${roverNum} @ ${ip}`;
    _showSaveBtn(fname);
    status(`Log loaded from RV${roverNum} (${ip}) — click Run to analyze`, '#27ae60');
  } catch(e) {
    status('Fetch error: ' + e, '#e74c3c');
  }
}

async function runAnalysis() {
  if (!logFileData) { status('Select a log CSV file first.', '#e74c3c'); return; }
  const missionName = document.getElementById('az-m-select').value;
  status('Analyzing...', '#888');
  try {
    const resp = await fetch('/analyze', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({log: logFileData, mission: missionName}),
    });
    const d = await resp.json();
    if (d.error) { status('Error: ' + d.error, '#e74c3c'); return; }
    analyzeResult = d;

    // Overall stats banner
    const o = d.overall;
    const overall = document.getElementById('az-overall');
    overall.style.display = '';
    overall.innerHTML =
      `Pts: <b>${o.n_points}</b>  Dur: <b>${o.duration_s.toFixed(1)} s</b><br>` +
      `CTE rms <b style="color:#7ef">${o.cte_rms.toFixed(3)} m</b>  ` +
      `max <b style="color:#f97">${o.cte_max.toFixed(3)} m</b>  ` +
      `mean <b>${o.cte_mean.toFixed(3)} m</b><br>` +
      `RTK fix: <b style="color:${o.rtk_pct > 80 ? '#0f0' : '#ff0'}">${o.rtk_pct.toFixed(1)} %</b>`;

    // Per-segment table
    document.getElementById('az-seg-table').style.display = '';
    document.getElementById('az-seg-tbody').innerHTML = d.segments.map(s =>
      `<tr>
        <td>${s.wp_idx}</td>
        <td style="color:${cteColor(s.cte_mean)}">${s.cte_mean.toFixed(3)}</td>
        <td style="color:${cteColor(s.cte_max)}">${s.cte_max.toFixed(3)}</td>
        <td style="color:${cteColor(s.cte_rms)}">${s.cte_rms.toFixed(3)}</td>
        <td>${s.duration_s.toFixed(1)}</td>
        <td style="color:${s.rtk_pct > 80 ? '#0f0' : '#ff0'}">${s.rtk_pct.toFixed(0)}%</td>
      </tr>`
    ).join('');

    // Load mission waypoints onto the map
    if (d.mission_wps && d.mission_wps.length) {
      waypoints = d.mission_wps.map((w, i) => ({...w, idx: i}));
    }

    fitAll();
    status(`Analyzed ${o.n_points} pts · ${d.segments.length} segs · RMS CTE ${o.cte_rms.toFixed(3)} m`);
  } catch(e) {
    status('Analysis error: ' + e, '#e74c3c');
  }
}

async function listRoverRuns() {
  const ip   = document.getElementById('az-rover-ip').value.trim();
  const user = document.getElementById('az-ssh-user').value.trim() || 'ilasa1';
  if (!ip) { status('Set rover IP first.', '#e74c3c'); return; }
  status('Listing runs on ' + ip + '...', '#888');
  try {
    const resp = await fetch('/list_rover_runs', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ip, user, key: document.getElementById('az-ssh-key').value.trim()}),
    });
    const d = await resp.json();
    if (d.error) { status('Error: ' + d.error, '#e74c3c'); return; }
    const sel = document.getElementById('az-run-select');
    sel.innerHTML = '<option value="">— select a run —</option>' +
      (d.runs || []).map(r => `<option value="${r.dir}">${r.label}</option>`).join('');
    status(d.runs.length + ' run(s) found on rover', '#27ae60');
  } catch(e) { status('List error: ' + e, '#e74c3c'); }
}

async function fetchAndCompare() {
  const ip     = document.getElementById('az-rover-ip').value.trim();
  const user   = document.getElementById('az-ssh-user').value.trim() || 'ilasa1';
  const runDir = document.getElementById('az-run-select').value;
  if (!ip) { status('Set rover IP.', '#e74c3c'); return; }
  if (!runDir) { status('Select a run first (click Runs to list).', '#e74c3c'); return; }
  status('Fetching run data from ' + ip + '...', '#888');
  try {
    const resp = await fetch('/fetch_rover_run', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ip, user, key: document.getElementById('az-ssh-key').value.trim(), run_dir: runDir}),
    });
    const d = await resp.json();
    if (d.error) { status('Fetch error: ' + d.error, '#e74c3c'); return; }

    // Load log CSV
    logFileData = d.log_csv;
    document.getElementById('az-log-name').textContent = runDir;
    _showSaveBtn('navigator_diag.csv');

    // Load mission waypoints + obstacles + metadata from run
    fetchedMission = d.mission;
    originalCorridors = d.original_corridors || null;
    optimizedPath = d.optimized_path || null;
    simPreflight = d.sim_preflight || null;
    simDiagCsv = d.sim_diag_csv || null;
    if (d.mission && d.mission.waypoints && d.mission.waypoints.length) {
      waypoints = d.mission.waypoints.map((w, i) => ({...w, idx: i}));
      if (d.mission.obstacles) obstacles = d.mission.obstacles;
    }

    refreshTable();
    if (simPreflight) {
      const sp = simPreflight;
      const col = sp.max_cte < 0.5 ? '#27ae60' : sp.max_cte < 1.0 ? '#f39c12' : '#e74c3c';
      status(`Preflight sim: max CTE ${sp.max_cte.toFixed(3)}m, avg ${sp.avg_cte.toFixed(3)}m, ${sp.complete ? 'complete' : 'INCOMPLETE'}`, col);
    }
    if (d.log_csv) {
      status('Run loaded. Running comparison...', '#f39c12');
      await runComparison();
    } else {
      renderAll();
    }
  } catch(e) { status('Fetch error: ' + e, '#e74c3c'); }
}

async function runSimOnRover() {
  if (waypoints.length < 2) { status('Need at least 2 waypoints.', '#e74c3c'); return; }
  const ip   = document.getElementById('az-rover-ip').value.trim();
  const user = document.getElementById('az-ssh-user').value.trim() || 'ilasa1';
  const key  = document.getElementById('az-ssh-key').value.trim();
  if (!ip) { status('Set rover IP first.', '#e74c3c'); return; }
  status('Starting sim on rover ' + ip + '...', '#f39c12');
  try {
    const resp = await fetch('/run_sim_harness', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({
        ip, user, key, timeout: 300,
        waypoints, obstacles,
        rover_sysid: 1, servos: typeof getServos === 'function' ? getServos() : {},
        corridor_json: typeof corridorJson !== 'undefined' ? corridorJson : null,
      }),
    });
    const d = await resp.json();
    if (d.error) { status('Sim error: ' + d.error, '#e74c3c'); return; }
    if (!d.ok) { status('Sim failed: ' + (d.message || 'unknown'), '#e74c3c'); return; }
    status('Sim complete — fetching results...', '#f39c12');
    // Auto-fetch the latest run
    const listResp = await fetch('/list_rover_runs', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ip, user, key}),
    });
    const listData = await listResp.json();
    if (listData.runs && listData.runs.length) {
      const sel = document.getElementById('az-run-select');
      sel.innerHTML = '<option value="">— select a run —</option>' +
        listData.runs.map(r => `<option value="${r.dir}">${r.label}</option>`).join('');
      sel.value = listData.runs[0].dir;
      await fetchAndCompare();
    } else {
      status('Sim done but no runs found on rover.', '#e74c3c');
    }
  } catch(e) { status('Sim error: ' + e, '#e74c3c'); }
}

async function runComparison() {
  if (!logFileData) { status('Load a log CSV first (Browse or fetch from rover).', '#e74c3c'); return; }
  const missionName = document.getElementById('az-m-select').value;

  // 1. Run analysis
  status('Analyzing real log...', '#888');
  try {
    const aResp = await fetch('/analyze', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({log: logFileData, mission: missionName}),
    });
    const aData = await aResp.json();
    if (aData.error) { status('Analysis error: ' + aData.error, '#e74c3c'); return; }
    analyzeResult = aData;

    // Load mission waypoints from analysis result
    if (aData.mission_wps && aData.mission_wps.length) {
      waypoints = aData.mission_wps.map((w, i) => ({...w, idx: i}));
    }
    if (waypoints.length < 2) { status('Need a mission selected to run comparison.', '#e74c3c'); return; }

    // 2. Run simulation with same waypoints, start from first track point
    status('Running simulation...', '#f39c12');
    const track = aData.track;
    const startLat = track.length ? track[0].lat : waypoints[0].lat;
    const startLon = track.length ? track[0].lon : waypoints[0].lon;
    const startHdg = track.length && track[0].heading ? track[0].heading : null;
    const algoSel = document.getElementById('algo-select') ? document.getElementById('algo-select').value : '';
    const roverAlgo  = aData.algorithm || (fetchedMission && fetchedMission.algorithm) || '';
    const isCorridor = roverAlgo === 'corridor' || (fetchedMission && fetchedMission.corridor_mode);
    const navParams = algoSel ? {control_algorithm: algoSel} : {};
    // Auto-select matching algorithm from rover's actual run
    if (!algoSel && roverAlgo) {
      navParams.control_algorithm = roverAlgo;
    }

    const sResp = await fetch('/simulate', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({waypoints, start: {lat: startLat, lon: startLon, heading: startHdg}, obstacles, nav_params: navParams}),
    });
    const sData = await sResp.json();
    if (sData.error) { status('Sim error: ' + sData.error, '#e74c3c'); return; }
    simResult = sData;

    // 3. Show comparison stats
    const o = aData.overall;
    const overall = document.getElementById('az-overall');
    overall.style.display = '';
    const simDur = (sData.total_steps / 25).toFixed(1);
    const dRms = o.cte_rms - sData.rms_xte;
    const dMax = o.cte_max - sData.max_xte;
    const modeTag = isCorridor ? '<span style="color:#4f8">corridor</span>' :
                    roverAlgo ? `<span style="color:#4f8">${roverAlgo}</span>` : '';
    overall.innerHTML =
      `<b style="color:#ff9">Real rover:</b>${modeTag ? ' ' + modeTag : ''}<br>` +
      `Pts: <b>${o.n_points}</b>  Dur: <b>${o.duration_s.toFixed(1)} s</b>  ` +
      `RTK: <b style="color:${o.rtk_pct > 80 ? '#0f0' : '#ff0'}">${o.rtk_pct.toFixed(1)}%</b><br>` +
      `CTE rms <b style="color:#7ef">${o.cte_rms.toFixed(3)} m</b>  ` +
      `max <b style="color:#f97">${o.cte_max.toFixed(3)} m</b>  ` +
      `mean <b>${o.cte_mean.toFixed(3)} m</b>` +
      `<hr style="border-color:#335;margin:4px 0">` +
      `<b style="color:#4af">Simulation:</b> <span style="color:#4af">${algoSel || sData.algorithm || 'stanley'}${isCorridor ? ' (corridor params)' : ''}</span><br>` +
      `Steps: <b>${sData.total_steps}</b>  Dur: <b>${simDur} s</b>  ` +
      `WPs: <b>${(sData.waypoints_reached||[]).length}/${waypoints.length}</b><br>` +
      `XTE rms <b style="color:#4af">${sData.rms_xte.toFixed(3)} m</b>  ` +
      `max <b style="color:#fa5">${sData.max_xte.toFixed(3)} m</b>  ` +
      `avg <b>${sData.avg_xte.toFixed(3)} m</b>` +
      `<hr style="border-color:#335;margin:4px 0">` +
      `<b style="color:#aaa">\u0394 (real\u2212sim):</b>  ` +
      `rms <b style="color:${dRms > 0 ? '#f97' : '#7ef'}">${dRms > 0 ? '+' : ''}${dRms.toFixed(3)} m</b>  ` +
      `max <b style="color:${dMax > 0 ? '#f97' : '#7ef'}">${dMax > 0 ? '+' : ''}${dMax.toFixed(3)} m</b>`;

    // Per-segment table
    document.getElementById('az-seg-table').style.display = '';
    document.getElementById('az-seg-tbody').innerHTML = aData.segments.map(s =>
      `<tr>
        <td>${s.wp_idx}</td>
        <td style="color:${cteColor(s.cte_mean)}">${s.cte_mean.toFixed(3)}</td>
        <td style="color:${cteColor(s.cte_max)}">${s.cte_max.toFixed(3)}</td>
        <td style="color:${cteColor(s.cte_rms)}">${s.cte_rms.toFixed(3)}</td>
        <td>${s.duration_s.toFixed(1)}</td>
        <td style="color:${s.rtk_pct > 80 ? '#0f0' : '#ff0'}">${s.rtk_pct.toFixed(0)}%</td>
      </tr>`
    ).join('');

    fitAll();
    status(`Compare: real CTE rms=${o.cte_rms.toFixed(3)}m vs sim XTE rms=${sData.rms_xte.toFixed(3)}m (\u0394=${dRms > 0 ? '+' : ''}${dRms.toFixed(3)}m)`);
  } catch(e) {
    status('Comparison error: ' + e, '#e74c3c');
  }
}

// ── Layer: Raw — corridor vertices as rover received from GQC ────
function drawRawCorridors() {
  if (!originalCorridors) return;
  const corrs = originalCorridors.corridors || [];
  let gi = 0, turns = 0, total = 0;
  corrs.forEach(c => {
    const cl = c.centerline || [];
    const sp = c.speeds || [];
    total += cl.length;
    // Line
    if (cl.length > 1) {
      ctx.save(); ctx.strokeStyle = 'rgba(255,255,255,0.3)';
      ctx.lineWidth = 1; ctx.setLineDash([3, 3]); ctx.beginPath();
      cl.forEach(([la, lo], i) => { const p = project(la, lo); i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke(); ctx.restore();
    }
    // Dots + labels
    cl.forEach(([la, lo], i) => {
      const p = project(la, lo);
      const isTurn = sp[i] !== undefined && sp[i] < 0;
      if (isTurn) turns++;
      ctx.beginPath(); ctx.arc(p.x, p.y, isTurn ? 6 : 3, 0, Math.PI * 2);
      ctx.fillStyle = isTurn ? '#ffeb3b' : '#ffffff'; ctx.fill();
      if (isTurn) { ctx.strokeStyle = '#f44336'; ctx.lineWidth = 1.5; ctx.stroke(); }
      if (isTurn || gi % 10 === 0) {
        ctx.fillStyle = isTurn ? '#ffeb3b' : 'rgba(255,255,255,0.5)';
        ctx.font = '9px monospace'; ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
        ctx.fillText(String(gi), p.x, p.y - (isTurn ? 8 : 5));
      }
      gi++;
    });
  });
  ctx.fillStyle = 'rgba(255,255,255,0.7)'; ctx.font = '11px monospace';
  ctx.textAlign = 'left'; ctx.textBaseline = 'top';
  ctx.fillText(`Raw: ${total} pts, ${turns} turns`, 8, 8);
}

// ── Layer: Optimized — path as rover built it after corridor split ──
function drawOptimizedPath() {
  if (!optimizedPath || !optimizedPath.length) return;
  // Line
  ctx.save(); ctx.strokeStyle = 'rgba(46,204,113,0.7)'; ctx.lineWidth = 2; ctx.beginPath();
  optimizedPath.forEach((pt, i) => { const p = project(pt.lat, pt.lon); i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
  ctx.stroke(); ctx.restore();
  // Dots + numbers — turn points highlighted
  let turnCount = 0;
  optimizedPath.forEach((pt, i) => {
    const p = project(pt.lat, pt.lon);
    const isTurn = pt.turn === true;
    if (isTurn) turnCount++;
    ctx.beginPath(); ctx.arc(p.x, p.y, isTurn ? 7 : 3, 0, Math.PI * 2);
    ctx.fillStyle = isTurn ? '#ff5722' : '#2ecc71'; ctx.fill();
    if (isTurn) {
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
      // Turn label
      ctx.fillStyle = '#ff5722'; ctx.font = 'bold 10px monospace';
      ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
      ctx.fillText('T' + turnCount, p.x, p.y - 9);
    } else if (i % 10 === 0) {
      ctx.fillStyle = 'rgba(46,204,113,0.7)'; ctx.font = '9px monospace';
      ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
      ctx.fillText(String(i), p.x, p.y - 5);
    }
  });
  ctx.fillStyle = 'rgba(46,204,113,0.7)'; ctx.font = '11px monospace';
  ctx.textAlign = 'left'; ctx.textBaseline = 'top';
  ctx.fillText(`Opt: ${optimizedPath.length} pts, ${turnCount} turns`, 8, 22);
}

// ── Layer: Sim Diag — CTE-colored track from preflight sim ──
function drawSimDiagTrack() {
  if (!simDiagCsv) return;
  // Parse CSV into track points (same format as navigator_diag.csv)
  if (!drawSimDiagTrack._parsed || drawSimDiagTrack._src !== simDiagCsv) {
    const lines = simDiagCsv.trim().split('\n');
    const hdr = lines[0].split(',');
    const ci = {lat: hdr.indexOf('lat'), lon: hdr.indexOf('lon'), cte: hdr.indexOf('cte'),
                heading: hdr.indexOf('heading'), target_brg: hdr.indexOf('target_brg'),
                hdg_err: hdr.indexOf('hdg_err'), steer_ppm: hdr.indexOf('steer_ppm'),
                throttle_ppm: hdr.indexOf('throttle_ppm'), wp_idx: hdr.indexOf('wp_idx'),
                algo: hdr.indexOf('algo')};
    drawSimDiagTrack._track = [];
    for (let i = 1; i < lines.length; i++) {
      const c = lines[i].split(',');
      if (c.length < 4) continue;
      drawSimDiagTrack._track.push({
        lat: parseFloat(c[ci.lat]), lon: parseFloat(c[ci.lon]),
        cte: Math.abs(parseFloat(c[ci.cte])),
        algo: ci.algo >= 0 ? c[ci.algo] : '',
      });
    }
    drawSimDiagTrack._src = simDiagCsv;
    drawSimDiagTrack._parsed = true;
  }
  const track = drawSimDiagTrack._track;
  if (!track.length) return;
  // Draw CTE-colored segments
  for (let i = 1; i < track.length; i++) {
    const a = project(track[i-1].lat, track[i-1].lon);
    const b = project(track[i].lat, track[i].lon);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
    ctx.strokeStyle = cteColor(track[i].cte);
    ctx.lineWidth = 3; ctx.stroke();
  }
  // Label with CTE stats
  if (simPreflight) {
    const sp = simPreflight;
    const col = sp.max_cte < 0.5 ? '#27ae60' : sp.max_cte < 1.0 ? '#f39c12' : '#e74c3c';
    ctx.fillStyle = col; ctx.font = '11px monospace';
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText(`Sim: max CTE ${sp.max_cte.toFixed(3)}m, avg ${sp.avg_cte.toFixed(3)}m`, 8, 34);
  }
}

function drawAnalyzeTrack() {
  if (!analyzeResult || !analyzeResult.track || !analyzeResult.track.length) return;
  const track = analyzeResult.track;
  // Draw colored track segments
  for (let i = 1; i < track.length; i++) {
    const a = project(track[i-1].lat, track[i-1].lon);
    const b = project(track[i].lat,   track[i].lon);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.strokeStyle = cteColor(track[i].cte);
    ctx.lineWidth = 3;
    ctx.stroke();
  }
  // Start marker (cyan dot)
  const s = project(track[0].lat, track[0].lon);
  ctx.beginPath(); ctx.arc(s.x, s.y, 5, 0, Math.PI * 2);
  ctx.fillStyle = '#00bcd4'; ctx.fill();
  // End marker (magenta dot)
  const e = project(track[track.length-1].lat, track[track.length-1].lon);
  ctx.beginPath(); ctx.arc(e.x, e.y, 5, 0, Math.PI * 2);
  ctx.fillStyle = '#e040fb'; ctx.fill();
}

// ── Init ──────────────────────────────────────────────────────────
resizeCanvas();
loadAnalyzeMissionList();
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
            name = _unquote(self.path.split('=', 1)[1])
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

        elif self.path == '/gps_status':
            with _gps_lock:
                data = dict(_gps_fix)
            data['connected'] = _gps_serial is not None
            self._json(data)

        elif self.path == '/gps_list_ports':
            self._json(_gps_list_ports())

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

            # Compute pivot waypoints for the frontend
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
                                      data.get('obstacles', []),
                                      data.get('servos', {}),
                                      data.get('original_corridors'),
                                      data.get('optimized_path'),
                                      data.get('real_track'))
            print(f'[save] {path}', flush=True)
            self._json({'ok': True, 'name': name})


        elif self.path == '/upload_rover_ssh':
            import subprocess, tempfile
            try:
                data = json.loads(raw)
                rover_ip = data.get('rover_ip', '').strip()
                rover_sysid = int(data.get('rover_sysid', 1))
                ssh_user = (data.get('user', '') or 'ilasa1').strip()
                ssh_key = (data.get('key', '') or '').strip()
                mission = data.get('mission', {})
                if not rover_ip:
                    self._json({'error': 'rover IP required'}); return
                ns = f'/rv{rover_sysid}'

                # 1. Write mission JSON to temp file
                with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as tmp:
                    json.dump(mission, tmp)
                    tmp_path = tmp.name

                # 2. SCP to rover
                scp_cmd = ['scp', '-o', 'StrictHostKeyChecking=no',
                           '-o', 'ConnectTimeout=5', '-o', 'BatchMode=yes']
                if ssh_key:
                    scp_cmd += ['-i', os.path.expanduser(ssh_key)]
                remote_path = '/tmp/mission_upload.json'
                scp_cmd += [tmp_path, f'{ssh_user}@{rover_ip}:{remote_path}']
                res = subprocess.run(scp_cmd, capture_output=True, timeout=15)
                os.unlink(tmp_path)
                if res.returncode != 0:
                    err = res.stderr.decode(errors='replace').strip()
                    self._json({'error': f'SCP failed: {err}'}); return

                # 3. SSH: run upload_mission.py inside Docker container
                ssh_cmd = ['ssh', '-o', 'StrictHostKeyChecking=no',
                           '-o', 'ConnectTimeout=5', '-o', 'BatchMode=yes']
                if ssh_key:
                    ssh_cmd += ['-i', os.path.expanduser(ssh_key)]
                ssh_cmd += [f'{ssh_user}@{rover_ip}',
                            f'docker exec agri_rover_rv{rover_sysid} bash -c '
                            f'"source /opt/ros/*/setup.bash && '
                            f'source /workspaces/isaac_ros-dev/install/setup.bash && '
                            f'python3 /workspaces/isaac_ros-dev/tools/upload_mission.py '
                            f'{remote_path} --ns {ns}"']
                res = subprocess.run(ssh_cmd, capture_output=True, timeout=30)
                stdout = res.stdout.decode(errors='replace').strip()
                stderr = res.stderr.decode(errors='replace').strip()
                if res.returncode != 0:
                    self._json({'error': f'SSH upload failed: {stderr or stdout}'}); return

                mission_type = 'corridor' if 'corridors' in mission else 'waypoint'
                n_items = len(mission.get('corridors', [])) or len(mission.get('waypoints', []))
                print(f'[upload_ssh] {mission_type} mission ({n_items} items) uploaded to RV{rover_sysid}', flush=True)
                self._json({'ok': True, 'message': f'Uploaded {mission_type} mission to RV{rover_sysid} via SSH'})

            except FileNotFoundError:
                self._json({'error': 'ssh/scp not found — install OpenSSH client'})
            except subprocess.TimeoutExpired:
                self._json({'error': f'SSH connection timed out'})
            except Exception as e:
                import traceback; traceback.print_exc()
                self._json({'error': str(e)})

        elif self.path == '/upload_rover':
            try:
                data      = json.loads(raw)
                rover_ip  = data.get('rover_ip', '192.168.100.19')
                rover_port = int(data.get('rover_port', 14550))
                rover_sysid = int(data.get('rover_sysid', 1))
                wps    = data.get('waypoints', [])
                obs    = data.get('obstacles', [])
                servos = data.get('servos', {})
                print(f'[upload] -> RV{rover_sysid} @ {rover_ip}:{rover_port}  '
                      f'wps={len(wps)} obs={len(obs)}', flush=True)
                result = _mavlink_upload(wps, obs, rover_ip, rover_port, rover_sysid, servos)
                print(f'[upload] {result["message"]}', flush=True)
                self._json(result)
            except Exception as e:
                print(f'[upload] handler error: {e}', flush=True)
                self._json({'ok': False, 'message': f'Server error: {e}'})

        elif self.path == '/upload_corridor_rover':
            try:
                data = json.loads(raw)
                rover_ip = data.get('rover_ip', '192.168.100.19')
                rover_port = int(data.get('rover_port', 14550))
                rover_sysid = int(data.get('rover_sysid', 1))
                corridor_json = data.get('corridor_json', '')
                if not corridor_json:
                    self._json({'ok': False, 'message': 'No corridor_json'})
                    return
                print(f'[upload_corridor] -> RV{rover_sysid} @ {rover_ip}:{rover_port}', flush=True)
                result = _mavlink_upload_corridor(corridor_json, rover_ip, rover_port, rover_sysid)
                print(f'[upload_corridor] {result["message"]}', flush=True)
                self._json(result)
            except Exception as e:
                print(f'[upload_corridor] error: {e}', flush=True)
                self._json({'ok': False, 'message': str(e)})

        elif self.path == '/generate_corridor':
            try:
                data = json.loads(raw)
                from corridor import generate_corridor_grid, corridors_to_path, corridor_mission_to_json
                mission = generate_corridor_grid(
                    origin_lat=float(data.get('lat', 20.727715)),
                    origin_lon=float(data.get('lon', -103.566782)),
                    heading_deg=float(data.get('heading', 0)),
                    row_count=int(data.get('rows', 4)),
                    row_length_m=float(data.get('row_length', 20)),
                    row_spacing_m=float(data.get('row_spacing', 3)),
                    corridor_width=float(data.get('width', 1.0)),
                    speed=float(data.get('speed', 0)),
                    min_turn_radius=float(data.get('min_turn_radius', 2.0)),
                    turn_type=data.get('turn_type', 'auto'),
                )
                path_pts = corridors_to_path(mission, default_speed=float(data.get('speed', 1.0)))
                wps = [{'lat': p[0], 'lon': p[1], 'speed': p[2], 'hold_secs': 0,
                        'corridor_width': p[3], 'turn': p[4],
                        'servo': p[5] if len(p) > 5 else None}
                       for p in path_pts]
                print(f'[corridor] {len(mission.corridors)} corridors -> {len(wps)} path pts', flush=True)
                self._json({
                    'ok': True,
                    'waypoints': wps,
                    'corridor_json': corridor_mission_to_json(mission),
                    'corridors': len(mission.corridors),
                })
            except Exception as e:
                print(f'[corridor] error: {e}', flush=True)
                import traceback; traceback.print_exc()
                self._json({'ok': False, 'error': str(e)})

        elif self.path == '/send_test_sensors':
            try:
                data       = json.loads(raw)
                rover_ip   = data.get('rover_ip', '192.168.100.19')
                rover_port = int(data.get('rover_port', 14550))
                rover_sysid = int(data.get('rover_sysid', 1))
                tank_level = float(data.get('tank_level', 50.0))
                batt_pct   = float(data.get('batt_pct', 85.0))
                result = _mavlink_send_test_sensors(
                    rover_ip, rover_port, rover_sysid, tank_level, batt_pct)
                print(f'[test_sensors] {result["message"]}', flush=True)
                self._json(result)
            except Exception as e:
                self._json({'ok': False, 'message': f'Server error: {e}'})

        elif self.path == '/analyze':
            data = json.loads(raw)
            result = _compute_analysis(data.get('log', ''), data.get('mission', ''))
            self._json(result)

        elif self.path == '/run_sim_harness':
            import subprocess, time as _time
            data     = json.loads(raw)
            rover_ip = data.get('ip', '').strip()
            ssh_user = (data.get('user', '') or 'ilasa1').strip()
            ssh_key  = (data.get('key', '') or '').strip()
            timeout  = int(data.get('timeout', 300))
            if not rover_ip:
                self._json({'error': 'rover IP required'}); return

            def _ssh_cmd(remote_cmd, ssh_timeout=15):
                cmd = ['ssh', '-o', 'StrictHostKeyChecking=no',
                       '-o', 'ConnectTimeout=5', '-o', 'BatchMode=yes']
                if ssh_key:
                    cmd += ['-i', os.path.expanduser(ssh_key)]
                cmd += [f'{ssh_user}@{rover_ip}', remote_cmd]
                return subprocess.run(cmd, capture_output=True, timeout=ssh_timeout)

            try:
                wps = data.get('waypoints', [])
                obs = data.get('obstacles', [])
                servos = data.get('servos', {})
                corridor_json = data.get('corridor_json')
                rover_sysid = int(data.get('rover_sysid', 1))

                # 1. Start sim_harness in background (must subscribe before upload)
                print(f'[sim_harness] Starting sim_harness on {rover_ip}...', flush=True)
                sim_cmd = (
                    'docker exec -d agri_rover_rv1 bash -c "'
                    'source /opt/ros/*/setup.bash && '
                    'source /workspaces/isaac_ros-dev/install/setup.bash && '
                    'ros2 run agri_rover_simulator sim_harness '
                    '--ros-args -r __ns:=/rv1 '
                    '--params-file /workspaces/isaac_ros-dev/install/agri_rover_bringup/share/agri_rover_bringup/config/rover1_params.yaml'
                    '"'
                )
                _ssh_cmd(sim_cmd, ssh_timeout=15)

                # 2. Wait for sim_harness to start and subscribe to topics
                _time.sleep(3)

                # 3. Upload mission via MAVLink
                print(f'[sim_harness] Uploading mission to {rover_ip}...', flush=True)
                if corridor_json:
                    upload_result = _mavlink_upload_corridor(
                        corridor_json, rover_ip, 14550, rover_sysid)
                else:
                    upload_result = _mavlink_upload(
                        wps, obs, rover_ip, 14550, rover_sysid, servos)
                if not upload_result.get('ok'):
                    self._json({'error': f'Upload failed: {upload_result.get("message", "?")}'})
                    return

                # 4. Poll until sim_harness process exits (mission complete)
                print(f'[sim_harness] Mission uploaded, waiting for sim...', flush=True)
                poll_start = _time.monotonic()
                while _time.monotonic() - poll_start < timeout:
                    _time.sleep(3)
                    res = _ssh_cmd(
                        'docker exec agri_rover_rv1 pgrep -f sim_harness_node > /dev/null 2>&1 && echo running || echo done',
                        ssh_timeout=10)
                    out = res.stdout.decode(errors='replace').strip()
                    if 'done' in out:
                        break

                print(f'[sim_harness] Sim complete on {rover_ip}', flush=True)
                self._json({'ok': True})

            except FileNotFoundError:
                self._json({'error': 'ssh not found — install OpenSSH client'})
            except subprocess.TimeoutExpired:
                self._json({'error': f'Sim timed out after {timeout}s'})
            except Exception as e:
                import traceback; traceback.print_exc()
                self._json({'error': str(e)})

        elif self.path == '/list_rover_runs':
            import subprocess
            data     = json.loads(raw)
            rover_ip = data.get('ip', '').strip()
            ssh_user = (data.get('user', '') or 'ilasa1').strip()
            ssh_key  = (data.get('key', '') or '').strip()
            if not rover_ip:
                self._json({'error': 'rover IP required'}); return
            try:
                cmd = ['ssh',
                       '-o', 'StrictHostKeyChecking=no',
                       '-o', 'ConnectTimeout=5',
                       '-o', 'BatchMode=yes']
                if ssh_key:
                    cmd += ['-i', os.path.expanduser(ssh_key)]
                cmd += [f'{ssh_user}@{rover_ip}',
                        'ls -1dt /tmp/rover_runs/run_* 2>/dev/null | head -20']
                res = subprocess.run(cmd, capture_output=True, timeout=10)
                if res.returncode != 0:
                    err = res.stderr.decode(errors='replace').strip() or 'ssh failed'
                    self._json({'error': err}); return
                dirs = [d.strip() for d in res.stdout.decode().strip().split('\n') if d.strip()]
                runs = []
                for d in dirs:
                    name = os.path.basename(d)
                    # run_YYYYMMDD_HHMMSS → readable label
                    parts = name.replace('run_', '').split('_')
                    if len(parts) == 2 and len(parts[0]) == 8:
                        dt = parts[0]
                        tm = parts[1]
                        label = f'{dt[:4]}-{dt[4:6]}-{dt[6:]} {tm[:2]}:{tm[2:4]}:{tm[4:]}'
                    else:
                        label = name
                    runs.append({'dir': name, 'label': label})
                self._json({'runs': runs})
            except FileNotFoundError:
                self._json({'error': 'ssh not found'})
            except subprocess.TimeoutExpired:
                self._json({'error': f'Connection to {rover_ip} timed out'})
            except Exception as e:
                self._json({'error': str(e)})

        elif self.path == '/fetch_rover_run':
            import subprocess, tempfile
            data     = json.loads(raw)
            rover_ip = data.get('ip', '').strip()
            ssh_user = (data.get('user', '') or 'ilasa1').strip()
            ssh_key  = (data.get('key', '') or '').strip()
            run_dir  = data.get('run_dir', '').strip()
            if not rover_ip or not run_dir:
                self._json({'error': 'rover IP and run_dir required'}); return
            base_path = f'/tmp/rover_runs/{run_dir}'

            def _scp_file(remote_path):
                tmp = tempfile.NamedTemporaryFile(delete=False)
                tmp.close()
                try:
                    cmd = ['scp',
                           '-o', 'StrictHostKeyChecking=no',
                           '-o', 'ConnectTimeout=5',
                           '-o', 'BatchMode=yes']
                    if ssh_key:
                        cmd += ['-i', os.path.expanduser(ssh_key)]
                    cmd += [f'{ssh_user}@{rover_ip}:{remote_path}', tmp.name]
                    res = subprocess.run(cmd, capture_output=True, timeout=15)
                    if res.returncode != 0:
                        return None
                    with open(tmp.name, encoding='utf-8', errors='replace') as f:
                        return f.read()
                finally:
                    try: os.unlink(tmp.name)
                    except: pass

            try:
                log_csv = _scp_file(f'{base_path}/navigator_diag.csv')
                mission_raw = _scp_file(f'{base_path}/mission.json')
                mission = None
                if mission_raw:
                    try: mission = json.loads(mission_raw)
                    except: pass
                # Fetch additional corridor debug files (optional)
                original_raw = _scp_file(f'{base_path}/original_corridors.json')
                original_corridors = None
                if original_raw:
                    try: original_corridors = json.loads(original_raw)
                    except: pass
                optimized_raw = _scp_file(f'{base_path}/optimized_path.json')
                optimized_path = None
                if optimized_raw:
                    try: optimized_path = json.loads(optimized_raw)
                    except: pass
                sim_preflight_raw = _scp_file(f'{base_path}/sim_preflight.json')
                sim_preflight = None
                if sim_preflight_raw:
                    try: sim_preflight = json.loads(sim_preflight_raw)
                    except: pass
                sim_diag_csv = _scp_file(f'{base_path}/sim_diag.csv')
                if not log_csv and not sim_preflight and not sim_diag_csv:
                    self._json({'error': f'No data in {run_dir}'}); return
                self._json({'ok': True, 'log_csv': log_csv, 'mission': mission,
                             'original_corridors': original_corridors,
                             'optimized_path': optimized_path,
                             'sim_preflight': sim_preflight,
                             'sim_diag_csv': sim_diag_csv,
                             'run_dir': run_dir})
            except FileNotFoundError:
                self._json({'error': 'scp not found — install OpenSSH client'})
            except subprocess.TimeoutExpired:
                self._json({'error': f'Connection to {rover_ip} timed out'})
            except Exception as e:
                self._json({'error': str(e)})

        elif self.path == '/fetch_rover_log':
            import subprocess, tempfile, os as _os2
            data     = json.loads(raw)
            rover_ip = data.get('ip', '').strip()
            ssh_user = (data.get('user', '') or 'ilasa1').strip()
            ssh_key  = (data.get('key', '') or '').strip()
            log_path = data.get('path', '/tmp/navigator_diag.csv')
            if not rover_ip:
                self._json({'error': 'rover IP required'}); return
            tmp = tempfile.NamedTemporaryFile(suffix='.csv', delete=False)
            tmp.close()
            try:
                cmd = ['scp',
                       '-o', 'StrictHostKeyChecking=no',
                       '-o', 'ConnectTimeout=5',
                       '-o', 'BatchMode=yes']
                if ssh_key:
                    import os as _os3
                    cmd += ['-i', _os3.path.expanduser(ssh_key)]
                cmd += [f'{ssh_user}@{rover_ip}:{log_path}', tmp.name]
                res = subprocess.run(cmd, capture_output=True, timeout=10)
                if res.returncode != 0:
                    err = res.stderr.decode(errors='replace').strip() or 'scp failed'
                    self._json({'error': err}); return
                with open(tmp.name, encoding='utf-8', errors='replace') as f:
                    content = f.read()
                self._json({'ok': True, 'content': content})
            except FileNotFoundError:
                self._json({'error': 'scp not found — install OpenSSH client'})
            except subprocess.TimeoutExpired:
                self._json({'error': f'Connection to {rover_ip} timed out'})
            except Exception as e:
                self._json({'error': str(e)})
            finally:
                try: _os2.unlink(tmp.name)
                except: pass

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

        elif self.path == '/gps_connect':
            data = json.loads(raw)
            port = data.get('port', '')
            baud = int(data.get('baud', 9600))
            if not port:
                self._json({'error': 'port required'}); return
            self._json(_gps_connect(port, baud))

        elif self.path == '/gps_disconnect':
            self._json(_gps_disconnect())

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
