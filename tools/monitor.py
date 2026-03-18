"""
tools/monitor.py — Live terminal dashboard for both rovers.

Listens on UDP :14550 — both rovers broadcast to this port (different Jetsons,
no conflict). Rovers are identified by MAVLink sysid in HEARTBEAT.

Usage:
  python tools/monitor.py
"""

from __future__ import annotations

import os
import socket
import threading
import time
from dataclasses import dataclass, field

os.environ['MAVLINK20'] = '1'
try:
    from pymavlink import mavutil
except ImportError:
    raise ImportError("pip install pymavlink")


@dataclass
class RoverState:
    rv_id:       int   = 0
    ip:          str   = '?'
    lat:         float = 0.0
    lon:         float = 0.0
    fix_type:    str   = 'NO_FIX'
    armed:       bool  = False
    mode:        str   = 'UNKNOWN'
    heading:     float = -1.0   # degrees, -1 = unknown
    battery_v:   float = 0.0
    battery_pct: float = -1.0
    cmd_thr:     int   = 1500   # navigator throttle PPM
    cmd_str:     int   = 1500   # navigator steering PPM
    wp_active:   int   = -1    # active waypoint seq (-1 = none)
    wp_total:    int   = 0     # total waypoints in mission
    sbus_ch:     list  = field(default_factory=lambda: [1500] * 16)  # raw SBUS, 16 ch
    last_hb:     float = 0.0
    sensors:     dict  = field(default_factory=dict)
    log:         list  = field(default_factory=list)


RV    = {1: RoverState(rv_id=1), 2: RoverState(rv_id=2)}
_lock = threading.Lock()

MAV_MODE_ARMED = 128


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


def listen():
    """
    Receive MAVLink UDP on :14550 using a raw socket so the source IP comes
    directly from recvfrom() — no dependency on pymavlink internal attributes.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(2.0)
    sock.bind(('', 14550))

    # One MAVLink parser per rover to maintain per-stream sequence state
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

        # Fast sysid from raw bytes — avoid a full parse just to route the packet
        if data[0] == 0xFD and len(data) >= 6:    # MAVLink v2
            sysid = data[5]
        elif data[0] == 0xFE and len(data) >= 4:  # MAVLink v1
            sysid = data[3]
        else:
            continue

        if sysid not in RV:
            continue

        msgs = parsers[sysid].parse_buffer(data)
        if not msgs:
            continue

        t_now = time.time()
        with _lock:
            rv = RV[sysid]
            rv.ip = src_ip                        # always fresh from recvfrom
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
                elif t == 'RC_CHANNELS':
                    rv.sbus_ch = [getattr(msg, f'chan{i}_raw', 1500) for i in range(1, 17)]
                elif t == 'SYS_STATUS':
                    rv.battery_v   = msg.voltage_battery / 1000.0
                    rv.battery_pct = msg.battery_remaining
                elif t == 'NAMED_VALUE_FLOAT':
                    name = msg.name.rstrip('\x00') if isinstance(msg.name, str) else msg.name.rstrip(b'\x00').decode()
                    if name == 'CMD_T':
                        rv.cmd_thr = int(msg.value)
                    elif name == 'CMD_S':
                        rv.cmd_str = int(msg.value)
                    elif name == 'WP_ACT':
                        rv.wp_active = int(msg.value)
                    elif name == 'WP_TOT':
                        rv.wp_total = int(msg.value)
                    else:
                        rv.sensors[name] = round(msg.value, 1)
                elif t == 'STATUSTEXT':
                    sev = {0: 'EMERG', 1: 'ALERT', 2: 'CRIT', 3: 'ERR',
                           4: 'WARN',  5: 'NOTE',  6: 'INFO', 7: 'DBG'}.get(msg.severity, '?')
                    rv.log.append(f'[{sev}] {msg.text}')
                    if len(rv.log) > 20:
                        rv.log.pop(0)


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
                    str_bar = '◄' * (max(0, -str_delta) // 50) + '►' * (max(0, str_delta) // 50)
                    wp_str = f'wp={rv.wp_active}/{rv.wp_total - 1}' if rv.wp_total > 0 else 'wp=--'
                    print(f'  CMD  thr={rv.cmd_thr} ({thr_delta:+d}) {thr_bar}'
                          f'   str={rv.cmd_str} ({str_delta:+d}) {str_bar or "─"}'
                          f'   {wp_str}')

                sbus = rv.sbus_ch
                # Raw SBUS channels 1-8 as received from RP2040, plus rover-select (ch9)
                sbus8 = ' '.join(f'{c:4d}' for c in sbus[:8])
                print(f'  SBUS {sbus8}  sel={sbus[8]:4d}')

                # Firmware-remapped PPM — what the motor controllers physically receive
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
                if rv.log:
                    print(f'  LOG  {rv.log[-1]}')
        print('\n' + '─' * 72)
        print('  Ctrl+C to exit')


if __name__ == '__main__':
    threading.Thread(target=listen, daemon=True).start()
    render()
