"""
tools/monitor.py — Live terminal dashboard for both rovers.

Listens:
  UDP :14550  RV1 MAVLink
  UDP :14551  RV2 MAVLink

Usage:
  python tools/monitor.py
"""

from __future__ import annotations

import os
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

os.environ['MAVLINK20'] = '1'
try:
    from pymavlink import mavutil
except ImportError:
    raise ImportError("pip install pymavlink")


@dataclass
class RoverState:
    rv_id:      int   = 0
    lat:        float = 0.0
    lon:        float = 0.0
    fix_type:   str   = 'NO_FIX'
    armed:      bool  = False
    mode:       str   = 'UNKNOWN'
    battery_v:  float = 0.0
    battery_pct: float = -1.0
    channels:   list  = field(default_factory=lambda: [1500]*9)
    last_hb:    float = 0.0
    sensors:    dict  = field(default_factory=dict)
    log:        list  = field(default_factory=list)


RV = {1: RoverState(rv_id=1), 2: RoverState(rv_id=2)}
_lock = threading.Lock()

FIX_QUALITY = {'0':'NO_FIX','1':'GPS','2':'DGPS','4':'RTK_FIX','5':'RTK_FLT'}

MAV_STATE_ACTIVE = 4
MAV_MODE_ARMED   = 128


def listen(port: int, rv_id: int):
    conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{port}')
    while True:
        msg = conn.recv_match(blocking=True, timeout=2.0)
        if msg is None:
            continue
        t = msg.get_type()
        with _lock:
            rv = RV[rv_id]
            if t == 'HEARTBEAT':
                rv.last_hb  = time.time()
                rv.armed    = bool(msg.base_mode & MAV_MODE_ARMED)
                rv.mode     = {0:'MANUAL',4:'AUTONOMOUS',16:'EMERGENCY'}.get(
                    msg.custom_mode, str(msg.custom_mode))
            elif t == 'GLOBAL_POSITION_INT':
                rv.lat = msg.lat / 1e7
                rv.lon = msg.lon / 1e7
            elif t == 'RC_CHANNELS':
                rv.channels = [getattr(msg, f'chan{i}_raw', 1500) for i in range(1,10)]
            elif t == 'SYS_STATUS':
                rv.battery_v   = msg.voltage_battery / 1000.0
                rv.battery_pct = msg.battery_remaining
            elif t == 'NAMED_VALUE_FLOAT':
                rv.sensors[msg.name] = round(msg.value, 1)
            elif t == 'STATUSTEXT':
                sev = {0:'EMERG',1:'ALERT',2:'CRIT',3:'ERR',
                       4:'WARN',5:'NOTE',6:'INFO',7:'DBG'}.get(msg.severity,'?')
                rv.log.append(f'[{sev}] {msg.text}')
                if len(rv.log) > 20:
                    rv.log.pop(0)


def render():
    while True:
        time.sleep(0.5)
        os.system('cls' if os.name == 'nt' else 'clear')
        now = time.time()
        print('═' * 70)
        print('  AGRIROVER MONITOR')
        print('═' * 70)
        with _lock:
            for rv_id, rv in RV.items():
                age    = now - rv.last_hb
                status = 'ONLINE' if age < 3 else f'LOST ({age:.0f}s)'
                print(f'\n  RV{rv_id}  {status}  {"ARMED" if rv.armed else "DISARMED"}  {rv.mode}')
                print(f'  GPS  {rv.fix_type:10s}  {rv.lat:.6f}, {rv.lon:.6f}')
                print(f'  BAT  {rv.battery_v:.2f}V  {rv.battery_pct:.0f}%')
                chs = ' '.join(f'{c:4d}' for c in rv.channels)
                print(f'  RC   {chs}')
                if rv.sensors:
                    s = rv.sensors
                    print(f'  TANK {s.get("TANK","?")}%  '
                          f'TEMP {s.get("TEMP","?")}°C  '
                          f'HUM {s.get("HUMID","?")}%  '
                          f'PRES {s.get("PRESSURE","?")}hPa')
                if rv.log:
                    print(f'  LOG  {rv.log[-1]}')
        print('\n' + '─' * 70)
        print('  Ctrl+C to exit')


if __name__ == '__main__':
    for rv_id, port in [(1, 14550), (2, 14551)]:
        t = threading.Thread(target=listen, args=(port, rv_id), daemon=True)
        t.start()
    render()
