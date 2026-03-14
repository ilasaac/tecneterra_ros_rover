"""
tools/mission_uploader.py — Upload a CSV mission to a rover via MAVLink.

CSV format (one waypoint per line):
  lat,lon[,speed[,acceptance_radius]]
  e.g.: -22.9068,43.1729,1.5,2.0

Usage:
  python tools/mission_uploader.py mission.csv --rover 1 --host 192.168.1.10
  python tools/mission_uploader.py mission.csv --rover 2 --host 192.168.1.11
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

os.environ['MAVLINK20'] = '1'
try:
    from pymavlink import mavutil
except ImportError:
    raise ImportError("pip install pymavlink")


def load_csv(path: str) -> list[dict]:
    waypoints = []
    with open(path, newline='') as f:
        for i, row in enumerate(csv.reader(f)):
            row = [r.strip() for r in row if r.strip()]
            if not row or row[0].startswith('#'):
                continue
            wp = {
                'seq':              i,
                'lat':              float(row[0]),
                'lon':              float(row[1]),
                'speed':            float(row[2]) if len(row) > 2 else 1.0,
                'acceptance_radius': float(row[3]) if len(row) > 3 else 1.5,
            }
            waypoints.append(wp)
    return waypoints


def upload(waypoints: list[dict], host: str, port: int, rover_id: int, timeout: float = 10.0):
    conn = mavutil.mavlink_connection(
        f'udpout:{host}:{port}', source_system=255, source_component=0)

    # Wait for heartbeat
    print(f'Waiting for RV{rover_id} heartbeat...')
    conn.wait_heartbeat(timeout=timeout)
    print(f'  Connected — sysid={conn.target_system}')

    target_sys  = conn.target_system
    target_comp = conn.target_component
    count       = len(waypoints)

    # Send MISSION_COUNT
    conn.mav.mission_count_send(target_sys, target_comp, count)
    print(f'Uploading {count} waypoints...')

    deadline = time.time() + timeout
    sent     = 0

    while sent < count and time.time() < deadline:
        msg = conn.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT',
                                    'MISSION_ACK'], blocking=True, timeout=2.0)
        if msg is None:
            continue
        mt = msg.get_type()

        if mt in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
            seq = msg.seq
            wp  = waypoints[seq]
            conn.mav.mission_item_int_send(
                target_sys, target_comp,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,          # current
                1,          # autocontinue
                0,          # hold time s
                wp['acceptance_radius'],
                0,          # pass radius
                wp['speed'],
                int(wp['lat'] * 1e7),
                int(wp['lon'] * 1e7),
                0,          # alt (ground rover)
            )
            print(f'  Sent WP {seq}: {wp["lat"]:.6f}, {wp["lon"]:.6f}')
            sent += 1

        elif mt == 'MISSION_ACK':
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f'Mission accepted ({count} waypoints)')
            else:
                print(f'Mission rejected: {msg.type}', file=sys.stderr)
            return

    if sent < count:
        print('Upload timed out', file=sys.stderr)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description='Upload MAVLink mission to AgriRover')
    parser.add_argument('csv',        help='Mission CSV file')
    parser.add_argument('--rover',    type=int, default=1, choices=[1, 2])
    parser.add_argument('--host',     default='192.168.1.10')
    parser.add_argument('--port',     type=int, default=14550)
    parser.add_argument('--timeout',  type=float, default=10.0)
    args = parser.parse_args()

    waypoints = load_csv(args.csv)
    if not waypoints:
        print('No waypoints found in CSV', file=sys.stderr)
        sys.exit(1)

    upload(waypoints, args.host, args.port, args.rover, args.timeout)


if __name__ == '__main__':
    main()
