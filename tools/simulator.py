#!/usr/bin/env python3
"""
tools/simulator.py — AgriRover GPS simulator  (standalone, no ROS2 required)

Dead-reckoning physics for both rovers.  Reads ppm_decoder RP2040 over USB
and sends simulated u-blox ZED-X20P RTK-quality NMEA to rover Jetsons over WiFi
(UDP unicast).  Run nmea_wifi_rx.py on each rover Jetson to receive.

Hardware on simulator Jetson Orin Nano:
  --ppm-port /dev/ttyACM0  ← ppm_decoder RP2040  "RV1:ch0..7 RV2:ch0..7"

Network targets (rover Jetsons on WiFi):
  --rv1-ip 192.168.1.10   RV1 Jetson IP
  --rv2-ip 192.168.1.11   RV2 Jetson IP
  Each rover listens on two UDP ports:  pri-port (default 5000) + sec-port (5001)

Heading simulation:
  Secondary antenna is `--baseline` metres ahead of primary along rover heading.
  gps_driver.py recovers heading via atan2(dlon*cos(lat), dlat) — identical to real hardware.
  The cos(lat) correction is required because secondary_pos() scales dlon by 1/cos(lat).

Requirements:
  pip3 install pyserial

Usage:
  python3 simulator.py --rv1-ip 192.168.1.10 --rv2-ip 192.168.1.11 \\
                        --rv1-lat -23.550520  --rv1-lon -46.633308   \\
                        --rv2-lat -23.550620  --rv2-lon -46.633408

  # Dry-run: print sample NMEA to stdout, do not send UDP
  python3 simulator.py --dry-run
"""

from __future__ import annotations

import argparse
import math
import signal
import socket
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not found.  Run: pip3 install pyserial")


# ── NMEA helpers ──────────────────────────────────────────────────────────────

def _checksum(body: str) -> str:
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f'{cs:02X}'


def _fmt_lat(lat: float):
    hem = 'N' if lat >= 0.0 else 'S'
    lat = abs(lat)
    d = int(lat)
    m = (lat - d) * 60.0
    return f'{d:02d}{m:010.7f}', hem


def _fmt_lon(lon: float):
    hem = 'E' if lon >= 0.0 else 'W'
    lon = abs(lon)
    d = int(lon)
    m = (lon - d) * 60.0
    return f'{d:03d}{m:010.7f}', hem


def make_gga(lat: float, lon: float, alt: float = 45.0) -> bytes:
    """$GNGGA — RTK fixed (quality 4), 12 sats, 0.5 HDOP."""
    t = time.gmtime()
    utc = f'{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.00'
    lat_s, lat_h = _fmt_lat(lat)
    lon_s, lon_h = _fmt_lon(lon)
    body = (f'GNGGA,{utc},{lat_s},{lat_h},{lon_s},{lon_h},'
            f'4,12,0.5,{alt:.1f},M,0.0,M,0.5,0001')
    return f'${body}*{_checksum(body)}\r\n'.encode('ascii')


def make_vtg(heading_deg: float, speed_mps: float) -> bytes:
    """$GNVTG — true COG and speed over ground."""
    cog       = heading_deg % 360.0
    speed_kn  = speed_mps * 1.94384
    speed_kmh = speed_mps * 3.6
    body = (f'GNVTG,{cog:.2f},T,,M,'
            f'{speed_kn:.2f},N,{speed_kmh:.2f},K,D')
    return f'${body}*{_checksum(body)}\r\n'.encode('ascii')


# ── Rover dead-reckoning state ────────────────────────────────────────────────

class RoverState:
    def __init__(self, lat: float, lon: float, heading_deg: float = 0.0):
        self.lat         = lat
        self.lon         = lon
        self.heading_rad = math.radians(heading_deg)
        self.speed_mps   = 0.0

    def update(self, throttle: int, steering: int,
               max_speed: float, wheelbase: float, dt: float,
               turn_scale: float = 0.1, steer_deadband: float = 0.05):
        speed = (throttle - 1500) / 500.0 * max_speed
        steer = (steering - 1500) / 500.0
        # Apply deadband: ignore tiny stick offsets from center (PPM noise / trim).
        # Without this, the skid-steer model drifts heading even at neutral sticks.
        if abs(steer) < steer_deadband:
            steer = 0.0
        # Skid/differential steer: turn rate is proportional to max_speed,
        # independent of forward speed — allows in-place spinning.
        # turn_scale (0.0–1.0) scales maximum turn rate relative to full differential.
        omega = -(turn_scale * 2.0 * steer * max_speed / wheelbase) if wheelbase > 0 else 0.0
        self.heading_rad += omega * dt
        lat_rad = math.radians(self.lat)
        cos_lat = math.cos(lat_rad) or 1e-9
        self.lat      += (speed * math.cos(self.heading_rad) * dt) / 111320.0
        self.lon      += (speed * math.sin(self.heading_rad) * dt) / (111320.0 * cos_lat)
        self.speed_mps = speed

    def secondary_pos(self, baseline_m: float):
        lat_rad = math.radians(self.lat)
        cos_lat = math.cos(lat_rad) or 1e-9
        s_lat = self.lat + (baseline_m * math.cos(self.heading_rad)) / 111320.0
        s_lon = self.lon + (baseline_m * math.sin(self.heading_rad)) / (111320.0 * cos_lat)
        return s_lat, s_lon

    @property
    def heading_deg(self):
        return math.degrees(self.heading_rad) % 360.0


# ── PPM decoder reader thread ─────────────────────────────────────────────────

class PpmReader:
    def __init__(self, port: str, baud: int, thr_ch: int, str_ch: int):
        self._thr_ch    = thr_ch
        self._str_ch    = str_ch
        self._lock      = threading.Lock()
        self._rv1_ch    = [1500] * 8
        self._rv2_ch    = [1500] * 8
        self._last_time = 0.0
        self._ok        = False
        self.port_ok    = False
        threading.Thread(target=self._run, args=(port, baud), daemon=True).start()

    def get(self):
        with self._lock:
            r1  = (self._rv1_ch[self._thr_ch], self._rv1_ch[self._str_ch])
            r2  = (self._rv2_ch[self._thr_ch], self._rv2_ch[self._str_ch])
            age = time.monotonic() - self._last_time if self._ok else 999.0
        return r1, r2, age

    def _run(self, port: str, baud: int):
        try:
            ser = serial.Serial(port, baud, timeout=1.0)
            self.port_ok = True
            print(f'  [OK]  ppm_port             {port}')
        except serial.SerialException as e:
            print(f'  [ERR] ppm_port             {port}  — {e}', file=sys.stderr)
            return
        while True:
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException as e:
                print(f'\n[ERR] ppm_port read error: {e}', file=sys.stderr)
                break
            if not line:
                continue
            try:
                rv1_part, rv2_part = line.split(' ')
                rv1 = [int(v) for v in rv1_part[4:].split(',')]
                rv2 = [int(v) for v in rv2_part[4:].split(',')]
                if len(rv1) == 8 and len(rv2) == 8:
                    with self._lock:
                        self._rv1_ch    = rv1
                        self._rv2_ch    = rv2
                        self._last_time = time.monotonic()
                        self._ok        = True
            except (ValueError, AttributeError):
                pass


# ── UDP output ────────────────────────────────────────────────────────────────

def udp_send(sock: socket.socket, data: bytes, addr: tuple, label: str):
    try:
        sock.sendto(data, addr)
    except OSError as e:
        print(f'\n[WARN] UDP send to {label} {addr}: {e}', file=sys.stderr)


# ── Status display ────────────────────────────────────────────────────────────

ANSI_CLR  = '\033[2J\033[H'
ANSI_BOLD = '\033[1m'
ANSI_RST  = '\033[0m'
ANSI_GRN  = '\033[32m'
ANSI_YLW  = '\033[33m'
ANSI_RED  = '\033[31m'


def _status_str(rv1: RoverState, rv2: RoverState,
                ppm_age: float, tick: int, args, dry_run: bool) -> str:
    def ppm_col(age):
        if age < 0.5:  return ANSI_GRN
        if age < 2.0:  return ANSI_YLW
        return ANSI_RED

    mode    = f'{ANSI_YLW}DRY-RUN{ANSI_RST}' if dry_run else f'{ANSI_GRN}LIVE{ANSI_RST}'
    ppm_str = f'{ppm_col(ppm_age)}{ppm_age*1000:.0f} ms{ANSI_RST}'
    lines = [
        f'{ANSI_BOLD}AgriRover GPS Simulator{ANSI_RST}  [{mode}]  tick={tick}  rate={args.rate:.0f} Hz',
        f'  PPM age : {ppm_str}  (>2 s → rovers frozen)',
        f'  Targets : RV1 {args.rv1_ip}:{args.pri_port}/{args.sec_port}'
        f'  RV2 {args.rv2_ip}:{args.pri_port}/{args.sec_port}',
        '',
        f'  {"Rover":<6} {"Lat":>14} {"Lon":>14} {"Heading":>10} {"Speed":>10}',
        f'  {"─"*6} {"─"*14} {"─"*14} {"─"*10} {"─"*10}',
        f'  {"RV1":<6} {rv1.lat:>14.7f} {rv1.lon:>14.7f} {rv1.heading_deg:>9.1f}° {rv1.speed_mps:>9.2f} m/s',
        f'  {"RV2":<6} {rv2.lat:>14.7f} {rv2.lon:>14.7f} {rv2.heading_deg:>9.1f}° {rv2.speed_mps:>9.2f} m/s',
        '',
        '  Ctrl-C to stop',
    ]
    return ANSI_CLR + '\n'.join(lines) + '\n'


# ── Args ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='AgriRover GPS simulator — sends NMEA over UDP WiFi',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # PPM decoder
    p.add_argument('--ppm-port',  default='/dev/ttyACM0')
    p.add_argument('--ppm-baud',  default=115200, type=int)

    # WiFi targets
    p.add_argument('--rv1-ip',    default='192.168.1.10',
                   help='RV1 Jetson IP address')
    p.add_argument('--rv2-ip',    default='192.168.1.11',
                   help='RV2 Jetson IP address')
    p.add_argument('--pri-port',  default=5000, type=int,
                   help='UDP port for primary GPS stream (same on both rovers)')
    p.add_argument('--sec-port',  default=5001, type=int,
                   help='UDP port for secondary GPS stream (same on both rovers)')

    # Start positions
    p.add_argument('--rv1-lat',   default=-23.550520, type=float)
    p.add_argument('--rv1-lon',   default=-46.633308, type=float)
    p.add_argument('--rv1-hdg',   default=0.0,        type=float)
    p.add_argument('--rv2-lat',   default=-23.550620, type=float)
    p.add_argument('--rv2-lon',   default=-46.633408, type=float)
    p.add_argument('--rv2-hdg',   default=0.0,        type=float)

    # Physics
    p.add_argument('--max-speed',   default=1.5,  type=float, metavar='M/S')
    p.add_argument('--wheelbase',   default=0.50, type=float, metavar='M')
    p.add_argument('--turn-scale',  default=0.1,  type=float, metavar='0-1',
                   help='Turn rate scale (1.0=full differential, 0.1=10%% — default)')
    p.add_argument('--baseline',    default=0.30, type=float, metavar='M',
                   help='Antenna separation in metres')
    p.add_argument('--alt',       default=45.0, type=float, metavar='M')
    p.add_argument('--rate',      default=10.0, type=float, metavar='HZ')
    p.add_argument('--thr-ch',    default=0,    type=int)
    p.add_argument('--str-ch',    default=1,    type=int)
    p.add_argument('--dry-run',   action='store_true',
                   help='Print sample NMEA; do not send UDP')
    return p.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    print('\nAgriRover GPS Simulator (WiFi mode)')
    print(f'  Targets: RV1={args.rv1_ip}  RV2={args.rv2_ip}'
          f'  ports pri:{args.pri_port} sec:{args.sec_port}')

    ppm  = PpmReader(args.ppm_port, args.ppm_baud, args.thr_ch, args.str_ch)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rv1_pri_addr = (args.rv1_ip, args.pri_port)
    rv1_sec_addr = (args.rv1_ip, args.sec_port)
    rv2_pri_addr = (args.rv2_ip, args.pri_port)
    rv2_sec_addr = (args.rv2_ip, args.sec_port)

    rv1 = RoverState(args.rv1_lat, args.rv1_lon, args.rv1_hdg)
    rv2 = RoverState(args.rv2_lat, args.rv2_lon, args.rv2_hdg)
    dt  = 1.0 / args.rate

    # ── Dry-run ───────────────────────────────────────────────────────────────
    if args.dry_run:
        print('\n--- sample NMEA (first tick) ---')
        s1_lat, s1_lon = rv1.secondary_pos(args.baseline)
        s2_lat, s2_lon = rv2.secondary_pos(args.baseline)
        for label, data in (
            ('RV1 pri GGA', make_gga(rv1.lat, rv1.lon, args.alt)),
            ('RV1 pri VTG', make_vtg(rv1.heading_deg, 0.0)),
            ('RV1 sec GGA', make_gga(s1_lat, s1_lon, args.alt)),
            ('RV2 pri GGA', make_gga(rv2.lat, rv2.lon, args.alt)),
            ('RV2 pri VTG', make_vtg(rv2.heading_deg, 0.0)),
            ('RV2 sec GGA', make_gga(s2_lat, s2_lon, args.alt)),
        ):
            print(f'  {label}: {data.decode().strip()}')
        sock.close()
        return

    stop = threading.Event()
    signal.signal(signal.SIGINT,  lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    print('\nRunning — Ctrl-C to stop\n')
    time.sleep(0.5)

    t_next = time.monotonic()
    tick   = 0

    while not stop.is_set():
        t_now = time.monotonic()
        if t_now < t_next:
            time.sleep(t_next - t_now)
        t_next += dt
        tick   += 1

        (rv1_thr, rv1_str), (rv2_thr, rv2_str), ppm_age = ppm.get()
        if ppm_age > 2.0:
            rv1_thr = rv1_str = rv2_thr = rv2_str = 1500

        rv1.update(rv1_thr, rv1_str, args.max_speed, args.wheelbase, dt, args.turn_scale)
        rv2.update(rv2_thr, rv2_str, args.max_speed, args.wheelbase, dt, args.turn_scale)

        s1_lat, s1_lon = rv1.secondary_pos(args.baseline)
        s2_lat, s2_lon = rv2.secondary_pos(args.baseline)

        # Primary packets: GGA + VTG concatenated in one UDP datagram
        rv1_pri_pkt = make_gga(rv1.lat, rv1.lon, args.alt) + make_vtg(rv1.heading_deg, rv1.speed_mps)
        rv2_pri_pkt = make_gga(rv2.lat, rv2.lon, args.alt) + make_vtg(rv2.heading_deg, rv2.speed_mps)
        # Secondary packets: GGA only
        rv1_sec_pkt = make_gga(s1_lat, s1_lon, args.alt)
        rv2_sec_pkt = make_gga(s2_lat, s2_lon, args.alt)

        udp_send(sock, rv1_pri_pkt, rv1_pri_addr, 'rv1_pri')
        udp_send(sock, rv1_sec_pkt, rv1_sec_addr, 'rv1_sec')
        udp_send(sock, rv2_pri_pkt, rv2_pri_addr, 'rv2_pri')
        udp_send(sock, rv2_sec_pkt, rv2_sec_addr, 'rv2_sec')

        if tick % max(1, round(args.rate)) == 0:
            sys.stdout.write(_status_str(rv1, rv2, ppm_age, tick, args, args.dry_run))
            sys.stdout.flush()

    print('\n\nSimulator stopped.')
    sock.close()


if __name__ == '__main__':
    main()
