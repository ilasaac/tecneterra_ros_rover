#!/usr/bin/env python3
"""
tools/simulator.py — AgriRover GPS simulator  (standalone, no ROS2 required)

Dead-reckoning physics for both rovers.  Reads ppm_decoder RP2040 over USB
and writes simulated u-blox ZED-X20P RTK-quality NMEA to 4 serial ports
(primary + secondary GPS for each rover Jetson).

Hardware connections on the simulator Jetson Nano (USB hub):
  ppm_port   /dev/ttyACM0  ← ppm_decoder RP2040   "RV1:ch0..7 RV2:ch0..7"
  rv1-pri    /dev/ttyUSB0  → RV1 Jetson /dev/ttyUSB0   (primary  GPS)
  rv1-sec    /dev/ttyUSB1  → RV1 Jetson /dev/ttyUSB1   (secondary GPS → heading)
  rv2-pri    /dev/ttyUSB2  → RV2 Jetson /dev/ttyUSB0   (primary  GPS)
  rv2-sec    /dev/ttyUSB3  → RV2 Jetson /dev/ttyUSB1   (secondary GPS → heading)

Heading simulation:
  Secondary antenna position is `--baseline` metres ahead of primary along the
  rover's current heading.  gps_driver.py on the rover Jetson recovers heading
  via atan2(dlon, dlat) between secondary and primary — identical to real hardware.

Requirements:
  pip3 install pyserial

Usage examples:
  # Minimal — uses default ports and São Paulo coordinates
  python3 simulator.py

  # Custom field origin
  python3 simulator.py --rv1-lat -23.550520 --rv1-lon -46.633308 \\
                        --rv2-lat -23.550620 --rv2-lon -46.633408

  # Different ports
  python3 simulator.py --ppm-port /dev/ttyACM2 --rv1-pri /dev/ttyUSB4

  # Dry-run: print NMEA to stdout only, do not open output ports
  python3 simulator.py --dry-run
"""

from __future__ import annotations

import argparse
import math
import signal
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
               max_speed: float, wheelbase: float, dt: float):
        speed = (throttle - 1500) / 500.0 * max_speed   # m/s, neg = reverse
        steer = (steering - 1500) / 500.0                # -1..+1
        omega = (speed * steer / wheelbase) if wheelbase > 0 else 0.0

        self.heading_rad += omega * dt
        lat_rad = math.radians(self.lat)
        cos_lat = math.cos(lat_rad) or 1e-9
        self.lat      += (speed * math.cos(self.heading_rad) * dt) / 111320.0
        self.lon      += (speed * math.sin(self.heading_rad) * dt) / (111320.0 * cos_lat)
        self.speed_mps = speed

    def secondary_pos(self, baseline_m: float):
        """(lat, lon) of secondary antenna, `baseline_m` ahead along heading."""
        lat_rad = math.radians(self.lat)
        cos_lat = math.cos(lat_rad) or 1e-9
        s_lat = self.lat + (baseline_m * math.cos(self.heading_rad)) / 111320.0
        s_lon = self.lon + (baseline_m * math.sin(self.heading_rad)) / (111320.0 * cos_lat)
        return s_lat, s_lon

    @property
    def heading_deg(self):
        return math.degrees(self.heading_rad) % 360.0


# ── Serial port helpers ───────────────────────────────────────────────────────

def open_port(path: str, baud: int, label: str, dry_run: bool):
    """Open a serial port, print result.  Returns Serial or None."""
    if dry_run:
        return None
    try:
        s = serial.Serial(path, baud, timeout=1.0)
        print(f'  [OK]  {label:20s} {path}')
        return s
    except serial.SerialException as e:
        print(f'  [ERR] {label:20s} {path}  — {e}', file=sys.stderr)
        return None


def write_port(ser, data: bytes, label: str):
    if ser is None:
        return
    try:
        ser.write(data)
    except serial.SerialException as e:
        print(f'\n[WARN] write failed on {label}: {e}', file=sys.stderr)


# ── PPM decoder reader thread ─────────────────────────────────────────────────

class PpmReader:
    """Background thread that reads the ppm_decoder serial output."""

    def __init__(self, port: str, baud: int, thr_ch: int, str_ch: int):
        self._thr_ch    = thr_ch
        self._str_ch    = str_ch
        self._lock      = threading.Lock()
        self._rv1_ch    = [1500] * 8
        self._rv2_ch    = [1500] * 8
        self._last_time = 0.0
        self._ok        = False   # True once first valid frame received
        self._port_ok   = False

        t = threading.Thread(target=self._run, args=(port, baud), daemon=True)
        t.start()

    def get(self):
        """Returns ((rv1_thr, rv1_str), (rv2_thr, rv2_str), age_seconds)."""
        with self._lock:
            r1 = (self._rv1_ch[self._thr_ch], self._rv1_ch[self._str_ch])
            r2 = (self._rv2_ch[self._thr_ch], self._rv2_ch[self._str_ch])
            age = time.monotonic() - self._last_time if self._ok else 999.0
        return r1, r2, age

    @property
    def port_ok(self):
        return self._port_ok

    def _run(self, port: str, baud: int):
        try:
            ser = serial.Serial(port, baud, timeout=1.0)
            self._port_ok = True
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


# ── Status display ────────────────────────────────────────────────────────────

ANSI_CLR  = '\033[2J\033[H'    # clear screen + home
ANSI_BOLD = '\033[1m'
ANSI_RST  = '\033[0m'
ANSI_GRN  = '\033[32m'
ANSI_YLW  = '\033[33m'
ANSI_RED  = '\033[31m'

def _status_str(rv1: RoverState, rv2: RoverState,
                ppm_age: float, tick: int, rate: float, dry_run: bool) -> str:
    def ppm_color(age):
        if age < 0.5:   return ANSI_GRN
        if age < 2.0:   return ANSI_YLW
        return ANSI_RED

    ppm_str = f'{ppm_color(ppm_age)}{ppm_age*1000:.0f} ms{ANSI_RST}'
    mode    = f'{ANSI_YLW}DRY-RUN{ANSI_RST}' if dry_run else f'{ANSI_GRN}LIVE{ANSI_RST}'

    lines = [
        f'{ANSI_BOLD}AgriRover GPS Simulator{ANSI_RST}  [{mode}]  tick={tick}  rate={rate:.0f} Hz',
        f'  PPM age : {ppm_str}  (>2 s → rovers frozen)',
        '',
        f'  {"Rover":<6} {"Lat":>14} {"Lon":>14} {"Heading":>10} {"Speed":>10}',
        f'  {"─"*6} {"─"*14} {"─"*14} {"─"*10} {"─"*10}',
        f'  {"RV1":<6} {rv1.lat:>14.7f} {rv1.lon:>14.7f} {rv1.heading_deg:>9.1f}° {rv1.speed_mps:>9.2f} m/s',
        f'  {"RV2":<6} {rv2.lat:>14.7f} {rv2.lon:>14.7f} {rv2.heading_deg:>9.1f}° {rv2.speed_mps:>9.2f} m/s',
        '',
        '  Ctrl-C to stop',
    ]
    return ANSI_CLR + '\n'.join(lines) + '\n'


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='AgriRover standalone GPS simulator (no ROS2 needed)',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # Ports
    p.add_argument('--ppm-port',  default='/dev/ttyACM0',
                   help='ppm_decoder RP2040 serial port')
    p.add_argument('--rv1-pri',   default='/dev/ttyUSB0',
                   help='→ RV1 Jetson /dev/ttyUSB0 (primary GPS)')
    p.add_argument('--rv1-sec',   default='/dev/ttyUSB1',
                   help='→ RV1 Jetson /dev/ttyUSB1 (secondary GPS)')
    p.add_argument('--rv2-pri',   default='/dev/ttyUSB2',
                   help='→ RV2 Jetson /dev/ttyUSB0 (primary GPS)')
    p.add_argument('--rv2-sec',   default='/dev/ttyUSB3',
                   help='→ RV2 Jetson /dev/ttyUSB1 (secondary GPS)')
    p.add_argument('--baud',      default=115200, type=int,
                   help='Baud for all ports')

    # Starting positions
    p.add_argument('--rv1-lat',   default=-23.550520, type=float)
    p.add_argument('--rv1-lon',   default=-46.633308, type=float)
    p.add_argument('--rv1-hdg',   default=0.0,        type=float,
                   help='RV1 initial heading (degrees, 0=north)')
    p.add_argument('--rv2-lat',   default=-23.550620, type=float)
    p.add_argument('--rv2-lon',   default=-46.633408, type=float)
    p.add_argument('--rv2-hdg',   default=0.0,        type=float,
                   help='RV2 initial heading (degrees, 0=north)')

    # Physics
    p.add_argument('--max-speed', default=1.5,  type=float, metavar='M/S',
                   help='Speed at full throttle (PPM 2000)')
    p.add_argument('--wheelbase', default=0.50, type=float, metavar='M',
                   help='Rover wheelbase in metres')
    p.add_argument('--baseline',  default=0.30, type=float, metavar='M',
                   help='Antenna separation (secondary ahead of primary)')

    # Output
    p.add_argument('--rate',      default=10.0, type=float, metavar='HZ',
                   help='NMEA output rate')
    p.add_argument('--alt',       default=45.0, type=float, metavar='M',
                   help='Simulated altitude (metres)')

    # Misc
    p.add_argument('--thr-ch',    default=0, type=int,
                   help='Throttle channel index in ppm_decoder output (0-7)')
    p.add_argument('--str-ch',    default=1, type=int,
                   help='Steering channel index in ppm_decoder output (0-7)')
    p.add_argument('--dry-run',   action='store_true',
                   help='Do not open output ports; print one sample NMEA to stdout')

    return p.parse_args()


def main():
    args = parse_args()

    print('\nAgriRover GPS Simulator — opening ports...')
    ppm = PpmReader(args.ppm_port, args.baud, args.thr_ch, args.str_ch)

    ports = {}
    for key, path in (('rv1_pri', args.rv1_pri), ('rv1_sec', args.rv1_sec),
                      ('rv2_pri', args.rv2_pri), ('rv2_sec', args.rv2_sec)):
        ports[key] = open_port(path, args.baud, key, args.dry_run)

    rv1 = RoverState(args.rv1_lat, args.rv1_lon, args.rv1_hdg)
    rv2 = RoverState(args.rv2_lat, args.rv2_lon, args.rv2_hdg)

    dt   = 1.0 / args.rate
    tick = 0

    # ── Dry-run: print sample sentences and exit ──────────────────────────────
    if args.dry_run:
        print('\n--- sample NMEA output (first tick) ---')
        s1_lat, s1_lon = rv1.secondary_pos(args.baseline)
        s2_lat, s2_lon = rv2.secondary_pos(args.baseline)
        for label, data in (
            ('RV1 primary  GGA', make_gga(rv1.lat, rv1.lon, args.alt)),
            ('RV1 primary  VTG', make_vtg(rv1.heading_deg, 0.0)),
            ('RV1 secondary GGA', make_gga(s1_lat, s1_lon, args.alt)),
            ('RV2 primary  GGA', make_gga(rv2.lat, rv2.lon, args.alt)),
            ('RV2 primary  VTG', make_vtg(rv2.heading_deg, 0.0)),
            ('RV2 secondary GGA', make_gga(s2_lat, s2_lon, args.alt)),
        ):
            print(f'  {label}: {data.decode().strip()}')
        print()
        return

    # ── Stop cleanly on Ctrl-C / SIGTERM ─────────────────────────────────────
    stop = threading.Event()
    signal.signal(signal.SIGINT,  lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    print('\nRunning — Ctrl-C to stop\n')
    time.sleep(0.5)   # let port-open messages settle before display takes over

    t_next = time.monotonic()

    while not stop.is_set():
        t_now = time.monotonic()
        if t_now < t_next:
            time.sleep(t_next - t_now)
        t_next += dt
        tick   += 1

        # ── Read PPM ──────────────────────────────────────────────────────────
        (rv1_thr, rv1_str), (rv2_thr, rv2_str), ppm_age = ppm.get()

        # Freeze rovers if no ppm_decoder data
        if ppm_age > 2.0:
            rv1_thr = rv1_str = rv2_thr = rv2_str = 1500

        # ── Physics ───────────────────────────────────────────────────────────
        rv1.update(rv1_thr, rv1_str, args.max_speed, args.wheelbase, dt)
        rv2.update(rv2_thr, rv2_str, args.max_speed, args.wheelbase, dt)

        # ── Build NMEA ────────────────────────────────────────────────────────
        s1_lat, s1_lon = rv1.secondary_pos(args.baseline)
        s2_lat, s2_lon = rv2.secondary_pos(args.baseline)

        rv1_gga_pri = make_gga(rv1.lat, rv1.lon, args.alt)
        rv1_vtg     = make_vtg(rv1.heading_deg, rv1.speed_mps)
        rv1_gga_sec = make_gga(s1_lat, s1_lon, args.alt)

        rv2_gga_pri = make_gga(rv2.lat, rv2.lon, args.alt)
        rv2_vtg     = make_vtg(rv2.heading_deg, rv2.speed_mps)
        rv2_gga_sec = make_gga(s2_lat, s2_lon, args.alt)

        # ── Write to serial ports ─────────────────────────────────────────────
        write_port(ports['rv1_pri'], rv1_gga_pri, 'rv1_pri')
        write_port(ports['rv1_pri'], rv1_vtg,     'rv1_pri')
        write_port(ports['rv1_sec'], rv1_gga_sec, 'rv1_sec')

        write_port(ports['rv2_pri'], rv2_gga_pri, 'rv2_pri')
        write_port(ports['rv2_pri'], rv2_vtg,     'rv2_pri')
        write_port(ports['rv2_sec'], rv2_gga_sec, 'rv2_sec')

        # ── Status display (refresh every ~1 s) ──────────────────────────────
        if tick % max(1, round(args.rate)) == 0:
            sys.stdout.write(_status_str(rv1, rv2, ppm_age, tick,
                                         args.rate, args.dry_run))
            sys.stdout.flush()

    # ── Shutdown ──────────────────────────────────────────────────────────────
    print('\n\nSimulator stopped.')
    for s in ports.values():
        if s is not None:
            s.close()


if __name__ == '__main__':
    main()
