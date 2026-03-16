#!/usr/bin/env python3
"""
tools/rtk_forwarder.py — RTK RTCM3 correction forwarder

Receives RTCM3 correction data from an NTRIP caster (internet) or an E610 DTU
base station (Ethernet/WiFi), and forwards the raw bytes to the u-blox ZED-X20P
GPS modules via serial.  Both primary and secondary modules receive the same
corrections so both can achieve RTK-fixed accuracy.

Run on each rover Jetson alongside gps_driver.  Both this script and gps_driver
open the same serial ports: gps_driver reads NMEA (module → Jetson),
this script writes RTCM3 (Jetson → module).  This is safe because they operate
in opposite directions on the same full-duplex UART.

                    ┌─────────────────────────────────────────┐
  NTRIP caster      │  rtk_forwarder.py (rover Jetson)        │
  (internet)  ──────►                                         │──► /dev/ttyUSB0 → u-blox primary
    OR               │  NtripClient / E610Client               │──► /dev/ttyUSB1 → u-blox secondary
  E610 DTU    ──────►  (RTCM3 bytes)                          │
  (Ethernet)        └─────────────────────────────────────────┘

Sources:
  --source ntrip   Connect to an NTRIP v1/v2 caster over TCP
  --source e610    Connect to E610 DTU transparent TCP bridge over Ethernet

Requirements:
  pip3 install pyserial

Usage:

  # NTRIP source (internet RTK correction service)
  python3 rtk_forwarder.py --source ntrip \\
    --ntrip-host caster.example.com --ntrip-port 2101 \\
    --mountpoint MOUNT1 --ntrip-user user --ntrip-pass pass \\
    --gps-ports /dev/ttyUSB0 /dev/ttyUSB1

  # E610 DTU base station over Ethernet port
  python3 rtk_forwarder.py --source e610 \\
    --e610-host 192.168.1.20 --e610-port 9000 \\
    --gps-ports /dev/ttyUSB0 /dev/ttyUSB1

  # NTRIP with approximate rover position (required by some VRS casters)
  python3 rtk_forwarder.py --source ntrip \\
    --ntrip-host caster.example.com --ntrip-port 2101 \\
    --mountpoint MOUNT1 --ntrip-user user --ntrip-pass pass \\
    --approx-lat -23.5505 --approx-lon -46.6333 \\
    --gps-ports /dev/ttyUSB0 /dev/ttyUSB1

Notes:
  - Both GPS modules receive the identical RTCM3 stream.
  - RTCM3 messages are variable length; forwarded byte-for-byte without parsing.
  - Automatic reconnection with exponential backoff on connection loss.
  - The --gps-baud must match the u-blox UART baud rate (default 115200).
"""

from __future__ import annotations

import argparse
import base64
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


# ── NMEA GGA builder (for NTRIP VRS position reporting) ──────────────────────

def _nmea_checksum(body: str) -> str:
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f'{cs:02X}'


def _make_approx_gga(lat: float, lon: float) -> bytes:
    """Minimal GGA sentence sent to NTRIP caster for VRS/network RTK."""
    t    = time.gmtime()
    utc  = f'{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.00'
    alat = abs(lat);  lat_h = 'N' if lat >= 0 else 'S'
    alon = abs(lon);  lon_h = 'E' if lon >= 0 else 'W'
    ld = int(alat);   lm = (alat - ld) * 60.0
    od = int(alon);   om = (alon - od) * 60.0
    lat_s = f'{ld:02d}{lm:010.7f}'
    lon_s = f'{od:03d}{om:010.7f}'
    body = f'GPGGA,{utc},{lat_s},{lat_h},{lon_s},{lon_h},1,04,2.4,0.0,M,0.0,M,,'
    return f'${body}*{_nmea_checksum(body)}\r\n'.encode('ascii')


# ── GPS serial writer ─────────────────────────────────────────────────────────

class GpsWriter:
    """Opens one or more serial ports for writing RTCM3 corrections."""

    def __init__(self, ports: list[str], baud: int):
        self._sers: list[serial.Serial] = []
        for p in ports:
            try:
                s = serial.Serial(p, baud, timeout=1.0)
                self._sers.append(s)
                print(f'  [OK]  GPS serial          {p}  @ {baud} baud')
            except serial.SerialException as e:
                print(f'  [ERR] GPS serial          {p}  — {e}', file=sys.stderr)

    def write(self, data: bytes):
        for s in self._sers:
            try:
                s.write(data)
            except serial.SerialException as e:
                print(f'\n[WARN] serial write error: {e}', file=sys.stderr)

    def close(self):
        for s in self._sers:
            s.close()


# ── NTRIP client ──────────────────────────────────────────────────────────────

class NtripClient:
    """
    NTRIP v1/v2 client.  Connects to caster, requests mount point, yields
    raw RTCM3 bytes via the `stream()` generator.

    VRS support: if approx_lat/lon are given, sends a GGA sentence to the
    caster every `gga_interval` seconds so network RTK casters can compute
    virtual reference station corrections for the rover's position.
    """

    RECV_SIZE = 4096

    def __init__(self, host: str, port: int, mountpoint: str,
                 user: str, password: str,
                 approx_lat: float = 0.0, approx_lon: float = 0.0,
                 gga_interval: float = 30.0):
        self._host         = host
        self._port         = port
        self._mountpoint   = mountpoint.lstrip('/')
        self._user         = user
        self._password     = password
        self._approx_lat   = approx_lat
        self._approx_lon   = approx_lon
        self._gga_interval = gga_interval

    def _build_request(self) -> bytes:
        creds   = base64.b64encode(f'{self._user}:{self._password}'.encode()).decode()
        request = (
            f'GET /{self._mountpoint} HTTP/1.0\r\n'
            f'User-Agent: NTRIP AgriRover/1.0\r\n'
            f'Authorization: Basic {creds}\r\n'
            f'Ntrip-Version: Ntrip/1.0\r\n'
            f'\r\n'
        )
        return request.encode('ascii')

    def stream(self):
        """Generator: yields chunks of raw RTCM3 bytes.  Raises on connection error."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)
        sock.connect((self._host, self._port))
        sock.sendall(self._build_request())

        # Read HTTP/ICY response header
        header = b''
        while b'\r\n\r\n' not in header:
            chunk = sock.recv(256)
            if not chunk:
                raise ConnectionError('NTRIP: connection closed during header')
            header += chunk

        first_line = header.split(b'\r\n')[0].decode('ascii', errors='ignore')
        if '200' not in first_line and 'ICY' not in first_line:
            raise ConnectionError(f'NTRIP: caster rejected request: {first_line}')

        print(f'  [OK]  NTRIP connected     {self._host}:{self._port}/{self._mountpoint}')

        sock.settimeout(5.0)
        last_gga = time.monotonic()

        # Yield any data that arrived after the header in the same recv
        leftover = header.split(b'\r\n\r\n', 1)[1]
        if leftover:
            yield leftover

        while True:
            # Periodically send GGA position to caster (VRS requirement)
            if (self._gga_interval > 0 and
                    self._approx_lat != 0.0 and
                    time.monotonic() - last_gga >= self._gga_interval):
                gga = _make_approx_gga(self._approx_lat, self._approx_lon)
                sock.sendall(gga)
                last_gga = time.monotonic()

            data = sock.recv(self.RECV_SIZE)
            if not data:
                raise ConnectionError('NTRIP: stream closed by caster')
            yield data

        sock.close()


# ── E610 DTU TCP client ───────────────────────────────────────────────────────

class E610Client:
    """
    Connects to an E610 DTU (Data Transfer Unit) base station over TCP.
    The E610 acts as a transparent serial-to-Ethernet bridge: RTCM3 bytes
    from the base station antenna arrive as a raw TCP stream.

    Connect the Jetson's Ethernet port directly to the E610's LAN port.
    Configure the E610 DTU to 'TCP server' mode with the RTCM3 baud rate
    matching the base station GNSS module output (typically 115200).
    """

    RECV_SIZE = 4096

    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port

    def stream(self):
        """Generator: yields chunks of raw RTCM3 bytes.  Raises on connection error."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)
        sock.connect((self._host, self._port))
        print(f'  [OK]  E610 DTU connected  {self._host}:{self._port}')
        sock.settimeout(5.0)
        while True:
            data = sock.recv(self.RECV_SIZE)
            if not data:
                raise ConnectionError('E610: connection closed')
            yield data
        sock.close()


# ── Status display ────────────────────────────────────────────────────────────

ANSI_CLR  = '\033[2J\033[H'
ANSI_BOLD = '\033[1m'
ANSI_RST  = '\033[0m'
ANSI_GRN  = '\033[32m'
ANSI_YLW  = '\033[33m'
ANSI_RED  = '\033[31m'


def _status(source: str, state: dict) -> str:
    conn_col = ANSI_GRN if state['connected'] else ANSI_RED
    conn_str = f'{conn_col}{"CONNECTED" if state["connected"] else "RECONNECTING..."}{ANSI_RST}'
    age      = time.monotonic() - state['last_rx'] if state['last_rx'] else None
    age_str  = (f'{ANSI_GRN}{age:.1f} s ago{ANSI_RST}' if age and age < 5.0
                else f'{ANSI_RED}{age:.0f} s ago{ANSI_RST}' if age
                else f'{ANSI_RED}none{ANSI_RST}')
    lines = [
        f'{ANSI_BOLD}AgriRover RTK Forwarder{ANSI_RST}',
        f'  Source    : {source}',
        f'  Status    : {conn_str}',
        f'  Last data : {age_str}',
        f'  Forwarded : {state["bytes_fwd"]:,} bytes  ({state["chunks"]} chunks)',
        f'  Reconnects: {state["reconnects"]}',
        '',
        '  Ctrl-C to stop',
    ]
    return ANSI_CLR + '\n'.join(lines) + '\n'


# ── Forwarding loop ───────────────────────────────────────────────────────────

def forward_loop(client, writer: GpsWriter, state: dict, stop: threading.Event):
    """Connect → stream → write loop with exponential backoff on failure."""
    backoff = 2.0
    while not stop.is_set():
        try:
            state['connected'] = True
            backoff = 2.0
            for chunk in client.stream():
                if stop.is_set():
                    return
                writer.write(chunk)
                state['last_rx']   = time.monotonic()
                state['bytes_fwd'] += len(chunk)
                state['chunks']    += 1
        except (ConnectionError, OSError, socket.error) as e:
            state['connected'] = False
            state['reconnects'] += 1
            print(f'\n[WARN] Source disconnected: {e}  — retry in {backoff:.0f} s',
                  file=sys.stderr)
            stop.wait(backoff)
            backoff = min(backoff * 2, 60.0)


# ── Args / main ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='RTK RTCM3 forwarder — NTRIP/E610 → u-blox serial',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--source',       required=True, choices=['ntrip', 'e610'],
                   help='Correction source')

    # NTRIP options
    g = p.add_argument_group('NTRIP options (--source ntrip)')
    g.add_argument('--ntrip-host',   default='',    help='NTRIP caster hostname')
    g.add_argument('--ntrip-port',   default=2101,  type=int)
    g.add_argument('--mountpoint',   default='',    help='Mount point name')
    g.add_argument('--ntrip-user',   default='',    help='Username (leave empty if none)')
    g.add_argument('--ntrip-pass',   default='',    help='Password')
    g.add_argument('--approx-lat',   default=0.0,   type=float,
                   help='Rover approx latitude for VRS casters (optional)')
    g.add_argument('--approx-lon',   default=0.0,   type=float,
                   help='Rover approx longitude for VRS casters (optional)')
    g.add_argument('--gga-interval', default=30.0,  type=float, metavar='S',
                   help='Seconds between GGA position reports to caster (0=off)')

    # E610 options
    g2 = p.add_argument_group('E610 DTU options (--source e610)')
    g2.add_argument('--e610-host',   default='192.168.1.20',
                    help='E610 DTU IP address (connect Jetson Ethernet → E610 LAN)')
    g2.add_argument('--e610-port',   default=9000, type=int,
                    help='E610 TCP server port')

    # GPS serial output
    p.add_argument('--gps-ports',    nargs='+',
                   default=['/dev/ttyUSB0', '/dev/ttyUSB1'],
                   metavar='PORT',
                   help='Serial ports of u-blox GPS modules (space-separated)')
    p.add_argument('--gps-baud',     default=115200, type=int)

    return p.parse_args()


def main():
    args = parse_args()

    print('\nAgriRover RTK Forwarder')
    writer = GpsWriter(args.gps_ports, args.gps_baud)

    if args.source == 'ntrip':
        if not args.ntrip_host or not args.mountpoint:
            sys.exit('[ERR] --source ntrip requires --ntrip-host and --mountpoint')
        client = NtripClient(
            host         = args.ntrip_host,
            port         = args.ntrip_port,
            mountpoint   = args.mountpoint,
            user         = args.ntrip_user,
            password     = args.ntrip_pass,
            approx_lat   = args.approx_lat,
            approx_lon   = args.approx_lon,
            gga_interval = args.gga_interval,
        )
        source_str = f'NTRIP  {args.ntrip_host}:{args.ntrip_port}/{args.mountpoint}'
    else:
        client     = E610Client(args.e610_host, args.e610_port)
        source_str = f'E610 DTU  {args.e610_host}:{args.e610_port}'

    state = {
        'connected': False, 'last_rx': None,
        'bytes_fwd': 0,     'chunks': 0,
        'reconnects': 0,
    }

    stop = threading.Event()
    signal.signal(signal.SIGINT,  lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    fwd_thread = threading.Thread(
        target=forward_loop, args=(client, writer, state, stop), daemon=True)
    fwd_thread.start()

    print(f'\nForwarding {source_str} → {args.gps_ports}')
    print('Ctrl-C to stop\n')
    time.sleep(0.5)

    while not stop.is_set():
        sys.stdout.write(_status(source_str, state))
        sys.stdout.flush()
        time.sleep(1.0)

    stop.set()
    fwd_thread.join(timeout=3.0)
    writer.close()
    print('\n\nForwarder stopped.')


if __name__ == '__main__':
    main()
