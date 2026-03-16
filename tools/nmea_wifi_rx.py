#!/usr/bin/env python3
"""
tools/nmea_wifi_rx.py — NMEA WiFi receiver for rover Jetsons

Receives simulated NMEA sentences from simulator.py over UDP WiFi and exposes
them as two virtual serial ports (PTY) that gps_driver.py reads without any
modification — exactly as if two physical u-blox GPS modules were connected.

Run this on each rover Jetson BEFORE launching gps_driver / ROS2 bringup.
It prints the PTY device paths to configure (or pass to launch).

  Simulator Jetson         WiFi            Rover Jetson
  simulator.py  ──── UDP :5000 ──────►  nmea_wifi_rx.py
                ──── UDP :5001 ──────►       │
                                       /dev/pts/2  (primary)   ← gps_driver
                                       /dev/pts/3  (secondary) ← gps_driver

Requirements:
  pip3 install pyserial   (needed only if using --write-serial fallback)

Usage:
  # RV1 Jetson  (simulator sends to rv1-ip:5000 and rv1-ip:5001)
  python3 nmea_wifi_rx.py --pri-port 5000 --sec-port 5001

  # RV2 Jetson  (simulator sends to rv2-ip:5000 and rv2-ip:5001)
  python3 nmea_wifi_rx.py --pri-port 5000 --sec-port 5001

  Both rovers use the same ports — the simulator sends to different IPs.

After startup, configure gps_driver with the printed PTY paths, e.g.:
  ros2 launch agri_rover_bringup rover1.launch.py \\
    primary_port:=/dev/pts/2 secondary_port:=/dev/pts/3

Or edit rover1_params.yaml:
  gps_driver:
    ros__parameters:
      primary_port:   /dev/pts/2
      secondary_port: /dev/pts/3
"""

from __future__ import annotations

import argparse
import os
import pty
import signal
import socket
import sys
import termios
import threading
import time
import tty


# ── PTY helpers ───────────────────────────────────────────────────────────────

def _open_pty_raw() -> tuple[int, str]:
    """Open a PTY pair, set master to raw mode. Returns (master_fd, slave_path)."""
    master_fd, slave_fd = pty.openpty()
    # Raw mode: disable all terminal processing so binary/NMEA data passes through
    tty.setraw(master_fd)
    slave_path = os.ttyname(slave_fd)
    # Keep slave_fd open in this process so the master stays valid when gps_driver opens it
    # Store it as an attribute on the fd int via a global to prevent GC
    _open_pty_raw._keep.append(slave_fd)
    return master_fd, slave_path

_open_pty_raw._keep = []   # prevent slave_fd from being GC'd / closed


# ── UDP → PTY bridge thread ───────────────────────────────────────────────────

class UdpPtyBridge:
    """Listens on a UDP port, writes every received datagram to a PTY master fd."""

    def __init__(self, bind_ip: str, port: int, master_fd: int, label: str):
        self._master_fd = master_fd
        self._label     = label
        self._lock      = threading.Lock()
        self._last_rx   = 0.0
        self._rx_count  = 0

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((bind_ip, port))
        sock.settimeout(1.0)
        self._sock = sock

        threading.Thread(target=self._run, daemon=True).start()

    @property
    def age(self) -> float:
        with self._lock:
            if self._last_rx == 0.0:
                return 999.0
            return time.monotonic() - self._last_rx

    @property
    def rx_count(self) -> int:
        with self._lock:
            return self._rx_count

    def _run(self):
        while True:
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                os.write(self._master_fd, data)
                with self._lock:
                    self._last_rx = time.monotonic()
                    self._rx_count += 1
            except OSError as e:
                print(f'\n[ERR] {self._label} PTY write: {e}', file=sys.stderr)
                break


# ── Status display ────────────────────────────────────────────────────────────

ANSI_RST  = '\033[0m'
ANSI_GRN  = '\033[32m'
ANSI_YLW  = '\033[33m'
ANSI_RED  = '\033[31m'


def _age_str(age: float) -> str:
    if age < 0.5:  return f'{ANSI_GRN}{age*1000:.0f} ms{ANSI_RST}'
    if age < 2.0:  return f'{ANSI_YLW}{age*1000:.0f} ms{ANSI_RST}'
    return f'{ANSI_RED}NO DATA ({age:.0f} s){ANSI_RST}'


def _status(pri: UdpPtyBridge, sec: UdpPtyBridge,
            pri_pty: str, sec_pty: str, args) -> str:
    ts = time.strftime('%H:%M:%S')
    pri_s = _age_str(pri.age)
    sec_s = _age_str(sec.age)
    return (f'[{ts}] pri={pri_pty} {pri_s} ({pri.rx_count} pkts)  '
            f'sec={sec_pty} {sec_s} ({sec.rx_count} pkts)\n')


# ── Args / main ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='NMEA WiFi receiver — creates virtual serial ports for gps_driver',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--bind-ip',   default='0.0.0.0',
                   help='IP to bind (0.0.0.0 = all interfaces)')
    p.add_argument('--pri-port',  default=5000, type=int,
                   help='UDP port for primary GPS stream')
    p.add_argument('--sec-port',  default=5001, type=int,
                   help='UDP port for secondary GPS stream')
    p.add_argument('--pri-symlink', default='',
                   help='Create a symlink at this path pointing to the primary PTY '
                        '(e.g. /tmp/rv1_gps_pri) so callers can use a fixed path')
    p.add_argument('--sec-symlink', default='',
                   help='Create a symlink at this path pointing to the secondary PTY')
    return p.parse_args()


def main():
    args = parse_args()

    # Create virtual serial ports
    pri_master, pri_pty = _open_pty_raw()
    sec_master, sec_pty = _open_pty_raw()

    print('\nAgriRover NMEA WiFi Receiver')
    print(f'\n  Virtual serial ports created:')
    print(f'    primary_port:   {pri_pty}')
    print(f'    secondary_port: {sec_pty}')
    print(f'\n  Listening for UDP NMEA:')
    print(f'    primary   {args.bind_ip}:{args.pri_port}')
    print(f'    secondary {args.bind_ip}:{args.sec_port}')
    print(f'\n  Add to rover params YAML:')
    print(f'    gps_driver:')
    print(f'      ros__parameters:')
    print(f'        primary_port:   {pri_pty}')
    print(f'        secondary_port: {sec_pty}')
    print()

    # Create fixed-path symlinks so other processes can use a predictable path
    def _make_symlink(target: str, link: str):
        if not link:
            return
        try:
            os.unlink(link)
        except FileNotFoundError:
            pass
        os.symlink(target, link)
        print(f'    symlink: {link} -> {target}')

    _make_symlink(pri_pty, args.pri_symlink)
    _make_symlink(sec_pty, args.sec_symlink)

    pri_bridge = UdpPtyBridge(args.bind_ip, args.pri_port, pri_master, 'primary')
    sec_bridge = UdpPtyBridge(args.bind_ip, args.sec_port, sec_master, 'secondary')

    stop = threading.Event()
    signal.signal(signal.SIGINT,  lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    time.sleep(0.5)

    while not stop.is_set():
        sys.stdout.write(_status(pri_bridge, sec_bridge, pri_pty, sec_pty, args))
        sys.stdout.flush()
        time.sleep(1.0)

    print('\n\nReceiver stopped.')
    os.close(pri_master)
    os.close(sec_master)


if __name__ == '__main__':
    main()
