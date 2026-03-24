#!/usr/bin/env python3
"""
Dynamics data collector for AgriRover model tuning.

Manually drive the rover while this script records:
  - PPM channels (throttle CH1, steering CH2) from RP2040 serial
  - GPS position (lat, lon) and heading from NMEA UDP
  - Computed speed and turn rate

Output: CSV file with all data for offline analysis to tune DiffDriveState
parameters (track width, max wheel speed, acceleration, turn response).

Usage (on the Jetson, with RP2040 connected):
    python3 tools/dynamics_collector.py --serial /dev/ttyACM0 \\
        --gps-port 5801 --output dynamics_log.csv

Usage (simulation — reads NMEA from simulator UDP):
    python3 tools/dynamics_collector.py --serial /dev/ttyACM0 \\
        --gps-port 5801 --output dynamics_log.csv

Press Ctrl+C to stop recording and save the file.
"""

import argparse
import csv
import math
import socket
import sys
import threading
import time

try:
    import serial
except ImportError:
    serial = None


# ── NMEA parser (minimal — only GGA and VTG) ────────────────────────────

def _parse_gga(fields):
    """Parse $GNGGA fields → (lat, lon, fix_quality, num_sats, hdop, alt) or None."""
    if len(fields) < 10:
        return None
    try:
        raw_lat = fields[2]
        lat_hem = fields[3]
        raw_lon = fields[4]
        lon_hem = fields[5]
        fix_q   = int(fields[6]) if fields[6] else 0
        n_sats  = int(fields[7]) if fields[7] else 0
        hdop    = float(fields[8]) if fields[8] else 99.9
        alt     = float(fields[9]) if fields[9] else 0.0

        # Convert NMEA ddmm.mmmm → decimal degrees
        lat_deg = int(raw_lat[:2])
        lat_min = float(raw_lat[2:])
        lat = lat_deg + lat_min / 60.0
        if lat_hem == 'S':
            lat = -lat

        lon_deg = int(raw_lon[:3])
        lon_min = float(raw_lon[3:])
        lon = lon_deg + lon_min / 60.0
        if lon_hem == 'W':
            lon = -lon

        return lat, lon, fix_q, n_sats, hdop, alt
    except (ValueError, IndexError):
        return None


def _parse_vtg(fields):
    """Parse $GNVTG fields → (cog_deg, speed_kmh) or None."""
    if len(fields) < 8:
        return None
    try:
        cog = float(fields[1]) if fields[1] else None
        spd = float(fields[7]) if fields[7] else 0.0
        return cog, spd
    except (ValueError, IndexError):
        return None


# ── Haversine / bearing ──────────────────────────────────────────────────

def _haversine(lat1, lon1, lat2, lon2):
    """Distance in metres between two lat/lon points."""
    R = 6_371_000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _bearing_to(lat1, lon1, lat2, lon2):
    """Bearing in degrees [0, 360) from point 1 to point 2."""
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(math.radians(lat2))
    x = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
         math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon))
    return (math.degrees(math.atan2(y, x)) + 360) % 360


# ── Shared state ─────────────────────────────────────────────────────────

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        # PPM
        self.channels = [1500] * 16
        self.mode = 'UNKNOWN'
        self.ppm_stamp = 0.0
        # GPS primary
        self.lat = 0.0
        self.lon = 0.0
        self.fix_quality = 0
        self.num_sats = 0
        self.hdop = 99.9
        self.alt = 0.0
        self.gps_stamp = 0.0
        # GPS secondary (for heading)
        self.lat2 = 0.0
        self.lon2 = 0.0
        self.gps2_stamp = 0.0
        # Derived
        self.heading = 0.0        # degrees, from dual-antenna baseline
        self.gps_speed = 0.0      # m/s, from position deltas
        self.turn_rate = 0.0      # deg/s
        # Previous values for derivative computation
        self._prev_lat = None
        self._prev_lon = None
        self._prev_heading = None
        self._prev_time = None
        # Running flag
        self.running = True


# ── Serial reader (RP2040) ───────────────────────────────────────────────

def serial_reader(state, port, baud):
    """Read CH: lines from RP2040 serial."""
    if serial is None:
        print('[WARN] pyserial not installed — PPM data will be zeros')
        return

    while state.running:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f'[SERIAL] Connected to {port} @ {baud}')
            while state.running:
                raw = ser.readline().decode('ascii', errors='ignore').strip()
                if not raw:
                    continue
                if raw.startswith('CH:') and ' MODE:' in raw:
                    try:
                        ch_part, mode_str = raw.split(' MODE:')
                        channels = [int(x) for x in ch_part[3:].split(',')]
                        with state.lock:
                            state.channels = channels + [1500] * max(0, 16 - len(channels))
                            state.mode = mode_str.strip()
                            state.ppm_stamp = time.monotonic()
                    except (ValueError, IndexError):
                        pass
        except Exception as e:
            print(f'[SERIAL] Error: {e} — retrying in 2s')
            time.sleep(2)


# ── GPS UDP reader ───────────────────────────────────────────────────────

def gps_reader(state, port, is_secondary=False):
    """Read NMEA sentences from UDP."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', port))
    sock.settimeout(1.0)
    label = 'GPS2' if is_secondary else 'GPS1'
    print(f'[{label}] Listening on UDP :{port}')

    while state.running:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except Exception as e:
            print(f'[{label}] Error: {e}')
            continue

        for line in data.decode('ascii', errors='ignore').split('\n'):
            line = line.strip()
            if not line.startswith('$'):
                continue
            # Strip checksum
            if '*' in line:
                line = line[:line.index('*')]
            fields = line.split(',')
            sentence = fields[0]

            if 'GGA' in sentence:
                result = _parse_gga(fields)
                if result is None:
                    continue
                lat, lon, fix_q, n_sats, hdop, alt = result
                now = time.monotonic()

                with state.lock:
                    if is_secondary:
                        state.lat2 = lat
                        state.lon2 = lon
                        state.gps2_stamp = now
                        # Compute heading from baseline
                        if state.gps_stamp > 0:
                            state.heading = _bearing_to(
                                state.lat, state.lon, lat, lon)
                    else:
                        # Compute speed and turn rate from position deltas
                        if state._prev_lat is not None and state._prev_time is not None:
                            dt = now - state._prev_time
                            if dt > 0.01:
                                dist = _haversine(state._prev_lat, state._prev_lon,
                                                  lat, lon)
                                state.gps_speed = dist / dt

                                if state._prev_heading is not None:
                                    dh = ((state.heading - state._prev_heading + 180) % 360) - 180
                                    state.turn_rate = dh / dt

                        state._prev_lat = state.lat
                        state._prev_lon = state.lon
                        state._prev_heading = state.heading
                        state._prev_time = now

                        state.lat = lat
                        state.lon = lon
                        state.fix_quality = fix_q
                        state.num_sats = n_sats
                        state.hdop = hdop
                        state.alt = alt
                        state.gps_stamp = now


# ── ROS2 topic reader (alternative to serial+UDP) ───────────────────────

def ros2_reader(state, namespace):
    """Read from ROS2 topics instead of raw serial/UDP."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import Float32
        from agri_rover_interfaces.msg import RCInput
    except ImportError:
        print('[ERROR] rclpy not available — use --serial/--gps-port instead')
        return

    rclpy.init()

    class CollectorNode(Node):
        def __init__(self):
            super().__init__('dynamics_collector')
            self.create_subscription(
                RCInput, f'{namespace}/rc_input', self._cb_rc, 10)
            self.create_subscription(
                NavSatFix, f'{namespace}/fix', self._cb_fix, 10)
            self.create_subscription(
                Float32, f'{namespace}/heading', self._cb_heading, 10)

        def _cb_rc(self, msg):
            with state.lock:
                state.channels = list(msg.channels)
                state.mode = msg.mode
                state.ppm_stamp = time.monotonic()

        def _cb_fix(self, msg):
            now = time.monotonic()
            with state.lock:
                if state._prev_lat is not None and state._prev_time is not None:
                    dt = now - state._prev_time
                    if dt > 0.01:
                        dist = _haversine(state._prev_lat, state._prev_lon,
                                          msg.latitude, msg.longitude)
                        state.gps_speed = dist / dt

                state._prev_lat = state.lat
                state._prev_lon = state.lon
                state._prev_time = now
                state.lat = msg.latitude
                state.lon = msg.longitude
                state.gps_stamp = now

        def _cb_heading(self, msg):
            now = time.monotonic()
            with state.lock:
                if state._prev_heading is not None and state._prev_time is not None:
                    dt = now - state._prev_time
                    if dt > 0.01:
                        dh = ((msg.data - state._prev_heading + 180) % 360) - 180
                        state.turn_rate = dh / dt
                state._prev_heading = state.heading
                state.heading = msg.data

    node = CollectorNode()
    print(f'[ROS2] Subscribing to {namespace}/rc_input, fix, heading')
    try:
        while state.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── Main ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Collect dynamics data for rover model tuning')
    parser.add_argument('--serial', default=None,
                        help='RP2040 serial port (e.g. /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Serial baud rate (default: 115200)')
    parser.add_argument('--gps-port', type=int, default=None,
                        help='UDP port for primary GPS NMEA')
    parser.add_argument('--gps2-port', type=int, default=None,
                        help='UDP port for secondary GPS NMEA (heading)')
    parser.add_argument('--ros2', default=None, metavar='NAMESPACE',
                        help='Use ROS2 topics instead (e.g. /rv1)')
    parser.add_argument('--output', '-o', default='dynamics_log.csv',
                        help='Output CSV file (default: dynamics_log.csv)')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='Sampling rate in Hz (default: 20)')
    args = parser.parse_args()

    if not args.serial and not args.ros2:
        print('Specify --serial PORT or --ros2 NAMESPACE')
        sys.exit(1)

    state = SharedState()
    threads = []

    if args.ros2:
        t = threading.Thread(target=ros2_reader, args=(state, args.ros2),
                             daemon=True)
        t.start()
        threads.append(t)
    else:
        if args.serial:
            t = threading.Thread(target=serial_reader,
                                 args=(state, args.serial, args.baud),
                                 daemon=True)
            t.start()
            threads.append(t)
        if args.gps_port:
            t = threading.Thread(target=gps_reader,
                                 args=(state, args.gps_port, False),
                                 daemon=True)
            t.start()
            threads.append(t)
        if args.gps2_port:
            t = threading.Thread(target=gps_reader,
                                 args=(state, args.gps2_port, True),
                                 daemon=True)
            t.start()
            threads.append(t)

    # CSV columns
    columns = [
        'time_s',           # monotonic seconds since start
        'lat', 'lon',       # GPS position (decimal degrees)
        'heading_deg',      # dual-antenna heading (degrees)
        'gps_speed_mps',    # speed from GPS deltas (m/s)
        'turn_rate_dps',    # heading change rate (deg/s)
        'fix_quality',      # GGA fix quality (4=RTK_FIX, 5=RTK_FLT)
        'num_sats',         # number of satellites
        'hdop',             # horizontal dilution of precision
        'throttle_ppm',     # PPM CH1 (forward/reverse)
        'steering_ppm',     # PPM CH2 (left/right)
        'ch3_ppm',          # PPM CH3 (arm/enable)
        'ch4_ppm',          # PPM CH4 (SWA safety)
        'ch5_ppm',          # PPM CH5 (servo 5)
        'ch6_ppm',          # PPM CH6 (servo 6)
        'ch7_ppm',          # PPM CH7 (servo 7)
        'ch8_ppm',          # PPM CH8 (servo 8)
        'mode',             # RP2040 mode string
    ]

    dt = 1.0 / args.rate
    t0 = time.monotonic()
    n_samples = 0

    print(f'\nRecording to {args.output} at {args.rate} Hz')
    print('Manually drive the rover. Press Ctrl+C to stop.\n')
    print('Recommended maneuvers for model identification:')
    print('  1. Straight-line acceleration: neutral → full throttle → hold → neutral')
    print('  2. Straight-line at constant speed: hold for 10+ seconds')
    print('  3. Full-left turn at low speed: hold for a full circle')
    print('  4. Full-right turn at low speed: hold for a full circle')
    print('  5. Slalom: alternate left/right every 3 seconds')
    print('  6. Spin in place: zero throttle, full steering left then right')
    print('  7. Step response: straight-line → sudden full steer → hold')
    print()

    try:
        with open(args.output, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(columns)

            while state.running:
                now = time.monotonic()
                elapsed = now - t0

                with state.lock:
                    row = [
                        f'{elapsed:.3f}',
                        f'{state.lat:.10f}',
                        f'{state.lon:.10f}',
                        f'{state.heading:.2f}',
                        f'{state.gps_speed:.4f}',
                        f'{state.turn_rate:.2f}',
                        state.fix_quality,
                        state.num_sats,
                        f'{state.hdop:.1f}',
                        state.channels[0] if len(state.channels) > 0 else 1500,
                        state.channels[1] if len(state.channels) > 1 else 1500,
                        state.channels[2] if len(state.channels) > 2 else 1500,
                        state.channels[3] if len(state.channels) > 3 else 1500,
                        state.channels[4] if len(state.channels) > 4 else 1500,
                        state.channels[5] if len(state.channels) > 5 else 1500,
                        state.channels[6] if len(state.channels) > 6 else 1500,
                        state.channels[7] if len(state.channels) > 7 else 1500,
                        state.mode,
                    ]
                    spd = state.gps_speed
                    hdg = state.heading
                    tr  = state.turn_rate

                writer.writerow(row)
                n_samples += 1

                # Live status (every 1 second)
                if n_samples % int(args.rate) == 0:
                    print(f'\r  t={elapsed:6.1f}s  samples={n_samples}  '
                          f'spd={spd:.2f}m/s  hdg={hdg:.1f}  '
                          f'turn={tr:+.1f}/s  '
                          f'thr={row[9]}  str={row[10]}  ',
                          end='', flush=True)

                # Sleep until next sample
                next_t = t0 + (n_samples) * dt
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)

    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        elapsed = time.monotonic() - t0
        print(f'\n\nRecording stopped. {n_samples} samples in {elapsed:.1f}s')
        print(f'Saved to: {args.output}')
        print()
        print('Analysis tips:')
        print('  - Plot throttle_ppm vs gps_speed_mps → max speed, acceleration curve')
        print('  - Plot steering_ppm vs turn_rate_dps → steering gain, max turn rate')
        print('  - At constant speed, turn_rate/steering → effective track width:')
        print('      track_width = 2 * speed / (turn_rate_rad_s * PPM_to_steer_ratio)')
        print('  - Step response (sudden steer) → time constant for SmoothSpeed model')
        print('  - Spin-in-place → max angular velocity at zero forward speed')


if __name__ == '__main__':
    main()
