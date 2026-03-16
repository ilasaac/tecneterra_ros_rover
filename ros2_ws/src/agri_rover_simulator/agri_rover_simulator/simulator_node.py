"""
agri_rover_simulator — simulator_node

Dead-reckoning GPS simulator for AgriRover development.

Hardware topology (simulator Jetson Nano, with USB hub):
  /dev/ttyACM0  ← ppm_decoder RP2040
                   reads: "RV1:ch0,ch1,...,ch7 RV2:ch0,...,ch7\\n" at ~50 Hz
  /dev/ttyUSB0  → RV1 Jetson /dev/ttyUSB0  (primary GPS)
  /dev/ttyUSB1  → RV1 Jetson /dev/ttyUSB1  (secondary GPS → baseline heading)
  /dev/ttyUSB2  → RV2 Jetson /dev/ttyUSB0  (primary GPS)
  /dev/ttyUSB3  → RV2 Jetson /dev/ttyUSB1  (secondary GPS → baseline heading)

Each rover Jetson runs gps_driver normally (heading_source: baseline).
The secondary GPS stream carries a position offset `antenna_baseline_m` ahead
of the primary along the rover's current heading — gps_driver then recovers
that heading via atan2(dlon, dlat).

Dead-reckoning model (bicycle kinematic):
  throttle_ppm: 1500 = stop, 2000 = max forward, 1000 = max reverse
  steering_ppm: 1500 = straight, 2000 = full right, 1000 = full left
  speed  = (throttle - 1500) / 500 * max_speed_mps        [m/s]
  steer  = (steering  - 1500) / 500                        [-1..+1]
  omega  = speed * steer / wheelbase_m                     [rad/s]
  heading += omega * dt
  lat += speed * cos(heading) * dt / 111320
  lon += speed * sin(heading) * dt / (111320 * cos(lat_rad))

NMEA output (u-blox ZED-X20P RTK quality emulation):
  $GNGGA,...,4,...   fix quality 4 = RTK fixed
  $GNVTG,...         true course over ground + speed (informational)

Publishes (for monitoring via ros2 topic echo):
  /sim/rv1/fix   (sensor_msgs/NavSatFix)
  /sim/rv2/fix   (sensor_msgs/NavSatFix)
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

try:
    import serial
except ImportError:
    raise ImportError("pip install pyserial")


# ── NMEA helpers ──────────────────────────────────────────────────────────────

def _nmea_checksum(body: str) -> str:
    """XOR checksum of characters between '$' and '*' (exclusive)."""
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f'{cs:02X}'


def _format_lat(lat: float) -> tuple[str, str]:
    hem = 'N' if lat >= 0.0 else 'S'
    lat = abs(lat)
    d = int(lat)
    m = (lat - d) * 60.0
    return f'{d:02d}{m:010.7f}', hem


def _format_lon(lon: float) -> tuple[str, str]:
    hem = 'E' if lon >= 0.0 else 'W'
    lon = abs(lon)
    d = int(lon)
    m = (lon - d) * 60.0
    return f'{d:03d}{m:010.7f}', hem


def _make_gga(lat: float, lon: float, alt: float = 45.0) -> bytes:
    """Build a $GNGGA sentence with RTK fixed quality (4), 12 sats, 0.5 HDOP."""
    t = time.gmtime()
    utc = f'{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.00'
    lat_s, lat_h = _format_lat(lat)
    lon_s, lon_h = _format_lon(lon)
    # quality=4 (RTK fixed), sats=12, HDOP=0.5, alt, geoid sep=0
    body = f'GNGGA,{utc},{lat_s},{lat_h},{lon_s},{lon_h},4,12,0.5,{alt:.1f},M,0.0,M,0.5,0001'
    sentence = f'${body}*{_nmea_checksum(body)}\r\n'
    return sentence.encode('ascii')


def _make_vtg(heading_deg: float, speed_mps: float) -> bytes:
    """Build a $GNVTG sentence with true COG and speed."""
    speed_kn  = speed_mps * 1.94384
    speed_kmh = speed_mps * 3.6
    cog = heading_deg % 360.0
    body = (f'GNVTG,{cog:.2f},T,,M,'
            f'{speed_kn:.2f},N,{speed_kmh:.2f},K,D')
    sentence = f'${body}*{_nmea_checksum(body)}\r\n'
    return sentence.encode('ascii')


# ── Rover state ───────────────────────────────────────────────────────────────

class RoverState:
    def __init__(self, lat: float, lon: float, heading_deg: float):
        self.lat         = lat
        self.lon         = lon
        self.heading_rad = math.radians(heading_deg)
        self.speed_mps   = 0.0   # current speed, updated each tick

    def update(self, throttle_ppm: int, steering_ppm: int,
               max_speed: float, wheelbase: float, dt: float):
        speed = (throttle_ppm - 1500) / 500.0 * max_speed
        steer = (steering_ppm - 1500) / 500.0       # -1..+1
        omega = (speed * steer / wheelbase) if wheelbase > 0 else 0.0

        self.heading_rad += omega * dt
        lat_rad = math.radians(self.lat)
        self.lat += (speed * math.cos(self.heading_rad) * dt) / 111320.0
        self.lon += (speed * math.sin(self.heading_rad) * dt) / (
            111320.0 * math.cos(lat_rad) if math.cos(lat_rad) != 0 else 1.0)
        self.speed_mps = speed

    def secondary_position(self, baseline_m: float) -> tuple[float, float]:
        """Return (lat, lon) of secondary antenna, offset `baseline_m` ahead."""
        lat_rad = math.radians(self.lat)
        sec_lat = self.lat + (baseline_m * math.cos(self.heading_rad)) / 111320.0
        sec_lon = self.lon + (baseline_m * math.sin(self.heading_rad)) / (
            111320.0 * math.cos(lat_rad) if math.cos(lat_rad) != 0 else 1.0)
        return sec_lat, sec_lon

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.heading_rad) % 360.0


# ── Simulator node ────────────────────────────────────────────────────────────

class SimulatorNode(Node):

    def __init__(self):
        super().__init__('simulator')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('ppm_port',           '/dev/ttyACM0')
        self.declare_parameter('ppm_baud',           115200)
        self.declare_parameter('rv1_primary_port',   '/dev/ttyUSB0')
        self.declare_parameter('rv1_secondary_port', '/dev/ttyUSB1')
        self.declare_parameter('rv2_primary_port',   '/dev/ttyUSB2')
        self.declare_parameter('rv2_secondary_port', '/dev/ttyUSB3')
        self.declare_parameter('nmea_baud',          115200)
        self.declare_parameter('rv1_start_lat',      0.0)
        self.declare_parameter('rv1_start_lon',      0.0)
        self.declare_parameter('rv1_start_heading_deg', 0.0)
        self.declare_parameter('rv2_start_lat',      0.0)
        self.declare_parameter('rv2_start_lon',      0.0)
        self.declare_parameter('rv2_start_heading_deg', 0.0)
        self.declare_parameter('max_speed_mps',      1.5)
        self.declare_parameter('wheelbase_m',        0.5)
        self.declare_parameter('antenna_baseline_m', 0.3)
        self.declare_parameter('nmea_rate_hz',       10.0)
        self.declare_parameter('throttle_ch',        0)
        self.declare_parameter('steering_ch',        1)

        p = self.get_parameter
        self._max_speed  = p('max_speed_mps').value
        self._wheelbase  = p('wheelbase_m').value
        self._baseline   = p('antenna_baseline_m').value
        self._thr_ch     = p('throttle_ch').value
        self._str_ch     = p('steering_ch').value

        nmea_rate = p('nmea_rate_hz').value
        self._dt = 1.0 / nmea_rate

        # ── Rover states ─────────────────────────────────────────────────────
        self._rv1 = RoverState(p('rv1_start_lat').value,
                               p('rv1_start_lon').value,
                               p('rv1_start_heading_deg').value)
        self._rv2 = RoverState(p('rv2_start_lat').value,
                               p('rv2_start_lon').value,
                               p('rv2_start_heading_deg').value)

        # ── PPM decoder state (updated by reader thread) ──────────────────────
        self._rv1_ch = [1500] * 8
        self._rv2_ch = [1500] * 8
        self._ppm_lock = threading.Lock()
        self._last_ppm = 0.0   # monotonic time of last valid ppm frame

        # ── Publishers (monitoring) ───────────────────────────────────────────
        self._rv1_fix_pub = self.create_publisher(NavSatFix, '/sim/rv1/fix', 10)
        self._rv2_fix_pub = self.create_publisher(NavSatFix, '/sim/rv2/fix', 10)

        # ── Open NMEA output serial ports ─────────────────────────────────────
        nmea_baud = p('nmea_baud').value
        self._nmea_ports: dict[str, serial.Serial | None] = {}
        for key in ('rv1_primary_port', 'rv1_secondary_port',
                    'rv2_primary_port', 'rv2_secondary_port'):
            port = p(key).value
            try:
                self._nmea_ports[key] = serial.Serial(port, nmea_baud, timeout=1.0)
                self.get_logger().info(f'Opened NMEA output: {key} → {port}')
            except serial.SerialException as e:
                self.get_logger().error(f'Cannot open {key} ({port}): {e}')
                self._nmea_ports[key] = None

        # ── Start ppm_decoder reader thread ──────────────────────────────────
        ppm_port = p('ppm_port').value
        ppm_baud = p('ppm_baud').value
        self._start_ppm_reader(ppm_port, ppm_baud)

        # ── Physics + NMEA output timer ───────────────────────────────────────
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'Simulator ready — physics at {nmea_rate:.0f} Hz, '
            f'baseline={self._baseline:.2f} m, wheelbase={self._wheelbase:.2f} m')

    # ── PPM reader thread ─────────────────────────────────────────────────────

    def _start_ppm_reader(self, port: str, baud: int):
        def run():
            try:
                ser = serial.Serial(port, baud, timeout=1.0)
                self.get_logger().info(f'PPM decoder opened: {port}')
            except serial.SerialException as e:
                self.get_logger().error(f'Cannot open ppm_port {port}: {e}')
                return
            while rclpy.ok():
                try:
                    raw = ser.readline().decode('ascii', errors='ignore').strip()
                except serial.SerialException:
                    break
                self._parse_ppm_line(raw)

        t = threading.Thread(target=run, daemon=True)
        t.start()

    def _parse_ppm_line(self, line: str):
        # Format: "RV1:ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7 RV2:ch0,...,ch7"
        try:
            rv1_part, rv2_part = line.split(' ')
            rv1_vals = [int(v) for v in rv1_part[4:].split(',')]  # skip "RV1:"
            rv2_vals = [int(v) for v in rv2_part[4:].split(',')]  # skip "RV2:"
            if len(rv1_vals) == 8 and len(rv2_vals) == 8:
                with self._ppm_lock:
                    self._rv1_ch = rv1_vals
                    self._rv2_ch = rv2_vals
                    self._last_ppm = time.monotonic()
        except (ValueError, AttributeError):
            pass   # malformed line — skip

    # ── Physics tick ─────────────────────────────────────────────────────────

    def _tick(self):
        with self._ppm_lock:
            rv1_ch = list(self._rv1_ch)
            rv2_ch = list(self._rv2_ch)
            ppm_age = time.monotonic() - self._last_ppm

        # Neutral if no ppm data received in last 2 seconds
        if ppm_age > 2.0:
            rv1_ch = [1500] * 8
            rv2_ch = [1500] * 8

        self._rv1.update(rv1_ch[self._thr_ch], rv1_ch[self._str_ch],
                         self._max_speed, self._wheelbase, self._dt)
        self._rv2.update(rv2_ch[self._thr_ch], rv2_ch[self._str_ch],
                         self._max_speed, self._wheelbase, self._dt)

        self._send_nmea(self._rv1,
                        self._nmea_ports['rv1_primary_port'],
                        self._nmea_ports['rv1_secondary_port'])
        self._send_nmea(self._rv2,
                        self._nmea_ports['rv2_primary_port'],
                        self._nmea_ports['rv2_secondary_port'])

        self._publish_fix(self._rv1, self._rv1_fix_pub)
        self._publish_fix(self._rv2, self._rv2_fix_pub)

    def _send_nmea(self, rover: RoverState,
                   primary_ser: 'serial.Serial | None',
                   secondary_ser: 'serial.Serial | None'):
        gga_primary = _make_gga(rover.lat, rover.lon)
        vtg         = _make_vtg(rover.heading_deg, rover.speed_mps)

        sec_lat, sec_lon = rover.secondary_position(self._baseline)
        gga_secondary = _make_gga(sec_lat, sec_lon)

        if primary_ser is not None:
            try:
                primary_ser.write(gga_primary)
                primary_ser.write(vtg)
            except serial.SerialException:
                pass

        if secondary_ser is not None:
            try:
                secondary_ser.write(gga_secondary)
            except serial.SerialException:
                pass

    def _publish_fix(self, rover: RoverState, pub):
        msg                 = NavSatFix()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude        = rover.lat
        msg.longitude       = rover.lon
        msg.altitude        = 45.0
        msg.status.status   = NavSatStatus.STATUS_GBAS_FIX   # RTK fixed
        msg.status.service  = NavSatStatus.SERVICE_GPS
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
