"""
agri_rover_gps — gps_driver node

Reads two uBlox RTK GPS receivers over serial (NMEA 0183).
  Primary GPS   → position (GGA), speed (VTG)
  Secondary GPS → position used to compute heading via baseline vector

Publishes:
  ~/fix          (sensor_msgs/NavSatFix)   — primary position, 5 Hz
  ~/heading      (std_msgs/Float32)        — degrees from north, 5 Hz
  ~/rtk_status   (std_msgs/String)         — NO_FIX|GPS|DGPS|RTK_FLT|RTK_FIX
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import NavSatFix, NavSatStatus

try:
    import serial
except ImportError:
    raise ImportError("pip install pyserial")


FIX_QUALITY = {
    '0': 'NO_FIX',
    '1': 'GPS',
    '2': 'DGPS',
    '4': 'RTK_FIX',
    '5': 'RTK_FLT',
}


class GpsDriverNode(Node):

    def __init__(self):
        super().__init__('gps_driver')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('primary_port',   '/dev/ttyUSB0')
        self.declare_parameter('secondary_port', '/dev/ttyUSB1')
        self.declare_parameter('baud',           115200)
        # heading_source: 'baseline' (secondary GPS vector) | 'vtg' (primary VTG COG)
        # Use 'vtg' when a single serial port provides both GGA and VTG (e.g. simulator).
        self.declare_parameter('heading_source', 'baseline')

        primary_port   = self.get_parameter('primary_port').value
        secondary_port = self.get_parameter('secondary_port').value
        baud           = self.get_parameter('baud').value
        self._heading_source = self.get_parameter('heading_source').value

        # ── Publishers ───────────────────────────────────────────────────────
        self.fix_pub    = self.create_publisher(NavSatFix, 'fix',        10)
        self.head_pub   = self.create_publisher(Float32,   'heading',    10)
        self.status_pub = self.create_publisher(String,    'rtk_status', 10)

        # ── GPS state ────────────────────────────────────────────────────────
        self._primary       = {'lat': 0.0, 'lon': 0.0, 'fix': '0', 'hdop': 99.9}
        self._secondary     = {'lat': 0.0, 'lon': 0.0}
        self._vtg_heading   = None   # degrees, filled when heading_source=='vtg'
        self._lock          = threading.Lock()

        # ── Serial threads ───────────────────────────────────────────────────
        self._start_reader(primary_port, baud, is_primary=True)
        if self._heading_source == 'baseline':
            self._start_reader(secondary_port, baud, is_primary=False)

        # ── Publish timer (5 Hz) ─────────────────────────────────────────────
        self.create_timer(0.2, self._publish)

        self.get_logger().info(
            f'GPS driver on {primary_port} (primary), '
            f'heading_source={self._heading_source}'
            + (f', secondary={secondary_port}' if self._heading_source == 'baseline' else '')
        )

    # ── Serial reader thread ──────────────────────────────────────────────────

    def _start_reader(self, port: str, baud: int, is_primary: bool):
        def run():
            ser = None
            while rclpy.ok() and ser is None:
                try:
                    ser = serial.Serial(port, baud, timeout=1.0)
                except serial.SerialException as e:
                    self.get_logger().warning(f'GPS port {port} not ready, retrying in 5 s: {e}')
                    time.sleep(5.0)
            if ser is None:
                return
            self.get_logger().info(f'GPS port {port} opened')
            while rclpy.ok():
                try:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                except serial.SerialException:
                    break
                self._parse_nmea(line, is_primary)

        t = threading.Thread(target=run, daemon=True)
        t.start()

    # ── NMEA parsing ─────────────────────────────────────────────────────────

    def _parse_nmea(self, sentence: str, is_primary: bool):
        if not sentence.startswith('$'):
            return
        parts = sentence.split(',')
        msg_type = parts[0][1:]   # strip leading '$'

        if msg_type in ('GPGGA', 'GNGGA') and len(parts) >= 10:
            self._parse_gga(parts, is_primary)
        elif msg_type in ('GPVTG', 'GNVTG') and is_primary:
            self._parse_vtg(parts)

    def _parse_gga(self, parts: list, is_primary: bool):
        try:
            raw_lat = parts[2]; lat_hem = parts[3]
            raw_lon = parts[4]; lon_hem = parts[5]
            fix_q   = parts[6]
            hdop    = float(parts[8]) if parts[8] else 99.9

            lat = self._nmea_to_deg(raw_lat) * (-1 if lat_hem == 'S' else 1)
            lon = self._nmea_to_deg(raw_lon) * (-1 if lon_hem == 'W' else 1)

            with self._lock:
                if is_primary:
                    self._primary.update({'lat': lat, 'lon': lon,
                                          'fix': fix_q, 'hdop': hdop})
                else:
                    self._secondary.update({'lat': lat, 'lon': lon})
        except (ValueError, IndexError):
            pass

    def _parse_vtg(self, parts: list):
        # $GNVTG,COG_T,T,COG_M,M,SPEED_KN,N,SPEED_KMH,K,mode*CS
        # Field 1 = true COG (degrees), field 2 = 'T'
        try:
            cog_str = parts[1]
            if cog_str and self._heading_source == 'vtg':
                cog = float(cog_str) % 360.0
                with self._lock:
                    self._vtg_heading = cog
        except (ValueError, IndexError):
            pass

    @staticmethod
    def _nmea_to_deg(raw: str) -> float:
        if not raw:
            return 0.0
        dot = raw.index('.')
        deg = float(raw[:dot - 2])
        mins = float(raw[dot - 2:])
        return deg + mins / 60.0

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        with self._lock:
            p           = self._primary.copy()
            s           = self._secondary.copy()
            vtg_heading = self._vtg_heading

        now = self.get_clock().now().to_msg()

        # NavSatFix
        fix_msg                 = NavSatFix()
        fix_msg.header.stamp    = now
        fix_msg.header.frame_id = 'gps'
        fix_msg.latitude        = p['lat']
        fix_msg.longitude       = p['lon']
        fix_msg.altitude        = 0.0   # TODO: parse altitude from GGA
        status_map = {'0': NavSatStatus.STATUS_NO_FIX,
                      '1': NavSatStatus.STATUS_FIX,
                      '2': NavSatStatus.STATUS_SBAS_FIX,
                      '4': NavSatStatus.STATUS_GBAS_FIX,
                      '5': NavSatStatus.STATUS_GBAS_FIX}
        fix_msg.status.status  = status_map.get(p['fix'], NavSatStatus.STATUS_NO_FIX)
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        self.fix_pub.publish(fix_msg)

        # Heading — baseline vector or VTG COG depending on heading_source
        if self._heading_source == 'vtg':
            if vtg_heading is not None:
                head_msg      = Float32()
                head_msg.data = vtg_heading
                self.head_pub.publish(head_msg)
        else:
            if p['lat'] != 0.0 and s['lat'] != 0.0:
                dlat = s['lat'] - p['lat']
                dlon = s['lon'] - p['lon']
                lat_rad = math.radians(p['lat'])
                bearing = math.degrees(math.atan2(dlon * math.cos(lat_rad), dlat)) % 360.0
                head_msg      = Float32()
                head_msg.data = bearing
                self.head_pub.publish(head_msg)

        # RTK status string
        status_msg      = String()
        status_msg.data = FIX_QUALITY.get(p['fix'], 'UNKNOWN')
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
