"""
agri_rover_gps — gps_driver node

Reads two uBlox RTK GPS receivers over serial (NMEA 0183).
  Primary GPS   → position (GGA), speed (VTG)
  Secondary GPS → position used to compute heading via baseline vector

Publishes:
  ~/fix          (sensor_msgs/NavSatFix)   — primary position, up to publish_rate Hz
  ~/heading      (std_msgs/Float32)        — degrees from north, up to publish_rate Hz
  ~/rtk_status   (std_msgs/String)         — NO_FIX|GPS|DGPS|RTK_FLT|RTK_FIX

Note: publish_rate (default 25 Hz) is the timer rate; actual publish rate is bounded
by the GPS hardware NMEA output rate. A message is only published when new primary
GGA data has arrived, so the navigator's gps_timeout still fires if the GPS goes silent.
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
        # publish_rate: timer rate (Hz). Set to match GPS hardware NMEA output rate so the
        # navigator gets every fix immediately. The actual publish rate is bounded by the
        # GPS hardware; messages are only sent when new primary GGA data has arrived.
        self.declare_parameter('publish_rate', 25.0)

        primary_port   = self.get_parameter('primary_port').value
        secondary_port = self.get_parameter('secondary_port').value
        baud           = self.get_parameter('baud').value
        self._heading_source = self.get_parameter('heading_source').value
        publish_rate   = self.get_parameter('publish_rate').value
        self._publish_rate   = publish_rate

        # ── Publishers ───────────────────────────────────────────────────────
        self.fix_pub       = self.create_publisher(NavSatFix, 'fix',        10)
        self.fix_front_pub = self.create_publisher(NavSatFix, 'fix_front',  10)
        self.head_pub      = self.create_publisher(Float32,   'heading',    10)
        self.status_pub    = self.create_publisher(String,    'rtk_status', 10)

        # ── GPS state ────────────────────────────────────────────────────────
        self._primary          = {'lat': 0.0, 'lon': 0.0, 'fix': '0', 'hdop': 99.9}
        self._secondary        = {'lat': 0.0, 'lon': 0.0}
        self._vtg_heading      = None   # degrees, filled when heading_source=='vtg'
        self._lock             = threading.Lock()
        self._primary_updated  = False  # set True on each new primary GGA sentence

        # ── Serial threads ───────────────────────────────────────────────────
        self._start_reader(primary_port, baud, is_primary=True)
        if self._heading_source == 'baseline':
            self._start_reader(secondary_port, baud, is_primary=False)

        # ── Publish timer ────────────────────────────────────────────────────
        # Set to match GPS hardware NMEA output rate (default 25 Hz = RTK update rate).
        # Timer fires at publish_rate but _publish() skips if no new primary GGA arrived.
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'GPS driver on {primary_port} (primary), '
            f'heading_source={self._heading_source}, '
            f'publish_rate={publish_rate:.0f} Hz'
            + (f', secondary={secondary_port}' if self._heading_source == 'baseline' else '')
        )

    # ── UBX configuration ────────────────────────────────────────────────────

    def _ubx_configure(self, ser: 'serial.Serial') -> None:
        """Configure u-blox GPS output rate via UBX binary protocol.

        Tries two approaches:
          1. Legacy CFG-RATE + CFG-MSG  (u-blox M8 and older)
          2. CFG-VALSET                 (u-blox F9P / X20P / generation 9+)

        Both are sent; the module applies whichever it understands.
        Non-u-blox receivers silently ignore the bytes.  No ACK is awaited.
        """
        def _build(cls: int, mid: int, payload: bytes) -> bytes:
            body = bytes([cls, mid, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
            ck_a = ck_b = 0
            for b in body:
                ck_a = (ck_a + b) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            return bytes([0xB5, 0x62]) + body + bytes([ck_a, ck_b])

        period_ms = max(25, int(round(1000.0 / self._publish_rate)))

        # ── Legacy (M8 and older) ────────────────────────────────────────────
        # CFG-RATE: measRate=period_ms, navRate=1, timeRef=GPS
        ser.write(_build(0x06, 0x08, bytes([
            period_ms & 0xFF, (period_ms >> 8) & 0xFF,
            0x01, 0x00,
            0x01, 0x00,
        ])))
        time.sleep(0.05)
        # CFG-MSG: GGA on UART1 (idx 1) and USB (idx 3) at rate 1
        ser.write(_build(0x06, 0x01, bytes([0xF0, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00])))
        time.sleep(0.05)

        # ── CFG-VALSET (F9P / X20P / generation 9+) ─────────────────────────
        # Key IDs (little-endian 32-bit):
        #   CFG-RATE-MEAS              0x30210001  U2  measurement period ms
        #   CFG-MSGOUT-NMEA_ID_GGA_UART1  0x20910029  U1  GGA rate on UART1
        #   CFG-MSGOUT-NMEA_ID_GGA_USB    0x2091002B  U1  GGA rate on USB
        def _key(k: int) -> bytes:
            return k.to_bytes(4, 'little')

        valset_payload = (
            bytes([0x00, 0x01, 0x00, 0x00])           # version=0, layer=RAM, reserved
            + _key(0x30210001) + period_ms.to_bytes(2, 'little')  # CFG-RATE-MEAS
            + _key(0x20910029) + bytes([0x01])         # GGA on UART1
            + _key(0x2091002B) + bytes([0x01])         # GGA on USB
        )
        ser.write(_build(0x06, 0x8A, valset_payload))
        time.sleep(0.05)

        # ── PUBX,40 — disable unused NMEA sentences (all u-blox generations) ─
        # GSV (satellite info) outputs 3-4 sentences per fix and is the main
        # source of serial congestion at 25 Hz.  GGA is all the navigator needs.
        def _pubx40(msg_id: str, rate: int = 0) -> bytes:
            body = f'PUBX,40,{msg_id},{rate},{rate},{rate},{rate},{rate},0'
            ck = 0
            for c in body:
                ck ^= ord(c)
            return f'${body}*{ck:02X}\r\n'.encode()

        for sentence in ('GLL', 'GSA', 'GSV', 'RMC', 'GNS', 'GRS', 'GST'):
            ser.write(_pubx40(sentence, 0))   # disable
            time.sleep(0.02)
        # Keep VTG enabled (rate=1) — used by gps_driver when heading_source='vtg'
        ser.write(_pubx40('VTG', 1))
        time.sleep(0.02)

        self.get_logger().info(
            f'UBX configured: {period_ms} ms ({1000 // period_ms} Hz) '
            f'GGA+VTG only (legacy + VALSET + PUBX,40)'
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
            self._ubx_configure(ser)
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
                    self._primary_updated = True
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
            if not self._primary_updated:
                return   # no new GPS sentence since last publish — skip to preserve gps_timeout
            self._primary_updated = False
            p           = self._primary.copy()
            s           = self._secondary.copy()
            vtg_heading = self._vtg_heading

        now = self.get_clock().now().to_msg()

        # NavSatFix — primary antenna
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

        # NavSatFix — front (secondary) antenna position for dual-antenna control
        if self._heading_source == 'baseline' and s['lat'] != 0.0:
            front_msg                 = NavSatFix()
            front_msg.header.stamp    = now
            front_msg.header.frame_id = 'gps_front'
            front_msg.latitude        = s['lat']
            front_msg.longitude       = s['lon']
            front_msg.altitude        = 0.0
            front_msg.status.status   = fix_msg.status.status   # same fix quality as primary
            front_msg.status.service  = NavSatStatus.SERVICE_GPS
            self.fix_front_pub.publish(front_msg)

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
