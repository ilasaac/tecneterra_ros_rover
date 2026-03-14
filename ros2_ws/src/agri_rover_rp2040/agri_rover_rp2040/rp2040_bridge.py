"""
agri_rover_rp2040 — rp2040_bridge node

Bridges RP2040 USB serial ↔ ROS2 topics.

Serial protocol (RP2040 → Jetson):
  CH:1500,1500,1700,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500 MODE:MANUAL
  [SBUS_OK] / [SBUS_LOST]
  [RF_LINK_OK] / [RF_LINK_LOST]   (master only)
  <HB:N+1>                         (heartbeat echo)
  [FAILSAFE] / [FAILSAFE_CLEARED]

Serial protocol (Jetson → RP2040):
  <HB:N>          heartbeat (must arrive within 300 ms or AUTO drops)
  <J:c0,...,c7>   8-channel autonomous override (PPM µs)
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:
    raise ImportError("pip install pyserial")

from agri_rover_interfaces.msg import RCInput


PPM_CENTER = 1500
CHANNELS   = 16


class Rp2040BridgeNode(Node):

    def __init__(self):
        super().__init__('rp2040_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('uart_port',          '/dev/ttyACM0')
        self.declare_parameter('uart_baud',          115200)
        self.declare_parameter('heartbeat_interval', 0.2)   # seconds

        port     = self.get_parameter('uart_port').value
        baud     = self.get_parameter('uart_baud').value
        hb_interval = self.get_parameter('heartbeat_interval').value

        # ── Publishers ───────────────────────────────────────────────────────
        self.rc_pub   = self.create_publisher(RCInput, 'rc_input', 10)
        self.mode_pub = self.create_publisher(String,  'mode',     10)

        # ── Subscribers ──────────────────────────────────────────────────────
        # Autonomous command from navigator — forwarded to RP2040 as <J:...>
        self.cmd_sub = self.create_subscription(
            RCInput, 'cmd_override', self._on_cmd_override, 10)

        # ── Serial ───────────────────────────────────────────────────────────
        self._ser  = serial.Serial(port, baud, timeout=0.05)
        self._lock = threading.Lock()
        self._hb_seq = 0

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(hb_interval, self._send_heartbeat)
        # UART read at 200 Hz — RP2040 sends at 50 Hz so we never miss a frame
        self.create_timer(0.005, self._read_uart)

        self.get_logger().info(f'RP2040 bridge ready on {port} @ {baud}')

    # ── UART read ─────────────────────────────────────────────────────────────

    def _read_uart(self):
        if not self._ser.in_waiting:
            return
        try:
            raw = self._ser.readline().decode('ascii', errors='ignore').strip()
        except serial.SerialException as e:
            self.get_logger().warn(f'UART read error: {e}')
            return

        if raw.startswith('CH:'):
            self._parse_ch_line(raw)
        elif raw.startswith('['):
            self.get_logger().debug(f'RP2040 status: {raw}')
        elif raw.startswith('<HB:'):
            pass  # heartbeat echo — no action needed

    def _parse_ch_line(self, line: str):
        # e.g. "CH:1500,1500,1700,1500,1500,1500,1500,1500,1500 MODE:MANUAL"
        try:
            ch_part, mode_str = line.split(' MODE:')
            channels = [int(x) for x in ch_part[3:].split(',')]
            if len(channels) < CHANNELS:
                return

            msg          = RCInput()
            msg.channels = channels[:CHANNELS]
            msg.mode     = mode_str.strip()
            msg.sbus_ok  = True   # TODO: track [SBUS_OK/LOST] flag separately
            msg.stamp    = self.get_clock().now().to_msg()
            self.rc_pub.publish(msg)

            mode_msg      = String()
            mode_msg.data = msg.mode
            self.mode_pub.publish(mode_msg)

        except (ValueError, IndexError):
            self.get_logger().warn(f'Malformed CH line: {line}')

    # ── UART write ────────────────────────────────────────────────────────────

    def _send_heartbeat(self):
        self._hb_seq += 1
        self._uart_write(f'<HB:{self._hb_seq}>\n')

    def _on_cmd_override(self, msg: RCInput):
        """Forward autonomous command to RP2040 as <J:c0,...,c7>."""
        chs = ','.join(str(c) for c in list(msg.channels[:8]))
        self._uart_write(f'<J:{chs}>\n')

    def _uart_write(self, text: str):
        with self._lock:
            try:
                self._ser.write(text.encode())
            except serial.SerialException as e:
                self.get_logger().warn(f'UART write error: {e}')

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Rp2040BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
