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

Reconnect behaviour:
  If the USB-CDC device disconnects (RP2040 reset, cable pull, OSError/EIO),
  the node closes the port and retries every 2 s.  No restart required.
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

# Both SerialException and OSError (errno 5 / EIO) can occur on USB disconnect
_SERIAL_ERRORS = (serial.SerialException, OSError)


class Rp2040BridgeNode(Node):

    def __init__(self):
        super().__init__('rp2040_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('uart_port',          '/dev/ttyACM0')
        self.declare_parameter('uart_baud',          115200)
        self.declare_parameter('heartbeat_interval', 0.2)   # seconds

        self._port  = self.get_parameter('uart_port').value
        self._baud  = self.get_parameter('uart_baud').value
        hb_interval = self.get_parameter('heartbeat_interval').value

        # ── Publishers ───────────────────────────────────────────────────────
        self.rc_pub   = self.create_publisher(RCInput, 'rc_input', 10)
        self.mode_pub = self.create_publisher(String,  'mode',     10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            RCInput, 'cmd_override', self._on_cmd_override, 10)

        # ── Serial state ─────────────────────────────────────────────────────
        self._ser  = None
        self._lock = threading.Lock()
        self._hb_seq = 0
        # Initialise both link flags optimistic (True). The RP2040 only sends
        # [SBUS_OK/LOST] and [RF_LINK_OK/LOST] on change, so if the link is
        # already healthy at bridge start we would never receive an OK event.
        self._sbus_ok    = True
        self._rf_link_ok = True

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(hb_interval, self._send_heartbeat)
        # UART read at 200 Hz — RP2040 sends at 50 Hz so we never miss a frame
        self.create_timer(0.005, self._read_uart)
        # Reconnect check — tries to reopen port when disconnected
        self.create_timer(2.0, self._reconnect)

        # Open serial immediately; _reconnect will retry if unavailable
        self._try_open()

    # ── Serial open / reconnect ────────────────────────────────────────────────

    def _try_open(self):
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.05)
            with self._lock:
                self._ser = ser
            self.get_logger().info(f'RP2040 bridge ready on {self._port} @ {self._baud}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Cannot open {self._port}: {e} — retrying in 2 s')

    def _reconnect(self):
        with self._lock:
            connected = self._ser is not None
        if not connected:
            self._try_open()

    def _serial_error(self, e):
        """Close the port on any serial/IO error and let _reconnect reopen it."""
        self.get_logger().warn(
            f'Serial error ({type(e).__name__} {getattr(e, "errno", "")}): {e}'
            f' — port closed, reconnecting in 2 s')
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass
            self._ser = None

    # ── UART read ─────────────────────────────────────────────────────────────

    def _read_uart(self):
        if self._ser is None:
            return
        try:
            if not self._ser.in_waiting:
                return
            raw = self._ser.readline().decode('ascii', errors='ignore').strip()
        except _SERIAL_ERRORS as e:
            self._serial_error(e)
            return

        if raw.startswith('CH:'):
            self._parse_ch_line(raw)
        elif raw == '[SBUS_OK]':
            self._sbus_ok = True
            self.get_logger().info('SBUS OK')
        elif raw == '[SBUS_LOST]':
            self._sbus_ok = False
            self.get_logger().warn('SBUS LOST')
        elif raw == '[RF_LINK_OK]':
            self._rf_link_ok = True
            self.get_logger().info('RF link OK')
        elif raw == '[RF_LINK_LOST]':
            self._rf_link_ok = False
            self.get_logger().warn('RF link LOST')
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
            msg.mode        = mode_str.strip()
            msg.sbus_ok     = self._sbus_ok
            msg.rf_link_ok  = self._rf_link_ok
            msg.stamp    = self.get_clock().now().to_msg()
            self.rc_pub.publish(msg)

            mode_msg      = String()
            # AUTO-TIMEOUT means CH5 in AUTO position + HB alive but no fresh cmd yet.
            # Treat it as AUTONOMOUS so the navigator starts publishing cmd_override,
            # which immediately transitions the RP2040 to true AUTONOMOUS state.
            mode_msg.data = 'AUTONOMOUS' if msg.mode == 'AUTO-TIMEOUT' else msg.mode
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
            if self._ser is None:
                return
            try:
                self._ser.write(text.encode())
            except _SERIAL_ERRORS as e:
                self.get_logger().warn(f'UART write error: {e}')
                try:
                    if self._ser.is_open:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass
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
