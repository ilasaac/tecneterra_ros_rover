"""
agri_rover_mavlink — mavlink_bridge node

Bridges ROS2 topics ↔ MAVLink v2 UDP to the Android GQC app.

Outbound (ROS2 → MAVLink → GQC):
  HEARTBEAT          1 Hz   — sysid, armed state, mode
  GLOBAL_POSITION_INT  5 Hz — lat, lon from /fix
  RC_CHANNELS        10 Hz  — raw PPM from /rc_input
  SYS_STATUS          1 Hz  — battery, comms state
  NAMED_VALUE_FLOAT   1 Hz  — TANK, TEMP, HUMID, PRESSURE
  STATUSTEXT          on event — log/alert messages

Inbound (GQC → MAVLink → ROS2):
  RC_CHANNELS_OVERRIDE → publishes to /cmd_override (→ rp2040_bridge)
  COMMAND_LONG (ARM/DISARM, SET_MODE) → calls /arm_disarm, /set_mode services
  MISSION_COUNT + MISSION_ITEM  → builds and publishes /mission

MAVLink addressing:
  sysid  = rover_id (1 or 2)
  compid = 1 (autopilot)
  GQC    = sysid 255
"""

from __future__ import annotations

import os
import socket as _socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

os.environ['MAVLINK20'] = '1'
try:
    from pymavlink import mavutil
except ImportError:
    raise ImportError("pip install pymavlink")

from agri_rover_interfaces.msg import RCInput, SensorData, RoverStatus, MissionWaypoint


# MAVLink mode mapping → custom_mode field
MODE_MAP = {
    'MANUAL':     0,
    'AUTONOMOUS': 4,
    'EMERGENCY':  16,
}


class MavlinkBridgeNode(Node):

    def __init__(self):
        super().__init__('mavlink_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('rover_id',       1)
        self.declare_parameter('gqc_host',       '192.168.1.255')  # broadcast
        self.declare_parameter('gqc_port',       14550)
        self.declare_parameter('bind_port',      14550)
        self.declare_parameter('heartbeat_rate', 1.0)

        self._rover_id   = self.get_parameter('rover_id').value
        self._gqc_host   = self.get_parameter('gqc_host').value
        gqc_port         = self.get_parameter('gqc_port').value
        bind_port        = self.get_parameter('bind_port').value

        # ── MAVLink connection ───────────────────────────────────────────────
        # udpin: listen for incoming; we also sendto GQC directly
        self._mav = mavutil.mavlink_connection(
            f'udpin:0.0.0.0:{bind_port}',
            source_system=self._rover_id,
            source_component=1,
        )
        self._gqc_addr = (self._gqc_host, gqc_port)

        # Dedicated outbound UDP socket (pymavlink internals vary by version)
        self._udp_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_BROADCAST, 1)

        # ── Local state (updated from subscriptions) ─────────────────────────
        self._fix         = NavSatFix()
        self._rc          = RCInput()
        self._sensors     = SensorData()
        self._status      = RoverStatus()
        self._armed       = False
        self._mode        = 'MANUAL'
        self._boot_ms     = int(time.time() * 1000)
        self._mission_buf: list[MissionWaypoint] = []
        self._mission_count = 0

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix,    'fix',         self._cb_fix,     10)
        self.create_subscription(RCInput,      'rc_input',    self._cb_rc,      10)
        self.create_subscription(SensorData,   'sensors',     self._cb_sensors, 10)
        self.create_subscription(RoverStatus,  'status',      self._cb_status,  10)
        self.create_subscription(String,       'mode',        self._cb_mode,    10)

        # ── Publishers (inbound MAVLink → ROS2) ──────────────────────────────
        self.cmd_pub     = self.create_publisher(RCInput,          'cmd_override', 10)
        self.mission_pub = self.create_publisher(MissionWaypoint,  'mission',      10)

        # ── MAVLink receive thread ────────────────────────────────────────────
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        # ── Send timers ──────────────────────────────────────────────────────
        self.create_timer(1.0,  self._send_heartbeat)
        self.create_timer(0.2,  self._send_gps)        # 5 Hz
        self.create_timer(0.1,  self._send_rc)         # 10 Hz
        self.create_timer(1.0,  self._send_sys_status)
        self.create_timer(1.0,  self._send_named_values)

        self.get_logger().info(
            f'MAVLink bridge sysid={self._rover_id} '
            f'bind={bind_port} → GQC {self._gqc_host}:{gqc_port}')

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _cb_fix(self, msg: NavSatFix):     self._fix = msg
    def _cb_rc(self, msg: RCInput):        self._rc = msg
    def _cb_sensors(self, msg: SensorData): self._sensors = msg
    def _cb_status(self, msg: RoverStatus): self._status = msg

    def _cb_mode(self, msg: String):
        self._mode = msg.data

    # ── MAVLink send helpers ──────────────────────────────────────────────────

    def _uptime_ms(self) -> int:
        return int(time.time() * 1000) - self._boot_ms

    def _send(self, msg):
        """Send a pre-packed MAVLink message to GQC via UDP."""
        try:
            buf = msg.pack(self._mav.mav)
            self._udp_sock.sendto(buf, self._gqc_addr)
        except Exception as e:
            self.get_logger().warn(f'MAVLink send error: {e}')

    def _send_heartbeat(self):
        self._send(self._mav.mav.heartbeat_encode(
            type=mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            base_mode=(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self._armed else 0),
            custom_mode=MODE_MAP.get(self._mode, 0),
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
        ))

    def _send_gps(self):
        if self._fix.latitude == 0.0:
            return
        self._send(self._mav.mav.global_position_int_encode(
            self._uptime_ms(),
            int(self._fix.latitude  * 1e7),
            int(self._fix.longitude * 1e7),
            0,       # alt MSL mm — TODO
            0,       # relative alt mm
            0, 0, 0, # vx, vy, vz cm/s
            0xFFFF,  # hdg unknown
        ))

    def _send_rc(self):
        chs = list(self._rc.channels) + [0] * (18 - len(self._rc.channels))
        self._send(self._mav.mav.rc_channels_encode(
            self._uptime_ms(), 9, *chs[:18], 0))

    def _send_sys_status(self):
        self._send(self._mav.mav.sys_status_encode(
            0, 0, 0,
            500,   # load % * 10
            int(getattr(self._status, 'battery_voltage', 0) * 1000),
            -1,    # current unknown
            int(getattr(self._status, 'battery_remaining', -1) * 100),
            0, 0, 0, 0, 0, 0,
        ))

    def _send_named_values(self):
        t = self._uptime_ms()
        pairs = [
            ('TANK',     self._sensors.tank_level),
            ('TEMP',     self._sensors.temperature),
            ('HUMID',    self._sensors.humidity),
            ('PRESSURE', self._sensors.pressure),
        ]
        for name, value in pairs:
            self._send(self._mav.mav.named_value_float_encode(
                t, name.encode(), value))

    # ── MAVLink receive loop ──────────────────────────────────────────────────

    def _recv_loop(self):
        while rclpy.ok():
            msg = self._mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == 'RC_CHANNELS_OVERRIDE':
                self._on_rc_override(msg)
            elif mt == 'COMMAND_LONG':
                self._on_command_long(msg)
            elif mt == 'MISSION_COUNT':
                self._mission_count = msg.count
                self._mission_buf   = []
                # Request first item
                self._send(self._mav.mav.mission_request_int_encode(
                    msg.get_srcSystem(), msg.get_srcComponent(), 0))
            elif mt == 'MISSION_ITEM_INT':
                self._on_mission_item(msg)

    def _on_rc_override(self, msg):
        rc = RCInput()
        raw = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
               msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw, 0]
        rc.channels = raw[:9]
        rc.mode     = 'OVERRIDE'
        rc.stamp    = self.get_clock().now().to_msg()
        self.cmd_pub.publish(rc)

    def _on_command_long(self, msg):
        if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            self._armed = (msg.param1 == 1.0)
            self.get_logger().info(f'{"ARM" if self._armed else "DISARM"} command received')
            self._send(self._mav.mav.command_ack_encode(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED))

    def _on_mission_item(self, msg):
        wp = MissionWaypoint()
        wp.seq       = msg.seq
        wp.latitude  = msg.x * 1e-7
        wp.longitude = msg.y * 1e-7
        wp.speed     = 1.0   # default; GQC can encode speed in param1
        wp.acceptance_radius = 1.5
        self._mission_buf.append(wp)
        self.mission_pub.publish(wp)

        if msg.seq + 1 < self._mission_count:
            self._send(self._mav.mav.mission_request_int_encode(
                msg.get_srcSystem(), msg.get_srcComponent(), msg.seq + 1))
        else:
            self._send(self._mav.mav.mission_ack_encode(
                msg.get_srcSystem(), msg.get_srcComponent(),
                mavutil.mavlink.MAV_MISSION_ACCEPTED))
            self.get_logger().info(
                f'Mission received: {self._mission_count} waypoints')


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._udp_sock.close()
        node.destroy_node()
        rclpy.shutdown()
