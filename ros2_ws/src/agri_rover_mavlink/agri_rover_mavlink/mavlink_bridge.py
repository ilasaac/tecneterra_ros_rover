"""
agri_rover_mavlink — mavlink_bridge node

Bridges ROS2 topics ↔ MAVLink v2 UDP to the Android GQC app.

Outbound (ROS2 → MAVLink → GQC):
  HEARTBEAT          1 Hz   — sysid, armed state, mode
  GLOBAL_POSITION_INT  5 Hz — lat, lon from /fix
  RC_CHANNELS        10 Hz  — PPM from /cmd_override in AUTO, /rc_input otherwise
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

import json
import os
import socket as _socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
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

# Live navigator parameter tuning — MAVLink param_id (≤16 chars) → nav_params JSON key.
# navigator.py publishes nav_params (JSON, every 5 s + on change).
# mavlink_bridge caches it and serves PARAM_REQUEST_LIST / PARAM_SET from GQC.
_MAVLINK_PARAMS: dict[str, str] = {
    'MAX_SPEED':    'max_speed',
    'MIN_SPEED':    'min_speed',
    'LOOKAHEAD':    'lookahead_distance',
    'STANLEY_K':    'stanley_k',
    'PIVOT_THRESH': 'pivot_threshold',
    'PIVOT_DIST':   'pivot_approach_dist',
    'ALIGN_THRESH': 'align_threshold',
    'ACCEPT_RAD':   'default_acceptance_radius',
    'AFS_MIN_THR':  'afs_min_throttle_ppm',
    'MIN_STEER_PPM':   'min_steer_ppm_delta',
    'STEER_COAST_ANG': 'steer_coast_angle',
    'GPS_ACC_ALARM':   'gps_accuracy_alarm_mm',
}
_MAVLINK_PARAM_LIST: list[str] = list(_MAVLINK_PARAMS.keys())  # stable ordered list


class MavlinkBridgeNode(Node):

    def __init__(self):
        super().__init__('mavlink_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('rover_id',       1)
        self.declare_parameter('gqc_host',       '192.168.1.255')  # broadcast
        self.declare_parameter('gqc_port',       14550)
        self.declare_parameter('bind_port',      14550)
        self.declare_parameter('heartbeat_rate', 1.0)
        # Resource management — base station coordinates and thresholds
        self.declare_parameter('recharge_lat',      0.0)
        self.declare_parameter('recharge_lon',      0.0)
        self.declare_parameter('water_lat',         0.0)
        self.declare_parameter('water_lon',         0.0)
        self.declare_parameter('battery_low_pct',  15.0)
        self.declare_parameter('tank_low_pct',       5.0)
        self.declare_parameter('base_acceptance_m', 1.5)

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
        self._heading_deg = None   # None until first heading message received
        self._boot_ms     = int(time.time() * 1000)
        self._cmd_thr      = 1500        # last cmd_override throttle (CH1), for telemetry
        self._cmd_str      = 1500        # last cmd_override steering (CH2), for telemetry
        self._cmd_channels: list[int] = []  # full cmd_override channel list
        self._wp_active   = -1    # active waypoint seq from navigator (-1 = none)
        self._test_tank:     float | None = None  # test-injected tank level (0-100 %)
        self._test_batt_pct: float | None = None  # test-injected battery charge (0-100 %)
        self._xte         = 0.0   # cross-track error from navigator (metres)
        self._mission_buf: list[MissionWaypoint] = []
        self._mission_count = 0
        # Fence polygon buffer: list of (lat, lon, vertex_count) for cmd=5003 items.
        # Cleared on MISSION_COUNT; published as ~/mission_fence JSON when mission completes.
        self._fence_buf: list[tuple[float, float, int]] = []
        # Servo state for PPM CH5-CH8 (servo numbers 5-8 → channel indices 4-7).
        # Updated by DO_SET_SERVO commands; forwarded to rp2040_bridge via cmd_override.
        self._servo_pwm = {5: 1500, 6: 1500, 7: 1500, 8: 1500}
        # Per-waypoint servo changes: built during mission upload.
        # {wp_seq: {servo_ch: pwm}} — applied when navigator publishes wp_active == wp_seq.
        # DO_SET_SERVO items that appear AFTER a NAV_WAYPOINT in the item stream are deferred
        # here; items that appear BEFORE the first NAV_WAYPOINT are applied immediately.
        self._wp_servo_map: dict[int, dict[int, int]] = {}
        self._last_nav_wp_seq: int | None = None  # seq of last NAV_WAYPOINT received
        # Mission upload retry state — protected by _mission_lock.
        # _gqc_unicast: GQC IP discovered from first incoming UDP packet; used instead
        #   of broadcast for mission handshake replies (broadcast is unreliable on WiFi APs).
        # _mission_src: (srcSys, srcComp) from MISSION_COUNT; None when no upload active.
        # _mission_expect_seq: next seq we're waiting for; None when no upload active.
        # _mission_last_req_t: monotonic time of last MISSION_REQUEST_INT sent.
        # Resource management state machine: 'normal' → 'going_to_base' → 'at_base' → 'normal'
        self._resource_state  = 'normal'   # 'normal' | 'going_to_base' | 'at_base'
        self._resource_reason: str | None  = None  # 'battery' | 'tank'
        self._resume_mission:  list        = []    # snapshot of _mission_buf at trigger time
        self._resume_wp_seq    = 0                 # wp_active index to resume from
        self._resume_servo_map: dict       = {}    # servo map renumbered for resume mission
        self._resume_fence_buf: list       = []    # obstacle fence saved at trigger time
        self._resume_position: tuple | None = None  # (lat, lon) rover position at trigger
        # Runtime station overrides (set via GQC cmd 50001/50002); override YAML params
        self._recharge_lat: float | None = None
        self._recharge_lon: float | None = None
        self._water_lat:    float | None = None
        self._water_lon:    float | None = None
        self._rtk_status  = 'NO_FIX'   # latest string from gps_driver rtk_status topic
        self._hacc_mm     = -1.0       # horizontal accuracy from UBX NAV-PVT (mm); -1 = unknown
        self._nav_params: dict[str, float] = {}  # latest nav_params JSON from navigator.py
        self._path_version        = 0
        self._reroute_pending     = False
        self._last_rerouted_json  = '[]'
        self._reroute_retries     = 0
        self._reroute_timer       = None
        self._mission_lock        = threading.Lock()
        self._gqc_unicast         = None            # (ip, port) or None
        self._mission_src         = None            # (srcSys, srcComp)
        self._mission_expect_seq  = None            # int or None
        self._mission_last_req_t  = 0.0
        self._mission_retry_count = 0               # consecutive retries for same seq
        self._mission_retry_at_seq = None           # seq being counted

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix,    'fix',          self._cb_fix,      10)
        self.create_subscription(RCInput,      'rc_input',     self._cb_rc,       10)
        self.create_subscription(SensorData,   'sensors',      self._cb_sensors,  10)
        self.create_subscription(RoverStatus,  'status',       self._cb_status,   10)
        self.create_subscription(String,       'mode',         self._cb_mode,     10)
        self.create_subscription(Float32,      'heading',      self._cb_heading,  10)
        self.create_subscription(RCInput,      'cmd_override', self._cb_cmd_mon,  10)
        self.create_subscription(Int32,        'wp_active',    self._cb_wp,       10)
        self.create_subscription(Float32,      'xte',           self._cb_xte,            10)
        self.create_subscription(String,       'rerouted_path', self._cb_rerouted_path,  10)
        self.create_subscription(Int32,        'path_version',  self._cb_path_version,   10)
        self.create_subscription(String,       'rtk_status',    self._cb_rtk_status,     10)
        self.create_subscription(Float32,      'hacc',          self._cb_hacc,           10)
        self.create_subscription(String,       'nav_params',    self._cb_nav_params,     10)
        self.create_subscription(Bool,         'reroute_pending', self._cb_reroute_pending, 10)

        # ── Publishers (inbound MAVLink → ROS2) ──────────────────────────────
        self.cmd_pub           = self.create_publisher(RCInput,          'cmd_override',   10)
        self.mission_pub       = self.create_publisher(MissionWaypoint,  'mission',        10)
        self.reroute_response_pub = self.create_publisher(Bool,          'reroute_response', 10)
        self.fence_pub         = self.create_publisher(String,           'mission_fence',  10)
        self.servo_pub         = self.create_publisher(RCInput,          'servo_state',    10)
        self.mode_pub          = self.create_publisher(String,           'mode',           10)
        self.armed_pub         = self.create_publisher(Bool,             'armed',          10)
        self.mission_clear_pub = self.create_publisher(Bool,             'mission_clear',  10)
        self.nav_param_set_pub = self.create_publisher(String,           'nav_param_set',  10)

        # ── MAVLink receive thread ────────────────────────────────────────────
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        # ── Send timers ──────────────────────────────────────────────────────
        self.create_timer(1.0,  self._send_heartbeat)
        self.create_timer(0.2,  self._send_gps)        # 5 Hz
        self.create_timer(0.1,  self._send_rc)         # 10 Hz
        self.create_timer(1.0,  self._send_sys_status)
        self.create_timer(1.0,  self._send_named_values)
        self.create_timer(0.5,  self._mission_retry)   # retransmit lost MISSION_REQUEST_INT
        self.create_timer(1.0,  self._check_resource_levels)  # battery/tank threshold check

        self.get_logger().info(
            f'MAVLink bridge sysid={self._rover_id} '
            f'bind={bind_port} → GQC {self._gqc_host}:{gqc_port}')

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _cb_fix(self, msg: NavSatFix):           self._fix = msg
    def _cb_rc(self, msg: RCInput):              self._rc = msg
    def _cb_sensors(self, msg: SensorData):      self._sensors = msg
    def _cb_status(self, msg: RoverStatus):      self._status = msg
    def _cb_rtk_status(self, msg: String):       self._rtk_status = msg.data
    def _cb_hacc(self, msg: Float32):            self._hacc_mm = msg.data

    def _cb_mode(self, msg: String):
        self._mode = msg.data

    def _cb_heading(self, msg: Float32):
        self._heading_deg = msg.data

    def _cb_cmd_mon(self, msg: RCInput):
        """Track cmd_override channels for autonomous RC_CHANNELS telemetry."""
        if len(msg.channels) > 0:
            self._cmd_thr = msg.channels[0]
        if len(msg.channels) > 1:
            self._cmd_str = msg.channels[1]
        self._cmd_channels = list(msg.channels)

    def _cb_wp(self, msg: Int32):
        self._wp_active = msg.data
        # Waiting point signal: navigator encodes as -(seq + 1000)
        if msg.data <= -1000:
            wp_seq = -(msg.data + 1000)
            self._wp_active = wp_seq          # show correct WP in telemetry
            self._armed = False
            self._mode  = 'MANUAL'
            a = Bool(); a.data = False
            m = String(); m.data = 'MANUAL'
            self.armed_pub.publish(a)
            self.mode_pub.publish(m)
            self.get_logger().info(
                f'Waiting point WP{wp_seq} — disarmed, mission kept (re-arm to continue)')
            self._send_statustext(f'Waiting at WP{wp_seq} — re-arm to continue')
            return  # _mission_count stays → STATUS=MSL
        if msg.data >= 0 and msg.data in self._wp_servo_map:
            for sn, pw in self._wp_servo_map[msg.data].items():
                self.get_logger().info(f'WP{msg.data} reached — applying servo CH{sn}={pw}')
                self._apply_servo_cmd(sn, pw)
        if msg.data == -1:
            a = Bool(); a.data = False
            m = String(); m.data = 'MANUAL'
            if self._resource_state == 'going_to_base':
                # Base trip complete — disarm, then upload resume mission immediately
                # so GQC has it ready (WP0 = base station = rover's current position,
                # proximity check will pass on START).
                self._resource_state = 'at_base'
                self._armed          = False
                self._mode           = 'MANUAL'
                self.armed_pub.publish(a)
                self.mode_pub.publish(m)
                reason_label = 'battery' if self._resource_reason == 'battery' else 'tank'
                self.get_logger().info(
                    f'[RESOURCE] Arrived at base ({reason_label}) — re-arm to resume mission '
                    f'from WP{self._resume_wp_seq}')
                self._send_statustext(
                    f'At base ({reason_label}) — re-arm to resume WP{self._resume_wp_seq}')
                # Upload resume mission (base → stopped pos → remaining WPs).
                # _internal_upload_mission sets _mission_count/_mission_buf and
                # broadcasts to GQC so the map shows the resume route.
                self._upload_resume_mission()
            else:
                # Normal mission complete — auto-disarm and reset mission state → STATUS → NA
                self._armed          = False
                self._mission_count  = 0
                self._mission_buf    = []
                self._wp_servo_map   = {}
                self.armed_pub.publish(a)
                self.mode_pub.publish(m)
                self.get_logger().info('Mission complete — auto-disarmed')

    def _cb_xte(self, msg: Float32):
        self._xte = msg.data

    def _cb_path_version(self, msg: Int32):
        self._path_version = msg.data

    def _cb_reroute_pending(self, msg: Bool):
        self._reroute_pending = msg.data
        if msg.data:
            self._send_statustext('Route changed — confirm in GQC')

    def _cb_rerouted_path(self, msg: String):
        """
        Forward the navigator's rerouted path to GQC as MAVLink TUNNEL messages.
        Payload format: [chunk_idx, total_chunks, ...utf8 json bytes...]
        payload_type = 0x5250 ('RP' — Rerouted Path), max 128 bytes per packet.

        Sends immediately then retransmits twice (at +2 s, +4 s) to survive
        UDP packet loss — the GQC reassembler is idempotent.
        """
        # Cancel any pending retransmit for the previous path
        if hasattr(self, '_reroute_timer') and self._reroute_timer is not None:
            self._reroute_timer.cancel()
            self._reroute_timer = None

        self._last_rerouted_json = msg.data
        self._reroute_retries    = 0
        self._send_rerouted_chunks(msg.data)
        self._schedule_reroute_retransmit()

    def _send_rerouted_chunks(self, json_str: str):
        try:
            json_bytes = (json_str or '[]').encode('utf-8')
            chunks = [json_bytes[i:i + 126] for i in range(0, len(json_bytes), 126)]
            n = len(chunks)
            for i, chunk in enumerate(chunks):
                payload_bytes = bytes([i, n]) + chunk
                padded = list(payload_bytes) + [0] * (128 - len(payload_bytes))
                tunnel = self._mav.mav.tunnel_encode(
                    target_system=0,
                    target_component=0,
                    payload_type=0x5250,
                    payload_length=len(payload_bytes),
                    payload=padded[:128])
                self._send(tunnel)
        except Exception as e:
            self.get_logger().warn(f'rerouted_path tunnel send: {e}')

    def _schedule_reroute_retransmit(self):
        if self._reroute_retries < 2:
            self._reroute_timer = threading.Timer(2.0, self._retransmit_rerouted)
            self._reroute_timer.start()

    def _retransmit_rerouted(self):
        self._reroute_retries += 1
        self.get_logger().info(
            f'rerouted_path retransmit #{self._reroute_retries}')
        self._send_rerouted_chunks(self._last_rerouted_json)
        self._schedule_reroute_retransmit()

    # ── MAVLink send helpers ──────────────────────────────────────────────────

    def _uptime_ms(self) -> int:
        return int(time.time() * 1000) - self._boot_ms

    def _send(self, msg):
        """Send a MAVLink message to GQC (unicast if discovered, else broadcast).

        Always sends to broadcast so passive tools (monitor.py, simulator) can
        receive it.  If GQC unicast address is known, sends a second copy
        to that address to bypass WiFi DTIM buffering (~100 ms).
        """
        try:
            buf = msg.pack(self._mav.mav)
            # 1. Always broadcast for tools
            self._udp_sock.sendto(buf, self._gqc_addr)
            # 2. Unicast to GQC for low latency
            if self._gqc_unicast:
                self._udp_sock.sendto(buf, self._gqc_unicast)
        except Exception as e:
            self.get_logger().warn(f'MAVLink send error: {e}')

    def _send_heartbeat(self):
        if self._mode == 'EMERGENCY':
            sys_status = mavutil.mavlink.MAV_STATE_EMERGENCY
        elif self._armed:
            sys_status = mavutil.mavlink.MAV_STATE_ACTIVE
        else:
            sys_status = mavutil.mavlink.MAV_STATE_STANDBY
        base_mode = 0
        if self._armed:
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if self._mode == 'AUTONOMOUS':
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        self._send(self._mav.mav.heartbeat_encode(
            type=mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            base_mode=base_mode,
            custom_mode=MODE_MAP.get(self._mode, 0),
            system_status=sys_status,
        ))

    # Maps gps_driver rtk_status strings → MAVLink GPS_RAW_INT fix_type (0–6)
    _RTK_FIX_TYPE = {
        'RTK_FIX': 6,
        'RTK_FLT': 5,
        'DGPS':    4,
        'GPS':     3,
        'NO_FIX':  1,
    }

    def _send_gps(self):
        if self._fix.latitude == 0.0:
            return
        # hdg field: cdeg (0-35999), or 65535 if unknown
        hdg = int(self._heading_deg * 100) % 36000 \
              if self._heading_deg is not None else 0xFFFF
        self._send(self._mav.mav.global_position_int_encode(
            self._uptime_ms(),
            int(self._fix.latitude  * 1e7),
            int(self._fix.longitude * 1e7),
            0,       # alt MSL mm — TODO
            0,       # relative alt mm
            0, 0, 0, # vx, vy, vz cm/s
            hdg,
        ))
        # GPS_RAW_INT (#24) — fix quality for GQC RTK indicator.
        # fix_type comes from gps_driver's rtk_status topic, which parses the
        # GGA quality field directly from the u-blox module NMEA output (or
        # simulator NMEA — same path through gps_driver either way).
        fix_type = self._RTK_FIX_TYPE.get(self._rtk_status, 0)
        self._send(self._mav.mav.gps_raw_int_encode(
            self._uptime_ms() * 1000,  # time_usec
            fix_type,
            int(self._fix.latitude  * 1e7),
            int(self._fix.longitude * 1e7),
            0,        # alt mm
            0xFFFF,   # eph (unknown)
            0xFFFF,   # epv (unknown)
            0xFFFF,   # vel (unknown)
            0xFFFF,   # cog (unknown)
            255,      # satellites_visible (unknown)
        ))

    def _send_rc(self):
        # In AUTONOMOUS mode show the actual PPM commands sent to the motors,
        # remapped back to SBUS channel order so GQC's remapSbusToLogical()
        # produces the correct PPM CH5/CH6 (servo/pump) display values.
        # In all other modes show the raw RC stick input (already in SBUS order).
        if self._mode == 'AUTONOMOUS' and self._cmd_channels:
            ppm = self._cmd_channels
            # Inverse of GQC remapSbusToLogical():
            #   PPM CH1 (throttle) → SBUS[2]  (CH3)
            #   PPM CH2 (steering) → SBUS[0]  (CH1)
            #   PPM CH3 (SWA)      → SBUS[4]  (CH5)
            #   PPM CH4 (SWB)      → SBUS[5]  (CH6)
            #   PPM CH5 (servo5)   → SBUS[10] (CH11)
            #   PPM CH6 (servo6)   → SBUS[11] (CH12)
            #   PPM CH7 (servo7)   → SBUS[6]  (CH7)
            #   PPM CH8 (servo8)   → SBUS[7]  (CH8)
            #   PPM CH9            → SBUS[8]  (CH9)
            def _p(i): return ppm[i] if len(ppm) > i else 1500
            sbus_sim = [1500] * 16
            sbus_sim[2]  = _p(0)   # throttle
            sbus_sim[0]  = _p(1)   # steering
            sbus_sim[4]  = _p(2)   # SWA (emergency)
            sbus_sim[5]  = _p(3)   # SWB (autonomous)
            sbus_sim[10] = _p(4)   # servo5 / pump
            sbus_sim[11] = _p(5)   # servo6
            sbus_sim[6]  = 3000 - _p(6)   # servo7 un-invert (CH7 inverted in ppm_map)
            sbus_sim[7]  = 3000 - _p(7)   # servo8 un-invert (CH8 inverted in ppm_map)
            # CH9 is a physical switch (rover select) — always read from raw RC,
            # not from navigator cmd_override which only has 8 channels.
            rc_chs = self._rc.channels
            sbus_sim[8]  = rc_chs[8] if len(rc_chs) > 8 else 1500
            src = sbus_sim
        else:
            src = list(self._rc.channels)
        chs = src + [0] * (18 - len(src))
        chancount = len(src)
        self._send(self._mav.mav.rc_channels_encode(
            self._uptime_ms(), chancount, *chs[:18], 0))

    def _send_sys_status(self):
        self._send(self._mav.mav.sys_status_encode(
            0, 0, 0,
            500,   # load % * 10
            int(getattr(self._status, 'battery_voltage', 0) * 1000),
            -1,    # current unknown
            int(self._test_batt_pct if self._test_batt_pct is not None else (self._status.battery_remaining * 100 if self._status.battery_remaining > 0 else -1)),
            0, 0, 0, 0, 0, 0,
        ))

    def _send_named_values(self):
        t = self._uptime_ms()
        nav_status = 2.0 if self._armed else (1.0 if self._mission_count > 0 else 0.0)
        pairs = [
            ('TANK',     self._test_tank   if self._test_tank   is not None else self._sensors.tank_level),
            ('TEMP',     self._sensors.temperature),
            ('HUMID',    self._sensors.humidity),
            ('PRESSURE', self._sensors.pressure),
            ('CMD_T',    float(self._cmd_thr)),
            ('CMD_S',    float(self._cmd_str)),
            ('WP_ACT',   float(self._wp_active)),
            ('WP_TOT',   float(self._mission_count)),
            ('XTE',      self._xte),
            ('STATUS',   nav_status),
            ('SBUS_OK',  1.0 if self._rc.sbus_ok   else 0.0),
            ('RF_OK',    1.0 if self._rc.rf_link_ok else 0.0),
            ('RTK',      float(self._RTK_FIX_TYPE.get(self._rtk_status, 0))),
            ('HACC',     self._hacc_mm),
            ('PATH_VER', float(self._path_version)),
            ('REROUTE',  1.0 if self._reroute_pending else 0.0),
        ]
        for name, value in pairs:
            self._send(self._mav.mav.named_value_float_encode(
                t, name.encode(), value))

    # ── MAVLink receive loop ──────────────────────────────────────────────────

    def _recv_loop(self):
        """Receive loop using the raw socket so source IP is always captured.

        pymavlink's recv_match() sometimes returns a buffered message without
        calling recvfrom() again, so last_address is stale or None.  Reading
        the underlying socket directly guarantees we get the sender's address
        for every packet, which is needed to switch telemetry to unicast.
        """
        sock = self._mav.port        # mavudp stores the DatagramSocket here
        sock.settimeout(1.0)

        while rclpy.ok():
            try:
                data, (src_ip, src_port) = sock.recvfrom(1024)
            except _socket.timeout:
                continue
            except OSError:
                continue

            msgs = self._mav.mav.parse_buffer(data)
            if not msgs:
                continue

            for msg in msgs:
                # Discovery: only record GQC address if packet comes from sysid 255.
                # If we recorded every packet, rovers would try to unicast to each other!
                # We force the port to gqc_port (14550) because GQC binds that port for RX,
                # even if its OS-assigned source port for sending is a random high port.
                if msg.get_srcSystem() == 255:
                    self._gqc_unicast = (src_ip, self._gqc_addr[1])

                mt = msg.get_type()
                if mt == 'RC_CHANNELS_OVERRIDE':
                    self._on_rc_override(msg)
                elif mt == 'COMMAND_LONG':
                    self._on_command_long(msg)
                elif mt == 'MISSION_COUNT':
                    if msg.get_srcSystem() != 255:
                        continue  # ignore re-broadcasts from other rovers (feedback loop prevention)
                    if msg.target_system not in (0, 255, self._rover_id):
                        continue  # not addressed to this rover
                    if msg.count == 0:
                        # Empty mission (CLEAR command) — ACK immediately, stop navigator.
                        with self._mission_lock:
                            self._mission_count      = 0
                            self._mission_buf        = []
                            self._mission_src        = None
                            self._mission_expect_seq = None
                        self._wp_servo_map    = {}
                        self._last_nav_wp_seq = None
                        addr = self._gqc_unicast or self._gqc_addr
                        packed = self._mav.mav.mission_ack_encode(
                            msg.get_srcSystem(), msg.get_srcComponent(),
                            mavutil.mavlink.MAV_MISSION_ACCEPTED).pack(self._mav.mav)
                        try:
                            self._udp_sock.sendto(packed, addr)
                        except Exception as e:
                            self.get_logger().warn(f'mission_ack send error: {e}')
                        # Tell navigator to drain its waypoint queue and halt
                        clr = Bool(); clr.data = True
                        self.mission_clear_pub.publish(clr)
                        m = String(); m.data = 'MANUAL'
                        self.mode_pub.publish(m)
                        self.get_logger().info('Empty mission received — navigator cleared')
                        continue
                    with self._mission_lock:
                        self._mission_count      = msg.count
                        self._mission_buf        = []
                        self._fence_buf          = []   # clear fence data for new mission
                        self._mission_src        = (msg.get_srcSystem(), msg.get_srcComponent())
                        self._mission_expect_seq = 0
                        self._mission_last_req_t = time.monotonic()
                    self._wp_servo_map       = {}
                    self._last_nav_wp_seq    = None
                    self._mission_retry_count  = 0
                    self._mission_retry_at_seq = None
                    # Cancel any in-progress resource management — user uploaded a new mission
                    if self._resource_state in ('going_to_base', 'at_base'):
                        self.get_logger().info(
                            '[RESOURCE] Manual mission upload — cancelling resource management')
                        self._resource_state = 'normal'
                        self._resume_mission  = []
                    self.get_logger().info(
                        f'Mission upload started: {msg.count} items '
                        f'from sysid={msg.get_srcSystem()}')
                    # Re-broadcast for passive tools (monitor.py).
                    # GQC sends unicast to our IP so tools on other hosts won't see it.
                    try:
                        fwd = self._mav.mav.mission_count_encode(
                            0, 0, msg.count, getattr(msg, 'mission_type', 0))
                        self._udp_sock.sendto(fwd.pack(self._mav.mav), self._gqc_addr)
                    except Exception:
                        pass
                    # Delay first REQUEST_INT slightly — GQC needs time to switch its
                    # coroutine back to Dispatchers.IO and start waiting for requests.
                    # Without this the first request arrives before GQC is listening.
                    threading.Timer(0.15, lambda: self._send_mission_request(0)).start()
                elif mt == 'MISSION_ITEM_INT':
                    self._on_mission_item(msg)
                elif mt == 'MISSION_REQUEST_LIST':
                    if msg.get_srcSystem() != 255:
                        continue
                    if msg.target_system not in (0, 255, self._rover_id):
                        continue
                    self._send_mission_to_gqc()
                elif mt == 'PARAM_REQUEST_LIST':
                    if msg.get_srcSystem() != 255:
                        continue
                    if msg.target_system not in (0, 255, self._rover_id):
                        continue
                    threading.Thread(target=self._send_all_params, daemon=True).start()
                elif mt == 'PARAM_REQUEST_READ':
                    if msg.get_srcSystem() != 255:
                        continue
                    if msg.target_system not in (0, 255, self._rover_id):
                        continue
                    count = len(_MAVLINK_PARAM_LIST)
                    idx   = msg.param_index
                    if 0 <= idx < count:
                        mav_name = _MAVLINK_PARAM_LIST[idx]
                    else:
                        raw = msg.param_id
                        name = raw.rstrip('\x00') if isinstance(raw, str) else raw.rstrip(b'\x00').decode('ascii', errors='ignore')
                        mav_name = name if name in _MAVLINK_PARAMS else None
                    if mav_name:
                        ros_name = _MAVLINK_PARAMS[mav_name]
                        real_idx = _MAVLINK_PARAM_LIST.index(mav_name)
                        self._send_param_value_mav(
                            mav_name,
                            float(self._nav_params.get(ros_name, 0.0)),
                            real_idx, count)
                elif mt == 'PARAM_SET':
                    if msg.get_srcSystem() != 255:
                        continue
                    if msg.target_system not in (0, 255, self._rover_id):
                        continue
                    raw = msg.param_id
                    mav_name = raw.rstrip('\x00') if isinstance(raw, str) else raw.rstrip(b'\x00').decode('ascii', errors='ignore')
                    # Test sensor injection (from mission_planner or other tools)
                    if mav_name == 'TANK_LEVEL':
                        self._test_tank = max(0.0, min(100.0, float(msg.param_value)))
                        self.get_logger().info(f'[TEST] tank={self._test_tank:.0f}%')
                        continue
                    if mav_name == 'BATT_PCT':
                        self._test_batt_pct = max(0.0, min(100.0, float(msg.param_value)))
                        self.get_logger().info(f'[TEST] battery={self._test_batt_pct:.0f}%')
                        continue
                    ros_name = _MAVLINK_PARAMS.get(mav_name)
                    if ros_name is None:
                        self.get_logger().warn(f'PARAM_SET: unknown param "{mav_name}"')
                        continue
                    value = float(msg.param_value)
                    req = String(); req.data = json.dumps({'name': ros_name, 'value': value})
                    self.nav_param_set_pub.publish(req)
                    idx = _MAVLINK_PARAM_LIST.index(mav_name)
                    self._send_param_value_mav(mav_name, value, idx, len(_MAVLINK_PARAM_LIST))
                    self.get_logger().info(f'PARAM_SET {mav_name}={value} → navigator')

    def _send_mission_request(self, seq: int):
        """Send MISSION_REQUEST_INT, preferring unicast over broadcast."""
        with self._mission_lock:
            src = self._mission_src
            self._mission_last_req_t = time.monotonic()
        if src is None:
            return
        packed = self._mav.mav.mission_request_int_encode(src[0], src[1], seq).pack(self._mav.mav)
        addr = self._gqc_unicast or self._gqc_addr
        if seq == 0:
            self.get_logger().info(f'Mission upload addr: {addr} ({"unicast" if self._gqc_unicast else "BROADCAST"})')
        try:
            self._udp_sock.sendto(packed, addr)
        except Exception as e:
            self.get_logger().warn(f'mission_request send error: {e}')

    def _mission_retry(self):
        """Retransmit MISSION_REQUEST_INT if the expected item hasn't arrived in 250 ms.

        With streaming upload GQC pushes items proactively so this fires only on
        genuine packet loss.  250 ms is long enough to cover one DTIM period on slow
        WiFi but short enough that a lost item doesn't stall the upload for long.
        Aborts after 20 consecutive retries for the same seq (stuck state, e.g. from
        a second upload interrupting the first).
        """
        with self._mission_lock:
            seq     = self._mission_expect_seq
            elapsed = time.monotonic() - self._mission_last_req_t
        if seq is None or elapsed < 0.25:
            return
        if seq == self._mission_retry_at_seq:
            self._mission_retry_count += 1
        else:
            self._mission_retry_count  = 1
            self._mission_retry_at_seq = seq
        if self._mission_retry_count > 20:
            self.get_logger().error(
                f'Mission upload aborted — seq={seq} not received after '
                f'{self._mission_retry_count} retries (~10 s); start a fresh upload')
            with self._mission_lock:
                self._mission_expect_seq = None
                self._mission_src        = None
            self._mission_retry_count  = 0
            self._mission_retry_at_seq = None
            try:
                stxt = self._mav.mav.statustext_encode(3, b'Mission upload aborted, retry')
                self._send(stxt)
            except Exception:
                pass
            return
        self.get_logger().info(f'Retrying MISSION_REQUEST_INT seq={seq} ({elapsed*1000:.0f}ms)')
        self._send_mission_request(seq)

    def _send_mission_to_gqc(self):
        """Respond to MISSION_REQUEST_LIST — stream current mission back to GQC.

        Sends MISSION_COUNT followed immediately by all MISSION_ITEM_INT from
        _mission_buf (original uploaded waypoints only, no fence/servo items).
        GQC uses this to stay in sync when PATH_VER changes (obstacle reroute).
        """
        items = list(self._mission_buf)   # snapshot under no lock — reads are safe
        count_msg = self._mav.mav.mission_count_encode(0, 0, len(items), 0)
        self._send(count_msg)
        for i, wp in enumerate(items):
            item = self._mav.mav.mission_item_int_encode(
                target_system=0, target_component=0,
                seq=i,
                frame=6,   # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                command=16,  # MAV_CMD_NAV_WAYPOINT
                current=0,
                autocontinue=1,
                param1=wp.hold_secs,
                param2=0.0, param3=0.0, param4=0.0,
                x=int(wp.latitude  * 1e7),
                y=int(wp.longitude * 1e7),
                z=wp.speed,
                mission_type=0)
            self._send(item)
        self.get_logger().info(
            f'Mission download: sent {len(items)} waypoints to GQC (PATH_VER={self._path_version})')

    def _parse_fence_polygons(self, buf=None) -> list[list[list[float]]]:
        """
        Parse a fence buffer (list of (lat, lon, vertex_count)) into a list of polygons.

        Each polygon is defined by consecutive items with the same vertex_count.
        When vertex_count consecutive vertices have been collected, a polygon is closed
        and the next vertex starts a new polygon.

        buf defaults to self._fence_buf if not provided.
        Returns: [[[lat, lon], ...], ...] — JSON-serialisable list of polygons.
        """
        polygons: list[list[list[float]]] = []
        current: list[list[float]] = []
        expected = 0
        for (lat, lon, vertex_count) in (buf if buf is not None else self._fence_buf):
            if expected == 0:
                expected = vertex_count
            current.append([lat, lon])
            if len(current) >= expected:
                polygons.append(current)
                current  = []
                expected = 0
        if current:
            polygons.append(current)
        return polygons

    @staticmethod
    def _ppm_inv(v: int) -> int:
        """PPM_INV: 3000 − v maps 1000↔2000 keeping 1500 centred."""
        return 3000 - v

    def _apply_servo_cmd(self, servo: int, pwm: int):
        """Apply a DO_SET_SERVO command: update servo state and publish to servo_state.

        Servo 5-8 map to PPM channels 4-7 (0-indexed).
        Published on servo_state topic — navigator subscribes, maintains the values,
        and includes them in every cmd_override tick so they are sent continuously.

        Servos 7 and 8 correspond to PPM CH7 and CH8, which apply_ppm_map inverts
        (PPM_CHn = 3000 − SBUS_CHn).  GQC's DO_SET_SERVO uses the SBUS/display
        convention (same values the user sees on the MK32 in manual mode), so we
        pre-invert here so the physical PPM output matches manual mode behaviour.
        """
        if servo not in self._servo_pwm:
            self.get_logger().warn(f'DO_SET_SERVO: servo {servo} out of range (5-8)')
            return
        pwm = max(1000, min(2000, pwm))
        # Servos 7 and 8 are inverted in the RP2040 ppm_map for manual mode.
        # Pre-invert so the autonomous PPM output is identical to manual mode.
        if servo in (7, 8):
            pwm = self._ppm_inv(pwm)
        self._servo_pwm[servo] = pwm
        rc = RCInput()
        rc.channels = [0, 0, 0, 0,
                       self._servo_pwm[5], self._servo_pwm[6],
                       self._servo_pwm[7], self._servo_pwm[8]]
        rc.mode  = 'SERVO'
        rc.stamp = self.get_clock().now().to_msg()
        self.servo_pub.publish(rc)
        inv_note = f' (hw={self._servo_pwm[servo]})' if servo in (7, 8) else ''
        self.get_logger().info(
            f'DO_SET_SERVO servo={servo} pwm={3000 - self._servo_pwm[servo] if servo in (7, 8) else self._servo_pwm[servo]} µs{inv_note}')

    def _cb_nav_params(self, msg: String):
        """Cache the latest navigator parameters (published every 5 s by navigator.py)."""
        try:
            self._nav_params = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'nav_params parse error: {e}')

    def _send_param_value_mav(self, mav_name: str, value: float, index: int, count: int):
        """Send a single PARAM_VALUE to GQC."""
        msg = self._mav.mav.param_value_encode(
            param_id=mav_name,        # pymavlink null-pads char[16] automatically
            param_value=value,
            param_type=9,             # MAV_PARAM_TYPE_REAL32
            param_count=count,
            param_index=index)
        self._send(msg)

    def _send_all_params(self):
        """Respond to PARAM_REQUEST_LIST — send all navigator params to GQC."""
        if not self._nav_params:
            self.get_logger().warn('PARAM_REQUEST_LIST: nav_params not yet received from navigator')
        count = len(_MAVLINK_PARAM_LIST)
        for i, mav_name in enumerate(_MAVLINK_PARAM_LIST):
            ros_name = _MAVLINK_PARAMS[mav_name]
            value    = float(self._nav_params.get(ros_name, 0.0))
            self._send_param_value_mav(mav_name, value, i, count)
            time.sleep(0.02)  # 20 ms gap to avoid UDP burst loss
        self.get_logger().info(f'PARAM_REQUEST_LIST: sent {count} params to GQC')

    # ── Resource management ───────────────────────────────────────────────────

    def _send_statustext(self, text: str, severity: int = 3):
        """Send MAVLink STATUSTEXT to GQC. severity: 3=WARNING (shown in MK32 log)."""
        try:
            msg = self._mav.mav.statustext_encode(severity, text[:50].encode('ascii', errors='ignore'))
            self._send(msg)
        except Exception as e:
            self.get_logger().warn(f'statustext send: {e}')

    def _check_resource_levels(self):
        """1 Hz: trigger base return if battery or tank falls below threshold during a mission."""
        if self._resource_state != 'normal':
            return  # already handling a resource event
        if not self._armed or self._mode != 'AUTONOMOUS':
            return  # only trigger during active autonomous operation
        if self._wp_active < 0:
            return  # no mission in progress

        batt_low  = self.get_parameter('battery_low_pct').value
        tank_low  = self.get_parameter('tank_low_pct').value

        # battery_remaining defaults to 0.0 in RoverStatus() when no real sensor is connected.
        # Treat 0.0 the same as -1 (unknown) — only check when the sensor reports a positive value.
        if self._test_batt_pct is not None:
            batt_pct: float | None = self._test_batt_pct
        else:
            raw = getattr(self._status, 'battery_remaining', -1)
            batt_pct = raw * 100 if raw > 0 else None

        # tank_level defaults to 0.0 in SensorData() when no sensor is connected.
        # Treat 0.0 the same as unknown — only check when test-injected OR sensor > 0.
        if self._test_tank is not None:
            tank: float | None = self._test_tank
        else:
            raw_tank = self._sensors.tank_level
            tank = raw_tank if raw_tank > 0 else None

        if batt_pct is not None and batt_pct < batt_low:
            self.get_logger().warn(f'[RESOURCE] Battery {batt_pct:.0f}% < {batt_low:.0f}%')
            self._trigger_return_to_base('battery')
        elif tank is not None and tank < tank_low:
            self.get_logger().warn(f'[RESOURCE] Tank {tank:.0f}% < {tank_low:.0f}%')
            self._trigger_return_to_base('tank')

    def _trigger_return_to_base(self, reason: str):
        """Disarm rover, save mission progress, upload single-waypoint base trip, notify GQC."""
        self._resource_state  = 'going_to_base'
        self._resource_reason = reason

        # Snapshot the current mission, servo map, fence, and rover position for later resume
        self._resume_mission   = list(self._mission_buf)
        self._resume_wp_seq    = self._wp_active
        self._resume_fence_buf = list(self._fence_buf)   # save before _internal_upload_mission clears it
        self._resume_position  = (self._fix.latitude, self._fix.longitude)
        # Renumber servo map relative to the resume waypoint (drop entries already passed)
        self._resume_servo_map = {
            (seq - self._resume_wp_seq): cmds
            for seq, cmds in self._wp_servo_map.items()
            if seq >= self._resume_wp_seq
        }

        # Stay armed + autonomous — the navigator will seamlessly switch to the
        # base-trip mission when the new seq=0 waypoint arrives.  Disarm only
        # happens when the rover reaches the base (wp_active=-1 in going_to_base).

        # Select the appropriate base station — runtime override takes priority over YAML
        if reason == 'battery':
            station_lat = self._recharge_lat if self._recharge_lat is not None \
                          else self.get_parameter('recharge_lat').value
            station_lon = self._recharge_lon if self._recharge_lon is not None \
                          else self.get_parameter('recharge_lon').value
            label       = 'BATTERY LOW'
        else:
            station_lat = self._water_lat if self._water_lat is not None \
                          else self.get_parameter('water_lat').value
            station_lon = self._water_lon if self._water_lon is not None \
                          else self.get_parameter('water_lon').value
            label       = 'TANK LOW'

        # Guard: abort if station has not been configured (default 0,0 means no station set)
        if station_lat == 0.0 and station_lon == 0.0:
            self.get_logger().error(
                f'[RESOURCE] {label} but {reason} station not configured — '
                f'set it in the GQC planner menu before running missions')
            self._send_statustext(f'{label}: no station set — configure in GQC')
            # Revert state so the check fires again next second (user must configure and re-arm)
            self._resource_state = 'normal'
            return
        accept_r = self.get_parameter('base_acceptance_m').value

        n_obs = len(self._resume_fence_buf)
        self.get_logger().warn(
            f'[RESOURCE] {label} — disarmed; base trip to ({station_lat:.6f},{station_lon:.6f}); '
            f'resume from WP{self._resume_wp_seq}; fence={n_obs} vertices')
        self._send_statustext(f'{label}: going to base, WP{self._resume_wp_seq} saved')

        # Build base-trip mission: WP0 = rover's current position, WP1 = base station.
        # Starting from the rover's own position lets the navigator reroute around any
        # obstacles that lie between the rover and the station.
        wp_here = MissionWaypoint()
        wp_here.seq               = 0
        wp_here.latitude          = self._fix.latitude
        wp_here.longitude         = self._fix.longitude
        wp_here.speed             = 0.5
        wp_here.acceptance_radius = float(accept_r)
        wp_here.hold_secs         = 0.0

        wp_base = MissionWaypoint()
        wp_base.seq               = 1
        wp_base.latitude          = station_lat
        wp_base.longitude         = station_lon
        wp_base.speed             = 0.5
        wp_base.acceptance_radius = float(accept_r)
        wp_base.hold_secs         = 0.0

        self._internal_upload_mission([wp_here, wp_base],
                                      fence_buf=self._resume_fence_buf)

    def _internal_upload_mission(self, waypoints: list, fence_buf: list | None = None):
        """Upload a mission generated internally (not from GQC).

        Publishes waypoints and optional obstacle fence to the navigator via ROS2 topics
        and broadcasts MISSION_COUNT + MISSION_ITEM_INT (+ fence items) over UDP so GQC
        and mission_planner display the new mission with obstacles.

        fence_buf: list of (lat, lon, vertex_count) tuples — same format as self._fence_buf.
                   Pass the saved _resume_fence_buf so the navigator reroutes around
                   obstacles on both the base trip and the resume leg.
        """
        with self._mission_lock:
            self._mission_count      = len(waypoints)
            self._mission_buf        = list(waypoints)
            self._fence_buf          = list(fence_buf) if fence_buf else []
            self._mission_src        = None
            self._mission_expect_seq = None
        self._wp_servo_map    = {}
        self._last_nav_wp_seq = None

        # Publish new waypoints — seq=0 resets the navigator's path automatically.
        # Do NOT publish mission_clear here: DDS does not guarantee inter-topic
        # ordering, so the clear may arrive AFTER wp[0] and wipe it out.
        for wp in waypoints:
            self.mission_pub.publish(wp)
        # Publish fence (may be empty for base trip without obstacles)
        polys = self._parse_fence_polygons(fence_buf) if fence_buf else []
        fence_msg = String()
        fence_msg.data = json.dumps({'polygons': polys})
        self.fence_pub.publish(fence_msg)

        # Broadcast via MAVLink so GQC and mission_planner see the new mission + obstacles
        threading.Thread(
            target=self._broadcast_mission_mavlink,
            args=(list(waypoints), list(fence_buf) if fence_buf else []),
            daemon=True
        ).start()

    def _broadcast_mission_mavlink(self, waypoints: list, fence_buf: list | None = None):
        """Send MISSION_COUNT + fence items + MISSION_ITEM_INT to broadcast AND unicast."""
        fence_buf = fence_buf or []
        def _sendto(buf):
            self._udp_sock.sendto(buf, self._gqc_addr)
            if self._gqc_unicast:
                self._udp_sock.sendto(buf, self._gqc_unicast)
        try:
            total = len(fence_buf) + len(waypoints)
            count_msg = self._mav.mav.mission_count_encode(0, 0, total, 0)
            _sendto(count_msg.pack(self._mav.mav))
            time.sleep(0.15)
            seq = 0
            # Fence vertices first (cmd=5003) so GQC/planner reconstruct obstacle polygons
            for (lat, lon, vertex_count) in fence_buf:
                item = self._mav.mav.mission_item_int_encode(
                    target_system=0, target_component=0,
                    seq=seq, frame=0, command=5003,
                    current=0, autocontinue=1,
                    param1=float(vertex_count), param2=0.0, param3=0.0, param4=0.0,
                    x=int(lat * 1e7), y=int(lon * 1e7), z=0.0, mission_type=0)
                _sendto(item.pack(self._mav.mav))
                seq += 1
                time.sleep(0.02)
            # Waypoints
            for wp in waypoints:
                item = self._mav.mav.mission_item_int_encode(
                    target_system=0, target_component=0,
                    seq=seq, frame=6, command=16,
                    current=0, autocontinue=1,
                    param1=float(wp.hold_secs), param2=0.0, param3=0.0, param4=0.0,
                    x=int(wp.latitude  * 1e7), y=int(wp.longitude * 1e7),
                    z=float(wp.speed), mission_type=0)
                _sendto(item.pack(self._mav.mav))
                seq += 1
                time.sleep(0.02)
            # Follow up with a clean waypoints-only download so GQC reliably
            # updates roverMissions even if a broadcast packet was lost.
            time.sleep(0.10)
            self._send_mission_to_gqc()
        except Exception as e:
            self.get_logger().warn(f'_broadcast_mission_mavlink: {e}')

    def _upload_resume_mission(self):
        """Build resume mission: base station → stopped position → remaining WPs.

        Called at the at_base transition so GQC has the mission ready for the
        proximity check (WP0 = rover's current position at the base station).
        Does NOT reset _resource_state — the ARM handler does that.
        """
        wps = self._resume_mission[self._resume_wp_seq:]
        if not wps:
            self.get_logger().warn('[RESOURCE] Resume mission empty — clearing resource state')
            self._resource_state = 'normal'
            return

        renumbered = []
        seq = 0

        # WP0 — rover's current position (base station) so GQC proximity check passes
        wp_here               = MissionWaypoint()
        wp_here.seq               = seq
        wp_here.latitude          = self._fix.latitude
        wp_here.longitude         = self._fix.longitude
        wp_here.speed             = wps[0].speed
        wp_here.acceptance_radius = wps[0].acceptance_radius
        wp_here.hold_secs         = 0.0
        renumbered.append(wp_here)
        seq += 1

        # WP1 — position where rover stopped (return there before continuing)
        if self._resume_position is not None:
            wp_return               = MissionWaypoint()
            wp_return.seq               = seq
            wp_return.latitude          = self._resume_position[0]
            wp_return.longitude         = self._resume_position[1]
            wp_return.speed             = wps[0].speed
            wp_return.acceptance_radius = wps[0].acceptance_radius
            wp_return.hold_secs         = 0.0
            renumbered.append(wp_return)
            seq += 1

        # Remaining original waypoints
        for orig in wps:
            wp               = MissionWaypoint()
            wp.seq               = seq
            wp.latitude          = orig.latitude
            wp.longitude         = orig.longitude
            wp.speed             = orig.speed
            wp.acceptance_radius = orig.acceptance_radius
            wp.hold_secs         = orig.hold_secs
            renumbered.append(wp)
            seq += 1

        n_obs = len(self._resume_fence_buf)
        self.get_logger().info(
            f'[RESOURCE] Resume mission ready: {len(renumbered)} WPs '
            f'(base → stopped → WP{self._resume_wp_seq}+), fence={n_obs} vertices')
        self._send_statustext(
            f'Resume ready — re-arm to continue from WP{self._resume_wp_seq}')
        self._internal_upload_mission(renumbered, fence_buf=self._resume_fence_buf)

        # Restore servo map AFTER _internal_upload_mission (which clears it).
        # Shift keys up by the number of prepended waypoints (base + stopped pos).
        prepended = seq - len(wps)
        self._wp_servo_map = {
            (k + prepended): v for k, v in self._resume_servo_map.items()
        }

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
            arming = (msg.param1 == 1.0)
            if arming and self._resource_state == 'at_base':
                # Resume mission was already uploaded at at_base transition —
                # just clear resource state so _check_resource_levels resumes.
                self._resource_state = 'normal'
            self._armed = arming
            self.get_logger().info(f'{"ARM" if self._armed else "DISARM"} command received')
            a = Bool(); a.data = self._armed
            self.armed_pub.publish(a)
            if not self._armed:
                # Disarm exits autonomous mode so navigator stops
                self._mode = 'MANUAL'
                m = String()
                m.data = 'MANUAL'
                self.mode_pub.publish(m)
            self._send(self._mav.mav.command_ack_encode(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED))
        elif msg.command == 176:  # MAV_CMD_DO_SET_MODE
            # param2 = custom_mode: 0 → MANUAL, non-zero → AUTONOMOUS
            new_mode = 'AUTONOMOUS' if int(msg.param2) != 0 else 'MANUAL'
            self._mode = new_mode
            m = String()
            m.data = new_mode
            self.mode_pub.publish(m)
            self.get_logger().info(f'SET_MODE → {new_mode}')
            self._send(self._mav.mav.command_ack_encode(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED))
        elif msg.command == 183:  # MAV_CMD_DO_SET_SERVO
            self._apply_servo_cmd(int(msg.param1), int(msg.param2))
            self._send(self._mav.mav.command_ack_encode(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED))
        elif msg.command == 50001:  # Set battery/recharge station (from GQC Set Battery Station)
            self._recharge_lat = float(msg.param1) / 1e5
            self._recharge_lon = float(msg.param2) / 1e5
            self.get_logger().info(
                f'[RESOURCE] Battery station set: ({self._recharge_lat:.5f},{self._recharge_lon:.5f})')
            self._send_statustext(f'Battery station updated')
            self._send(self._mav.mav.command_ack_encode(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED))
        elif msg.command == 50002:  # Set water/tank station (from GQC Set Water Station)
            self._water_lat = float(msg.param1) / 1e5
            self._water_lon = float(msg.param2) / 1e5
            self.get_logger().info(
                f'[RESOURCE] Water station set: ({self._water_lat:.5f},{self._water_lon:.5f})')
            self._send_statustext(f'Water station updated')
            self._send(self._mav.mav.command_ack_encode(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED))
        elif msg.command == 50003:  # Reroute response from GQC (confirm/reject)
            accepted = (msg.param1 == 1.0)
            resp = Bool(); resp.data = accepted
            self.reroute_response_pub.publish(resp)
            self._reroute_pending = False
            self.get_logger().info(
                f'Reroute {"accepted" if accepted else "rejected"} by user')
            self._send(self._mav.mav.command_ack_encode(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED))

    def _on_mission_item(self, msg):
        if msg.get_srcSystem() != 255:
            return  # ignore re-broadcasts from other rovers (feedback loop prevention)
        with self._mission_lock:
            expected = self._mission_expect_seq
            rtt_ms = (time.monotonic() - self._mission_last_req_t) * 1000
        if expected is None or msg.seq != expected:
            return  # duplicate or out-of-order — ignore
        self.get_logger().info(f'ITEM seq={msg.seq} RTT={rtt_ms:.0f}ms')
        # Re-broadcast for passive tools (monitor.py) — GQC sends unicast.
        try:
            fwd = self._mav.mav.mission_item_int_encode(
                0, 0, msg.seq, msg.frame, msg.command, msg.current,
                msg.autocontinue, msg.param1, msg.param2, msg.param3, msg.param4,
                msg.x, msg.y, msg.z, getattr(msg, 'mission_type', 0))
            self._udp_sock.sendto(fwd.pack(self._mav.mav), self._gqc_addr)
        except Exception:
            pass

        if msg.command == 183:  # MAV_CMD_DO_SET_SERVO
            sn, pw = int(msg.param1), int(msg.param2)
            if self._last_nav_wp_seq is None:
                # Before any NAV_WAYPOINT → apply immediately as initial servo state
                self._apply_servo_cmd(sn, pw)
            else:
                # After a NAV_WAYPOINT → defer: apply when navigator reaches that waypoint
                if 5 <= sn <= 8:
                    if self._last_nav_wp_seq not in self._wp_servo_map:
                        self._wp_servo_map[self._last_nav_wp_seq] = {}
                    self._wp_servo_map[self._last_nav_wp_seq][sn] = pw
                    self.get_logger().info(
                        f'Deferred servo CH{sn}={pw} → wp_seq={self._last_nav_wp_seq}')
        elif msg.command == 5003:  # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
            # Collect fence polygon vertices — published as ~/mission_fence JSON
            # when the full mission has been received.
            self._fence_buf.append((msg.x * 1e-7, msg.y * 1e-7, int(msg.param1)))
        else:
            wp = MissionWaypoint()
            # Normalize seq to 0-based index within nav waypoints only.
            # When fence items (cmd=5003) are prepended, the first nav waypoint
            # has msg.seq = N_fence (not 0), which would prevent the navigator from
            # detecting a new mission (it resets on seq==0).  Using len(_mission_buf)
            # ensures seq is always 0 for the first waypoint regardless of fence items.
            wp.seq              = len(self._mission_buf)
            wp.latitude         = msg.x * 1e-7
            wp.longitude        = msg.y * 1e-7
            wp.speed            = float(msg.z)    if msg.z    > 0.0 else 0.0
            wp.hold_secs        = float(msg.param1) if msg.param1 > 0.0 else 0.0
            wp.acceptance_radius = 0.0  # 0 → navigator uses default_acceptance_radius from YAML
            self._last_nav_wp_seq = wp.seq  # track for deferred servo association
            self._mission_buf.append(wp)
            self.mission_pub.publish(wp)

        next_seq = msg.seq + 1
        if next_seq < self._mission_count:
            with self._mission_lock:
                self._mission_expect_seq = next_seq
                self._mission_last_req_t = time.monotonic()
            # Don't immediately send REQUEST_INT — GQC streams items proactively so the
            # next item may already be in flight.  The retry timer sends REQUEST_INT after
            # 250 ms only if the item hasn't arrived yet (handles packet loss fallback).
        else:
            with self._mission_lock:
                self._mission_expect_seq = None
                self._mission_src        = None
            addr = self._gqc_unicast or self._gqc_addr
            packed = self._mav.mav.mission_ack_encode(
                msg.get_srcSystem(), msg.get_srcComponent(),
                mavutil.mavlink.MAV_MISSION_ACCEPTED).pack(self._mav.mav)
            try:
                self._udp_sock.sendto(packed, addr)
            except Exception as e:
                self.get_logger().warn(f'mission_ack send error: {e}')
            self.get_logger().info(
                f'Mission received: {self._mission_count} items '
                f'({len(self._mission_buf)} waypoints, {len(self._fence_buf)} fence vertices)')
            # Publish fence polygons to navigator — always, even if empty.
            # An empty polygon list signals mission-complete so the navigator
            # publishes its planned path back to GQC for the map overlay.
            polys = self._parse_fence_polygons() if self._fence_buf else []
            fence_msg = String()
            fence_msg.data = json.dumps({'polygons': polys})
            self.fence_pub.publish(fence_msg)


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
