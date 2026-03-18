# ROS2 Workspace — CLAUDE.md

See root CLAUDE.md for system-wide context. This file covers ROS2-specific details.

## Package entry points

| Package                 | Main file                                        |
|-------------------------|--------------------------------------------------|
| agri_rover_interfaces   | msg/, srv/, action/ — interface definitions only |
| agri_rover_rp2040       | agri_rover_rp2040/rp2040_bridge.py               |
| agri_rover_gps          | agri_rover_gps/gps_driver.py                     |
| agri_rover_mavlink      | agri_rover_mavlink/mavlink_bridge.py             |
| agri_rover_navigator    | agri_rover_navigator/navigator.py                |
| agri_rover_sensors      | agri_rover_sensors/sensor_node.py                |
| agri_rover_video        | agri_rover_video/video_streamer.py (RTSP node) + launch/video_pipeline.launch.py (NITROS container) |
| agri_rover_simulator    | agri_rover_simulator/simulator_node.py          |
| agri_rover_bringup      | launch/rover1.launch.py, launch/rover2.launch.py |

## Interface definitions

**Messages:**
- `RCInput`: `uint16[16] channels`, `string mode`, `bool sbus_ok`, `bool rf_link_ok`, `Time stamp`
- `RoverStatus`: `uint8 rover_id`, `bool armed`, `string mode`, `string gps_fix_type`, `float32 battery_voltage/remaining`, `string rc/rf_link_status`, `Time stamp`
- `SensorData`: `float32 tank_level/temperature/humidity/pressure`, `Time stamp`
- `MissionWaypoint`: `uint16 seq`, `float64 latitude/longitude`, `float32 speed`, `float32 acceptance_radius`, `bool hold`

**Services:**
- `SetMode`: request `uint8 mode` (0=MANUAL,1=AUTO,2=EMERGENCY) → response `bool success, string message`
- `ArmDisarm`: request `bool arm` → response `bool success, string message`

**Action:**
- `FollowMission`: goal `MissionWaypoint[] waypoints, float32 default_speed` → result `bool success, uint32 waypoints_completed` → feedback `uint32 current_waypoint_seq, float32 distance_to_next/heading_error/cross_track_error`

## Node patterns

All nodes follow this structure:
```python
class XxxNode(Node):
    def __init__(self):
        super().__init__('node_name')
        self.declare_parameter('key', default)   # always declare before use
        # publishers, subscribers, timers
        # NO blocking calls in __init__

def main(args=None):
    rclpy.init(args=args)
    node = XxxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## rp2040_bridge — serial parsing

```python
# Read from RP2040:
line = ser.readline().decode('ascii', errors='ignore').strip()
# Parse: "CH:1500,1500,...,1500 MODE:MANUAL"  (16 channels)
ch_part, mode_str = line.split(' MODE:')
channels = [int(x) for x in ch_part[3:].split(',')]  # len == 16

# Write heartbeat to RP2040:
ser.write(f'<HB:{seq}>\n'.encode())

# Write autonomous command:
ser.write(f'<J:{",".join(str(c) for c in channels[:8])}>\n'.encode())
```

Lock `self._lock` around all `ser.write()` calls (called from both timer and subscription callbacks).

Status messages from RP2040 (not CH: lines) are parsed separately:
```python
elif raw == '[RF_LINK_OK]':    self._rf_link_ok = True
elif raw == '[RF_LINK_LOST]':  self._rf_link_ok = False
```
`rf_link_ok` is stamped onto every published `RCInput` message. `sbus_ok` tracking works the same way via `[SBUS_OK]`/`[SBUS_LOST]`.

## mavlink_bridge — key patterns

```python
os.environ['MAVLINK20'] = '1'          # must be set before pymavlink import
from pymavlink import mavutil

# Connection: listen for inbound + sendto for outbound
self._mav = mavutil.mavlink_connection(f'udpin:0.0.0.0:{bind_port}', ...)
self._gqc_addr   = (gqc_host, gqc_port)   # broadcast addr (subnet .255)
self._gqc_unicast = None                   # set to (ip, port) once first GQC packet arrives

# Send — dedicated outbound socket created in __init__ (pymavlink internals vary by version):
#   self._udp_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
#   self._udp_sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_BROADCAST, 1)
# _send() sends BOTH broadcast (gqc_addr) AND unicast (gqc_unicast) if GQC is known.
#   Broadcast: ensures monitor.py and simulator.py (passive tools) always receive telemetry.
#   Unicast:   bypasses WiFi AP DTIM buffering (~100 ms) so GQC gets immediate delivery.
# Close in main(): node._udp_sock.close()

# Receive loop — uses raw socket, NOT recv_match():
#   sock = self._mav.port          # mavudp stores the DatagramSocket here
#   data, (src_ip, src_port) = sock.recvfrom(1024)
#   msgs = self._mav.mav.parse_buffer(data)  # parse with pymavlink's protocol object
#   if msg.get_srcSystem() == 255:            # only record GQC address (not other rovers!)
#       self._gqc_unicast = (src_ip, gqc_port)  # force port — GQC RX port ≠ OS send port
#
# WHY not recv_match(): pymavlink buffers packets — recv_match() can return a buffered
# message without calling recvfrom(), leaving last_address stale/None. Using recvfrom()
# directly guarantees the source IP is captured from every packet.
#
# WHY sysid==255 filter: without it, rovers would record each other's IPs and unicast
# telemetry to each other instead of to GQC.
#
# WHY port forced to gqc_port: Android GQC binds :14550 for receiving, but the OS may
# assign a different ephemeral source port for outbound packets — using src_port would
# send to the wrong port.
#
# Mission handshake (MISSION_REQUEST_INT) uses unicast via _send_mission_request():
#   addr = self._gqc_unicast or self._gqc_addr   # unicast once GQC discovered
#   self._udp_sock.sendto(packed, addr)
# This eliminates the WiFi AP DTIM buffering that caused ~100ms per-item delay.
```

Subscriptions and state:
- Subscribes to `heading`, `cmd_override`, `wp_active`, `xte` in addition to `fix`, `rc_input`, `sensors`, `status`, `mode`
- `self._heading_deg = None` until first heading message; sent as `hdg` (cdeg) in `GLOBAL_POSITION_INT`
- `GLOBAL_POSITION_INT.hdg = int(heading_deg * 100) % 36000`  (65535 = unknown, used until first fix)
- `self._xte` updated from `xte` topic; included in `NAMED_VALUE_FLOAT` broadcast as `'XTE'`
- `NAMED_VALUE_FLOAT` names sent: `TANK`, `TEMP`, `HUMID`, `PRESSURE`, `CMD_T`, `CMD_S`, `WP_ACT`, `WP_TOT`, `XTE`

HEARTBEAT `base_mode` flags:
- `MAV_MODE_FLAG_SAFETY_ARMED` (128) set when `self._armed`
- `MAV_MODE_FLAG_GUIDED_ENABLED` (8) set when `self._mode == 'AUTONOMOUS'`
- Android app `checkLinkMismatch()` reads bit `0x08` to detect slave autonomous state

RC_CHANNELS `chancount` field uses `len(self._rc.channels)` (was incorrectly hardcoded to 9).

Inbound MAVLink command handling (`_on_command_long`):
- `MAV_CMD_COMPONENT_ARM_DISARM` (400): updates `self._armed`, sends ACK; DISARM also publishes `'MANUAL'` to `mode` topic (stops navigator)
- `MAV_CMD_DO_SET_MODE` (176): param2=0 → `'MANUAL'`, param2≠0 → `'AUTONOMOUS'`; publishes to `mode` topic and sends ACK. **Required in simulation** (no rp2040_bridge serial) to activate the navigator.
- `MAV_CMD_DO_SET_SERVO` (183): calls `_apply_servo_cmd(servo, pwm)`, sends ACK

`_apply_servo_cmd(servo, pwm)`:
- Validates servo in range 5–8; clamps pwm to 1000–2000
- Updates `self._servo_pwm[servo]`
- Publishes full servo state to `servo_state` topic (channels 4-7 set, channels 0-3 = 0)
- Navigator receives it via `_cb_servo_state` and re-publishes in every cmd_override

`_on_mission_item` skips `DO_SET_SERVO` items for waypoint publishing (lat=0/lon=0 would navigate to null island); calls `_apply_servo_cmd` instead.

## navigator — Stanley path follower

Key parameters (in `config/navigator_params.yaml`):
- `lookahead_distance: 3.0`     metres ahead on segment for heading error θ_e
- `default_acceptance_radius: 0.3` metres — waypoint reached (RTK precision)
- `max_speed: 1.5`              m/s → PPM throttle range 1500–2000
- `min_speed: 0.3`              m/s floor (never slower regardless of waypoint speed)
- `max_steering: 0.8`           fraction of full deflection (0.0–1.0)
- `control_rate: 25.0`          Hz — matches RTK GPS update rate
- `gps_timeout: 2.0`            seconds before halt
- `align_threshold: 30.0`       degrees — spin-in-place only above this; normal curves move
- `stanley_k: 1.0`              cross-track error gain (increase = tighter path hug)
- `stanley_softening: 0.3`      m/s denominator floor (prevents div/0 at low speed)

**Stanley controller formula:**
```
δ = θ_e + arctan(stanley_k × e_cte / max(v, stanley_softening))
```
- `θ_e` — heading error to lookahead point (degrees)
- `e_cte` — signed cross-track error; positive = rover is LEFT of segment
- `v` — current target speed (m/s)
- `δ` — total steering correction, clamped to ±90°, mapped to PPM via `steer_frac = δ/45`

Speed is held constant at the recorded waypoint speed. No steering-based slowdown.

**Waypoint advance logic:**
- `dist < acceptance_radius` → reached; if `hold_secs > 0` pause then advance
- Overshoot detection: rover past waypoint along segment direction → advance (no 180° turn)

**XTE publication:**
- `_cross_track_error()` uses 2D cross product in flat-earth metres
- Publishes absolute XTE to `xte` topic (Float32) at every 25 Hz tick
- mavlink_bridge forwards as `NAMED_VALUE_FLOAT 'XTE'` to GQC / monitor.py

Coordinate math:
```python
def haversine(lat1, lon1, lat2, lon2) -> float   # distance in metres
def bearing_to(lat1, lon1, lat2, lon2) -> float  # degrees 0-360
```

PPM mapping:
- Throttle: `PPM_CENTER + (v_mps / max_speed) * 500`  (1500 neutral, 2000 = max)
- Steering: `PPM_CENTER - steer_frac * 500`  (< 1500 = right, > 1500 = left)

Servo channels (PPM CH5-CH8):
- Navigator subscribes to `servo_state` (RCInput from mavlink_bridge).
- `_cb_servo_state()` updates `self._servo_ch[0..3]` (channels 4-7, non-zero values only).
- `_publish_cmd()` always includes `_servo_ch` in channels 4-7 of every `cmd_override` message.
- Servo state is continuously re-sent at 25 Hz — an RP2040 reset does not lose servo state.

## gps_driver — NMEA parsing

Reads two serial ports in background threads. Publishes at 5 Hz via timer.
- **Primary** (USB0): position, fix quality from GGA sentences
- **Secondary** (USB1): position only — baseline vector → heading
- Heading: `bearing = atan2(dlon * cos(lat_rad), dlat)` — `cos(lat)` correction is required because `secondary_pos()` scales `dlon` by `1/cos(lat)` when placing the secondary antenna; without the correction heading drifts as the rover moves and latitude changes.
- Fix quality map: `'0'→NO_FIX, '1'→GPS, '2'→DGPS, '4'→RTK_FIX, '5'→RTK_FLT`

## bringup — launch conventions

- **`rover1.launch.py`**: hardcodes namespace `'/rv1'`, loads `config/rover1_params.yaml`
- **`rover2.launch.py`**: hardcodes namespace `'/rv2'`, loads `config/rover2_params.yaml`
- All nodes receive parameters via `parameters=[config]` pointing to the YAML
- YAML key must match `node_name.ros__parameters.param_name` (ROS2 YAML convention)
- `camera_source` launch arg (default `csi`) selects video mode:
  - `csi` → `IncludeLaunchDescription(video_pipeline.launch.py)` + `video_streamer` in Isaac ROS mode
  - `usb`/`test` → `video_streamer` in GStreamer subprocess mode; NITROS container NOT launched

## agri_rover_video — Isaac ROS pipeline

```
camera_source=csi:
  [ComposableNodeContainer /rvN/video_container]
    ArgusMonoNode  → camera/image_raw  (NITROS)
    EncoderNode    → image_compressed  (NITROS, format="h264")
  [VideoStreamerNode /rvN/video_streamer]
    subscribes image_compressed → GstRtspServer appsrc → rtsp://<ip>:<port>/stream

camera_source=usb|test:
  [VideoStreamerNode /rvN/video_streamer]
    spawns gst-rtsp-server subprocess directly
```

- `ArgusMonoNode` and `EncoderNode` must be in the same `component_container_mt` for zero-copy NITROS.
- `VideoStreamerNode` subscribes to `sensor_msgs/CompressedImage` (format `"h264"`).
- `_H264AppsrcFactory.push_frame()` is called from the ROS2 subscription callback — thread-safe via `threading.Lock`.
- The `GLib.MainLoop` (GstRtspServer event loop) runs in a daemon thread.

## agri_rover_simulator — key patterns

```python
# PPM decoder line parsing
raw = ser.readline().decode('ascii', errors='ignore').strip()
# "RV1:ch0,ch1,...,ch7 RV2:ch0,...,ch7"
rv1_part, rv2_part = raw.split(' ')
rv1_vals = [int(v) for v in rv1_part[4:].split(',')]  # skip "RV1:"
rv2_vals = [int(v) for v in rv2_part[4:].split(',')]  # skip "RV2:"

# Secondary GPS position (antenna_baseline_m ahead of primary)
sec_lat = lat + (baseline_m * math.cos(heading_rad)) / 111320.0
sec_lon = lon + (baseline_m * math.sin(heading_rad)) / (111320.0 * math.cos(math.radians(lat)))

# NMEA output (GGA RTK fixed quality=4)
body = f'GNGGA,{utc},{lat_s},{lat_h},{lon_s},{lon_h},4,12,0.5,{alt:.1f},M,0.0,M,0.5,0001'
sentence = f'${body}*{checksum(body)}\r\n'
```

- `heading_source` parameter added to `gps_driver`:
  - `'baseline'` (default): heading from secondary–primary baseline vector (real hardware + simulator)
  - `'vtg'`: heading from `$GNVTG` COG field on primary port only (single-port mode — optional)
- Simulator always outputs 4 NMEA streams (primary + secondary per rover). Rover Jetsons need no config changes.
- PPM timeout: if no ppm_decoder data for >2 s, simulator freezes rovers at current position (uses neutral 1500).
- Physics model is **skid/differential steer** — `omega = turn_scale * 2 * steer * max_speed / wheelbase` (independent of forward speed, allows in-place spinning).
- `turn_scale` (0.0–1.0, default 0.1): scales max turn rate. Pass `--turn-scale` in `tools/simulator.py` or set in `simulator_params.yaml`.

## colcon tips

```bash
# Build only changed packages (faster):
colcon build --symlink-install --packages-select agri_rover_mavlink

# Check node is running:
ros2 node list
ros2 topic echo /rv1/rc_input

# Inspect parameters at runtime:
ros2 param list /rv1/rp2040_bridge
ros2 param get /rv1/navigator lookahead_distance
```
