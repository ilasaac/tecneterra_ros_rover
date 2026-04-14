# ROS2 Workspace — CLAUDE.md

See root `CLAUDE.md` for system-wide context. This file covers ROS2-specific details.

## Package entry points

| Package                 | Main file                                        |
|-------------------------|--------------------------------------------------|
| agri_rover_interfaces   | msg/, srv/, action/ — interface definitions only |
| agri_rover_rp2040       | agri_rover_rp2040/rp2040_bridge.py               |
| agri_rover_gps          | agri_rover_gps/gps_driver.py                     |
| agri_rover_navigator    | agri_rover_navigator/navigator.py                |
| agri_rover_sensors      | agri_rover_sensors/sensor_node.py                |
| agri_rover_video        | agri_rover_video/video_streamer.py + launch/video_pipeline.launch.py |
| agri_rover_simulator    | agri_rover_simulator/simulator_node.py          |
| agri_rover_bringup      | launch/rover{1,2}.launch.py, launch/sim_harness.launch.py |

`agri_rover_mavlink` was removed in the corridor-only rewrite — all GQC
traffic is over rosbridge now.

## Interface definitions

**Messages:**
- `RCInput`: `uint16[16] channels`, `string mode`, `bool sbus_ok`, `bool rf_link_ok`, `Time stamp`
- `RoverStatus`: `uint8 rover_id`, `bool armed`, `string mode`, `string gps_fix_type`, `float32 battery_voltage/remaining`, `Time stamp`
- `SensorData`: `float32 tank_level/temperature/humidity/pressure`, `Time stamp`
- `MissionWaypoint`: `uint16 seq`, `float64 latitude/longitude`, `float32 speed`, `float32 acceptance_radius`, `float32 hold_secs`
  - Still used internally to store corridor polyline points; not exposed as a user mission format.

## Node patterns

```python
class XxxNode(Node):
    def __init__(self):
        super().__init__('node_name')
        self.declare_parameter('key', default)
        # publishers, subscribers, timers
        # NO blocking calls in __init__
```

## rp2040_bridge — serial parsing

```python
# Read from RP2040:
line = ser.readline().decode('ascii', errors='ignore').strip()
# "CH:1500,1500,...,1500 MODE:MANUAL"
ch_part, mode_str = line.split(' MODE:')
channels = [int(x) for x in ch_part[3:].split(',')]  # len == 16

# Write heartbeat + autonomous command:
ser.write(f'<HB:{seq}>\n'.encode())
ser.write(f'<J:{",".join(str(c) for c in channels[:8])}>\n'.encode())
```

Lock `self._lock` around all `ser.write()` calls (timer + sub callbacks).

Status messages parsed separately; initialised **True** (optimistic)
because the RP2040 only emits `[SBUS_OK]`/`[RF_LINK_OK]` on state change.

## navigator — Stanley corridor controller

**Key parameters** (`config/navigator_params.yaml` or `rover{1,2,3}_params.yaml`):

| Param | Default | Purpose |
|-------|---------|---------|
| `lookahead_distance` | 3.0 m | lookahead for heading error θ_e |
| `default_acceptance_radius` | 0.3 m | turn-point advance threshold |
| `max_speed` / `min_speed` | 1.5 / 0.3 m/s | throttle range |
| `max_steering` | 0.8 | fraction of full deflection |
| `control_rate` | 25 Hz | matches GPS |
| `gps_timeout` | 2 s | halt after stale fix or stale hAcc |
| `gps_accuracy_alarm_mm` | 200 | halt if hAcc > threshold or hAcc stops |
| `align_threshold` | 30° | spin-in-place only above this; else Stanley |
| `stanley_k` | 1.0 | CTE gain |
| `stanley_softening` | 0.3 m/s | denominator floor |
| `stanley_cte_scale_m` | 1.0 | CTE at which speed halves |
| `stanley_cte_alarm_m` | 1.0 | disarm threshold |
| `min_turn_radius_m` | 2.0 m | mission validator rejects tighter arcs |
| `post_turn_speed` | 0.5 m/s | speed at first point after a turn arc |
| `min_throttle_ppm` | 1550 | stiction floor for forward motion |
| `min_steer_ppm_delta` | 50 | stiction floor during active turns |
| `steer_coast_angle` | 5° | below this |heading_err|, skip the steer floor |

**Stanley control law:**
```
δ = θ_e + arctan(stanley_k × e_cte / max(v, stanley_softening))
```
- `θ_e` — heading error to lookahead point
- `e_cte` — signed CTE at the **front antenna** (Stanley front-axle reference)
- `v` — current target speed
- Output clamped to ±90°, mapped via `steer_frac = δ/45`

**Dual-antenna positioning:**
- `fix` (rear antenna) + `fix_front` (front antenna) both subscribed
- `_center_pos()` = midpoint — acceptance/lookahead/overshoot
- `_front_pos()` = front antenna — CTE only
- Falls back to `fix` alone if `fix_front` missing

**Mission flow:**
1. `corridor_mission` topic (JSON) arrives
2. Parse via `agri_rover_navigator.corridor`
3. `corridors_to_path` → polyline of `(lat, lon, speed, half_width, is_turn, servo)` tuples
4. **Mission validator:** at every interior point compute `R = chord / (2·sin(turn/2))`; if `min(R) < min_turn_radius_m`, log the offending index, publish disarm, return
5. Populate `_path[]`, `_path_s[]`, `_corridor_widths[]`, `_corridor_turn_indices`
6. Apply per-point servo state

**Control loop** (`_control_loop_corridor`):
- Stanley CTE on the polyline, lookahead clamped to next turn point
- Decel ramp: linear over `max(1, lookahead × 2)` m before each turn
- Turn advance: when `s_nearest > turn_s − accept_r`, drop the turn from `_corridor_turn_indices` and advance `_path_idx` — no on-axis spin
- Align-spin only when `|heading_err| > align_threshold`; once inside the deadband, normal Stanley takes over and never re-enters spin

**Full-path helpers:**
- `_nearest_on_path(lat, lon)` — never backtracks; returns `(s_nearest, seg_idx)`
- `_cte_to_seg(lat, lon, seg_idx)` — signed perpendicular distance
- `_point_at_s(s_target)` — interpolates a point at any arc-length
- `_turn_angle_at(idx)` — heading change at `_path[idx]` (0 for endpoints)

**Inter-rover proximity safety:**
- Enabled by `peer_rover_ns: '/rv2'` (on RV1) or `'/rv1'` (on RV2); empty disables
- Each navigator subs to `{peer_ns}/fix` and `{peer_ns}/heading` (absolute paths)
- `_prox_corners()` builds a 4-corner quad from rear antenna + heading + overhangs
- `_rects_clearance()` — minimum corner-to-edge distance between the two quads
- Three levels:
  - `slow` < `proximity_slow_m` (1.5 m) → halve throttle
  - `halt` < `proximity_halt_m` (1.0 m) → publish_halt + return
  - `estop` < `proximity_estop_m` (0.5 m) → halt + publish `MANUAL` to peer's `mode` topic
- Stale peer GPS (> `gps_timeout`) → level falls back to `ok`

**Live parameter tuning (GQC via rosbridge):**
- `_PARAM_META` dict lists all tunable params with bounds
- `nav_params` topic publishes current values @5 Hz as JSON
- `nav_param_set` topic receives set requests; `_cb_nav_param_set` validates bounds, updates state, republishes

**PPM mapping:**
- Throttle: `PPM_CENTER + (v_mps / max_speed) × 500` (1500 neutral, 2000 max)
- Steering: `PPM_CENTER − steer_frac × 500` (< 1500 = right, > 1500 = left)

**Servo channels (CH5-CH8):**
- Navigator subs `servo_state` (from rosbridge / GQC DO_SET_SERVO)
- `_cb_servo_state` updates `self._servo_ch[0..3]`
- `_publish_cmd` always stamps `_servo_ch` into every `cmd_override`
- Continuous 25 Hz re-publish — RP2040 reset doesn't lose servo state

## gps_driver — NMEA + UBX

- **Primary** (USB0): position/quality from GGA → `~/fix` (rear antenna)
- **Secondary** (USB1): position only; baseline → heading → `~/fix_front`
- Heading: `atan2(dlon × cos(lat_rad), dlat)` — `cos(lat)` is required
- Fix quality: `0→NO_FIX, 1→GPS, 2→DGPS, 4→RTK_FIX, 5→RTK_FLT`
- `~/hacc` resets to `-1` (unknown) when the last UBX NAV-PVT is > 2 s old — prevents stale accuracy when NMEA continues but UBX stops

Publishes on every new primary GGA (event-driven), up to `publish_rate` Hz (default 25).

## bringup — launch conventions

- `rover1.launch.py` / `rover2.launch.py`: hardcode namespace, load per-rover YAML
- `sim_harness.launch.py`: navigator + sim_harness only (no hardware nodes)
- YAML keys MUST be fully-qualified (`/rvN/<node>`) or ROS2 silently ignores
- `camera_source` launch arg selects video mode (csi / usb / test)

## colcon tips

```bash
colcon build --symlink-install --packages-select agri_rover_navigator
ros2 node list
ros2 topic echo /rv1/rc_input
ros2 param list /rv1/navigator
ros2 param get /rv1/navigator lookahead_distance
```
