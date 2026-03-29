# AgriRover ROS2 System — CLAUDE.md

Quick-reference for every session. Read before touching any file.

## Project purpose

Dual autonomous ground rover platform for agriculture.
- **RV1 (master):** Jetson Orin Nano, HM30 air unit, RP2040 with SBUS→SX1278 TX
- **RV2 (slave):** Jetson Orin Nano, RP2040 with SX1278 RX→PPM
- **GQC:** SIYI MK32 Android GCS, MAVLink over WiFi
- **Prior codebase:** `C:\agri_rover` (standalone Python, nRF24) — reference only, do not modify

---

## Repo layout

```
ros_agri_rover/
├── ros2_ws/src/
│   ├── agri_rover_interfaces/   ← custom msgs/srvs/actions (ament_cmake)
│   ├── agri_rover_rp2040/       ← USB serial bridge node
│   ├── agri_rover_gps/          ← dual NMEA GPS driver node
│   ├── agri_rover_mavlink/      ← MAVLink ↔ ROS2 bridge node
│   ├── agri_rover_navigator/    ← Stanley/MPC/TTR path follower + pivot turns + obstacle reroute
│   ├── agri_rover_sensors/      ← agricultural sensor node (stub)
│   ├── agri_rover_video/        ← GStreamer RTSP streamer node
│   ├── agri_rover_simulator/    ← dead-reckoning GPS simulator (separate Jetson)
│   └── agri_rover_bringup/      ← launch files + per-rover YAML configs
├── firmware/rc_link_sx1278/
│   ├── master/                  ← RP2040: SBUS→PIO, PPM→PIO+DMA, SX1278 TX
│   └── slave/                   ← RP2040: SX1278 RX, PPM→PIO+DMA, Jetson serial
├── android/AgriRoverGQC/        ← Android GQC app (Kotlin + Google Maps)
└── tools/
    ├── simulator.py             ← standalone GPS simulator (no ROS2)
    ├── nmea_wifi_rx.py          ← UDP NMEA → PTY virtual serial ports
    ├── rtk_forwarder.py         ← NTRIP/E610 RTCM3 → u-blox serial
    ├── start_rover1_sim.sh / start_rover2_sim.sh ← single-command sim launchers
    ├── sim_navigator.py         ← SIL: Stanley+MPC+TTR+pivot+obstacle; reads rover1_params.yaml; --algo flag overrides; TTR uses DiffDriveState (track-width kinematics + SmoothSpeed + angular limit from Robot.cpp); diagnostics → tools/obstacle_debug.log
    ├── mission_planner.py       ← web mission editor + SIL (HTTP :8089); Gen button: Grid/Zigzag/Scatter/Spiral
    ├── monitor.py               ← terminal dashboard + Leaflet map (HTTP :8088); auto-fit, localStorage zoom, 5s refresh
    ├── mission_uploader.py      ← CSV waypoints → MAVLink mission upload
    └── dynamics_collector.py    ← manual-drive data logger for model tuning (PPM + GPS → CSV)
```

---

## Communication layers

| Layer   | Protocol           | Path                                           | Rate   | Purpose               |
|---------|--------------------|------------------------------------------------|--------|-----------------------|
| RC      | SBUS → SX1278 LoRa | MK32→HM30→RP2040(master)→SX1278→RP2040(slave) | 25 Hz  | Manual control/safety |
| MAVLink | UDP / WiFi         | Android GQC ↔ Each Jetson                      | 10 Hz  | Telemetry/missions    |
| Video   | RTSP / WiFi        | Each Jetson → GQC                              | 30 fps | Camera feeds          |
| ROS2    | DDS / WiFi         | Jetson ↔ Jetson (same domain)                  | varies | Autonomy              |

---

## ROS2 packages

| Package               | Type         | Node            | Key responsibility                                           |
|-----------------------|--------------|-----------------|--------------------------------------------------------------|
| agri_rover_interfaces | ament_cmake  | —               | RCInput, SensorData, MissionWaypoint msgs; SetMode, ArmDisarm srvs; FollowMission action |
| agri_rover_rp2040     | ament_python | rp2040_bridge   | USB serial ↔ ROS2: reads CH: lines, sends \<HB:\> \<J:\>   |
| agri_rover_gps        | ament_python | gps_driver      | Dual NMEA serial → NavSatFix + heading Float32               |
| agri_rover_mavlink    | ament_python | mavlink_bridge  | ROS2 topics ↔ MAVLink UDP to GQC                            |
| agri_rover_navigator  | ament_python | navigator       | Stanley/MPC/TTR + pivot turns + obstacle pre-reroute; publishes XTE |
| agri_rover_sensors    | ament_python | sensor_node     | Tank/temp/humidity/pressure (stub)                          |
| agri_rover_video      | ament_python | video_streamer  | GStreamer RTSP server                                        |
| agri_rover_simulator  | ament_python | simulator       | Dead-reckoning GPS simulator                                |
| agri_rover_bringup    | ament_cmake  | —               | rover1.launch.py, rover2.launch.py                          |

**ROS2 namespaces:** `/rv1/` master, `/rv2/` slave. Same `ROS_DOMAIN_ID`.

---

## ROS2 topic map (per rover, prefix /rvN/)

| Topic        | Type                                  | Producer                    | Consumers                 |
|--------------|---------------------------------------|-----------------------------|---------------------------|
| rc_input     | agri_rover_interfaces/RCInput         | rp2040_bridge               | mavlink_bridge            |
| mode         | std_msgs/String                       | rp2040_bridge, mavlink_bridge | navigator, mavlink_bridge |
| cmd_override | agri_rover_interfaces/RCInput         | navigator                   | rp2040_bridge             |
| servo_state  | agri_rover_interfaces/RCInput         | mavlink_bridge              | navigator                 |
| armed        | std_msgs/Bool                         | mavlink_bridge              | navigator                 |
| wp_active    | std_msgs/Int32                        | navigator                   | mavlink_bridge            |
| xte          | std_msgs/Float32                      | navigator                   | mavlink_bridge            |
| fix          | sensor_msgs/NavSatFix                 | gps_driver                  | navigator, mavlink_bridge |
| fix_front    | sensor_msgs/NavSatFix                 | gps_driver                  | navigator                 |
| heading      | std_msgs/Float32                      | gps_driver                  | navigator, mavlink_bridge |
| rtk_status   | std_msgs/String                       | gps_driver                  | mavlink_bridge            |
| sensors      | agri_rover_interfaces/SensorData      | sensor_node                 | mavlink_bridge            |
| mission      | agri_rover_interfaces/MissionWaypoint | mavlink_bridge              | navigator                 |
| mission_fence| std_msgs/String (JSON)                | mavlink_bridge              | navigator                 |

**Cross-namespace topics (inter-rover proximity, absolute paths — no /rvN/ prefix):**

| Topic       | Type              | Direction                 | Purpose                                  |
|-------------|-------------------|---------------------------|------------------------------------------|
| /rv1/fix    | NavSatFix         | rv2/navigator subscribes  | RV2 monitors RV1 position for proximity  |
| /rv2/fix    | NavSatFix         | rv1/navigator subscribes  | RV1 monitors RV2 position for proximity  |
| /rv1/heading| Float32           | rv2/navigator subscribes  | RV2 needs RV1 orientation for bbox       |
| /rv2/heading| Float32           | rv1/navigator subscribes  | RV1 needs RV2 orientation for bbox       |
| /rv1/mode   | String            | rv2/navigator publishes   | RV2 sends MANUAL to stop RV1 on e-stop   |
| /rv2/mode   | String            | rv1/navigator publishes   | RV1 sends MANUAL to stop RV2 on e-stop   |

---

## MAVLink & network constants

| Item            | Value                     | Notes                                              |
|-----------------|---------------------------|----------------------------------------------------|
| RV1 sysid       | 1                         | bind port :14550                                   |
| RV2 sysid       | 2                         | bind port :14550 (different Jetson, no conflict)   |
| GQC sysid       | 255                       | sends to WiFi broadcast (subnet .255)              |
| RV1 video       | rtsp://rv1-ip:8554/stream |                                                    |
| RV2 video       | rtsp://rv2-ip:8555/stream |                                                    |
| MAVLink version | v2 (MAVLINK20=1)          | set in all Python node env                         |

GQC identifies rovers by **sysid in HEARTBEAT**, not by port. Both rovers bind :14550 — **do not change RV2 to 14551**.

`mavlink_bridge` uses a dedicated outbound `socket.SOCK_DGRAM` (`self._udp_sock`) created in `__init__`. Never use `xxx_send()` convenience methods on a `udpin:` connection — they call `mav.file.write()` which fails silently. Use `xxx_encode()` + `_send()` instead.

`gqc_host` must match the **broadcast address** of the rover's WiFi subnet (e.g. `192.168.100.255`).

**Broadcast vs unicast:** `_send()` always uses `self._gqc_addr` (broadcast) so passive tools receive telemetry. If `self._gqc_unicast` is known, `_send()` also sends a second copy to bypass WiFi AP DTIM buffering. `_gqc_unicast` is updated only from **sysid 255** packets; target port forced to `gqc_port` (14550).

**Mission upload — streaming protocol:**
- GQC sends MISSION_COUNT, waits 150 ms, then pushes all items with 20 ms inter-packet gap (no per-item REQUEST_INT wait).
- Rover `_on_mission_item` accepts items in order; does **not** send REQUEST_INT per receipt.
- `_mission_retry` fires every 0.5 s; sends REQUEST_INT only if item not arrived within **250 ms** (packet-loss fallback).
- First REQUEST_INT (seq=0) delayed 150 ms after MISSION_COUNT to avoid race condition.
- Root causes fixed: Jetson WiFi power-save + per-item DTIM round-trip. Startup scripts run `iw dev <iface> set power_save off`.

---

## Firmware — RP2040 pin assignments

| Signal      | GPIO | Pico Pin | Dir | Notes                                        |
|-------------|------|----------|-----|----------------------------------------------|
| SBUS RX     | GP4  | 6        | IN  | Inverted 100kbaud 8E2 from HM30 (master only)|
| PPM OUT     | GP15 | 20       | OUT | To motor controllers / ESC (both)            |
| SX1278 MISO | GP16 | 21       | IN  | SPI0 RX                                      |
| SX1278 NSS  | GP17 | 22       | OUT | SPI0 chip select (active LOW)                |
| SX1278 SCK  | GP18 | 24       | OUT | SPI0 SCK                                     |
| SX1278 MOSI | GP19 | 25       | OUT | SPI0 TX                                      |
| SX1278 RST  | GP20 | 26       | OUT | Hardware reset (active LOW pulse)            |
| SX1278 DIO0 | GP21 | 27       | IN  | TxDone (master) / RxDone (slave)             |

SX1278 runs at **3.3 V** — do not connect to 5 V. No other DIOx pins (DIO1–DIO5) used.
**Master:** `RegDioMapping1 = 0x40` → DIO0 = TxDone. **Slave:** `RegDioMapping1 = 0x00` → DIO0 = RxDone.

---

## Firmware — PIO design

**Master:** `PIO0 SM0` — `sbus_rx.pio`: inverted UART RX, 8× oversampling, clkdiv=156.25. CPU reads: `byte = (pio_sm_get(pio0,0) >> 24) ^ 0xFF`.
`PIO0 SM1` — `ppm_tx.pio`: 1 µs/cycle (clkdiv=125), DMA feeds 20-word buffer, DMA IRQ0 → `ppm_buf_update()` at 20 ms. `LOW_CHn = channel_µs - 302`, `LOW_SYNC = 20000 - Σchannels - 302`.

**Slave:** `PIO0 SM0` — same `ppm_tx.pio`. No SBUS PIO — SX1278 feeds `rf_ch[]` directly.

**SX1278 LoRa config:** 433 MHz (0x6C8000), SF7, BW500kHz, CR4/5, explicit header, CRC on, +17dBm.
**TX:** 25 Hz (every 2nd PPM frame) — LoRa ToA ≈ 12.9 ms, 40 ms period = 32% duty.
**Payload:** 16 × uint16_t = 32 bytes, little-endian, **raw SBUS channels** (`sbus_ch[]`, never `out_ch[]`). Slave calls `apply_ppm_map()` once on received `rf_ch[]`. Sending `out_ch` would double-map and send neutral in RELAY mode.

---

## Mode logic (both RP2040s)

```
── Master (RV1) ──────────────────────────────────────────────────────────────
SBUS lost/timeout (200ms)          → EMERGENCY
CH4 (SWA) < 1700                   → EMERGENCY  ← only trigger besides signal loss
CH9 > 1750                         → RELAY  (master neutral, slave selected)
CH9 in [1250, 1750]:
  CH5 (SWB) > 1700:
    + no Jetson heartbeat          → AUTO-NO-HB  (neutral)
    + heartbeat but no cmd         → AUTO-TIMEOUT (neutral)
    + heartbeat + fresh cmd        → AUTONOMOUS  (Jetson controls master)
  CH5 <= 1700                      → RELAY  (master neutral, raw SBUS forwarded)
CH9 < 1250                         → MANUAL (master moves, raw SBUS passthrough)

── Slave (RV2) ───────────────────────────────────────────────────────────────
RF lost/timeout (500ms)            → EMERGENCY
CH4 (from RF) < 1700               → EMERGENCY
CH9 < 1250                         → IDLE  (master selected, not alarm)
CH9 in [1250, 1750]:
  CH5 > 1700 + HB alive + fresh cmd → AUTONOMOUS
  else                               → IDLE
CH9 > 1750                         → MODE_RF (slave moves, relay values)
```

CH9 switch: Low(<1250)=RV1 manual, Mid(1250–1750)=both neutral/AUTO valid, High(>1750)=RV2 manual.

---

## RP2040 ↔ Jetson serial protocol

```
RP2040 → Jetson:
  CH:1500,...,1500 MODE:MANUAL           (every frame, 16 channels)
  [SBUS_OK] / [SBUS_LOST]               (on change)
  [RF_LINK_OK] / [RF_LINK_LOST]         (slave, on change)
  <HB:N+1>                              (heartbeat echo)

Jetson → RP2040:
  <HB:N>                                (must arrive < 300 ms for AUTO to stay active)
  <J:c0,c1,c2,c3,c4,c5,c6,c7>          (8-channel autonomous command, µs)
```

### Servo command flow (DO_SET_SERVO)

`MAV_CMD_DO_SET_SERVO` (cmd=183): `param1` = servo 5–8, `param2` = PWM µs.

```
GQC → mavlink_bridge._apply_servo_cmd() → servo_state topic
  → navigator._servo_ch[0..3] → _publish_cmd() at 10 Hz
  → cmd_override topic → rp2040_bridge → <J:c0,...,c7>
```

Navigator re-publishes servo state with every cmd_override tick so values survive RP2040 resets.

| index | PPM ch | Servo |
|-------|--------|-------|
| 4     | CH5    | 5     |
| 5     | CH6    | 6     |
| 6     | CH7    | 7     |
| 7     | CH8    | 8     |

`mavlink_bridge._apply_servo_cmd()` called from both `_on_command_long` and `_on_mission_item`.

---

## Build commands

```bash
# Docker (Jetson)
docker build -t agri_rover:latest .
bash tools/start_rover1_sim.sh       # simulation (handles PTY, container, GPS)
bash tools/start_rover2_sim.sh
docker compose up rover1             # production
CAMERA_SOURCE=usb docker compose up rover1

# Inside container
docker exec -it agri_rover_rv1 bash
source /opt/ros/*/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash

# Firmware (dev machine)
cd firmware/rc_link_sx1278/master
cmake -B build -DPICO_SDK_PATH=$PICO_SDK_PATH && cmake --build build

# Tools
python tools/monitor.py                  # map → http://<host>:8088/monitor_map.html
python tools/monitor.py --map-port 9000
python tools/mission_uploader.py missions/field.csv --rover 1 --host 192.168.100.19

# Simulator Jetson (after rover containers are up)
python3 tools/simulator.py --rv1-ip 192.168.100.19 --rv2-ip 192.168.100.20 --ppm-port /dev/ttyACM0

# RTK corrections (real hardware)
python3 tools/rtk_forwarder.py --source ntrip --ntrip-host <host> --ntrip-port 2101 \
  --mountpoint <mount> --ntrip-user <u> --ntrip-pass <p> --gps-ports /dev/ttyUSB0 /dev/ttyUSB1
python3 tools/rtk_forwarder.py --source e610 --e610-host 192.168.1.20 --e610-port 9000 \
  --gps-ports /dev/ttyUSB0 /dev/ttyUSB1
```

---

## Config files

| File                                                         | What to edit                              |
|--------------------------------------------------------------|-------------------------------------------|
| `ros2_ws/src/agri_rover_bringup/config/rover1_params.yaml`  | Ports, IPs, speeds for RV1               |
| `ros2_ws/src/agri_rover_bringup/config/rover2_params.yaml`  | Ports, IPs, speeds for RV2               |
| `ros2_ws/src/agri_rover_navigator/config/navigator_params.yaml` | Lookahead, speed limits, `obstacle_clearance_m`, `rover_width_m` |
| `ros2_ws/src/agri_rover_video/config/gstreamer.yaml`        | Camera source, resolution, bitrate       |

**YAML key format — critical:** Keys must include the fully-qualified namespace or ROS2 silently ignores them:
```yaml
# WRONG
gps_driver:
  ros__parameters:
    primary_port: /dev/ttyUSB0
# CORRECT
/rv1/gps_driver:
  ros__parameters:
    primary_port: /dev/ttyUSB0
```
Config dir is volume-mounted — edits take effect on container restart, no rebuild needed.

---

## Conventions

- **Python nodes:** rclpy, class inherits `Node`, parameters declared in `__init__`, timers not threads.
- **MAVLink:** always `os.environ['MAVLINK20'] = '1'` before importing pymavlink. Use `xxx_encode()` + `_send()` — never `xxx_send()`. `_send()` uses `self._udp_sock` (broadcast SOCK_DGRAM in `__init__`) — do not access pymavlink's internal socket attributes.
- **GPS port retry:** `gps_driver._start_reader()` retries every 5 s intentionally — `nmea_wifi_rx.py` starts after `gps_driver` in simulation.
- **PTY and Docker devpts:** PTYs created on host are NOT visible inside container. Always run `nmea_wifi_rx.py` via `docker exec -d` inside the container; PTY symlinks appear at `/tmp/rv1_gps_{pri,sec}` (shared via `/tmp:/tmp` mount).
- **PPM buffer:** updated only in `dma_irq_handler()`. Main loop writes `out_ch[]` (volatile), IRQ reads it. No mutex needed for uint16_t aligned writes on M0+.
- **SX1278 TX:** fire-and-forget. Check `RegIrqFlags.TxDone` before next TX; returns `false` if busy.
- **LoRa payload = raw SBUS only:** master transmits `sbus_ch[]`, never `out_ch[]`. Slave calls `apply_ppm_map()` exactly once.
- **SBUS decode:** validate `frame[0]==0x0F && frame[24]==0x00`. Re-sync on every `0x0F` byte.

---

## Isaac ROS — architecture

ROS2 workspace runs inside NVIDIA Isaac ROS (Jazzy, aarch64) on each Jetson.
Docker base: `nvcr.io/nvidia/isaac/ros:isaac_ros_740c8500df2685ab1f4a4e53852601df-arm64-jetpack` (Ubuntu 24.04).

**Video pipeline (CSI):**
```
[component_container_mt /rvN/video_container]
  ArgusMonoNode → camera/image_raw (NITROS)
  EncoderNode   → image_compressed (NITROS H264)
[VideoStreamerNode] subscribes image_compressed → GstRtspServer appsrc → RTSP
```
`ArgusMonoNode` + `EncoderNode` must share `component_container_mt` for zero-copy NITROS. **Fallback (USB):** `video_streamer._start_fallback_rtsp()` uses GstRtspServer Python GI — no `gst-rtsp-server` binary in container.

**Docker:** `Dockerfile` uses dynamic ROS distro detection; pip installs use `--break-system-packages`. Python node files + config YAML are volume-mounted — live-edit without rebuild.

---

## Android GQC

`android/AgriRoverGQC/` — Kotlin, Google Maps SDK, java-mavlink (io.dronefleet), XML layouts (no Compose).

**Key classes:** `MainActivity` (UI/map/modes), `RoverPositionManager` (all MAVLink UDP I/O).

**RoverPositionManager API:**
- `sendCommand(sysId, cmd, p1, p2)` — single COMMAND_LONG
- `sendCriticalCommand(sysId, cmd, p1, p2)` — **3× at 100 ms** — use for E-STOP/disarm
- `uploadMission / uploadRecordedMission` — plain waypoints or mixed waypoint+servo items
- `sysId=0` → broadcast 255.255.255.255:14550

Rover IPs auto-discovered from first incoming packet per sysid. Callbacks dispatched on Main thread.

**NAMED_VALUE_FLOAT telemetry (mavlink_bridge → GQC):**
- `SBUS_OK` / `RF_OK` @ 10 Hz → link dots (green/red)
- `RTK` @ 1 Hz → GPS fix type (6=RTK_FIX, 5=RTK_FLT, 4=DGPS, 3=3D, 0=NO_GPS)
- `STATUS` @ 1 Hz → 0=NA, 1=MSL, 2=ARM
- `WP_ACT` @ 1 Hz → current waypoint index

Note: `GpsRawInt` is sent but java-mavlink 1.1.9 rejects it (pymavlink truncates zero extension fields → 30-byte payload). Use `NAMED_VALUE_FLOAT 'RTK'` instead.

**Modes:** App launches in PLANNER. MANUAL is rover-reported only — no RC override sent.

| Mode    | UI                                              | Behaviour                              |
|---------|-------------------------------------------------|----------------------------------------|
| PLANNER | ⋮ menu: ADD/CLEAR/UPLOAD; REC FAB; OBS FAB     | Record/upload mission. Default.        |
| AUTO    | R1/R2 toggle, ▶ START, ⏸ PAUSE, CLEAR         | ARM + AUTO; mission progress on map    |

**Mission recording:**
- **ADD** (overflow menu) — inserts selected rover's current GPS position.
- **REC** — samples GPS @ 500 ms; appends ServoCmd on any PPM CH5–CH8 change > 100 µs.
- **OBS** — tap vertices to draw obstacle polygon; DONE closes it (min 3 vertices); red semi-transparent overlay.
- **UPLOAD** — sends DISARM × 3 before MISSION_COUNT. Dispatches: `uploadMissionWithObstacles()` if OBS polygons exist, `uploadRecordedMission()` if ServoCmd items, else `uploadMission()`. Fence vertices sent as cmd=5003.

```kotlin
sealed class MissionAction {
    data class Waypoint(val lat: Double, val lon: Double, val speed: Float) : MissionAction()
    data class ServoCmd(val servo: Int, val pwm: Int) : MissionAction()
}
```

Waypoint speed = `dist_m / 0.5s`, clamped [0.3, 1.5] m/s; sent as `z` field of NAV_WAYPOINT; first WP uses 0 (navigator default).

**Safety:**
- E-STOP → `sendCriticalCommand` DISARM (cmd 400, p1=0) to all rovers.
- `checkLinkMismatch()` → auto-disarm after 2 s RC switch ≠ slave HEARTBEAT mismatch.
- **ARM gate (START):** mission must be loaded + STATUS ≠ NA + rover within **0.5 m** of first waypoint + CH9 must select a rover (not mid-position).

**Mission auto-disarm:** `wp_active = -1` → mavlink_bridge auto-disarms, clears mission, publishes MANUAL.

**Resource management state machine** (`mavlink_bridge._check_resource_levels`, 1 Hz):
```
normal → going_to_base → at_base → normal
```
- `normal` + armed + AUTONOMOUS + `wp_active >= 0` + (battery < threshold OR tank < threshold) → `going_to_base`. Saves mission state, uploads 2-WP base trip (rover pos → station) via `_internal_upload_mission`. Rover stays armed+autonomous — navigator seamlessly switches to base trip.
- Battery/tank default 0.0 treated as unknown (skip check) — prevents false-trigger before sensors publish.
- `going_to_base` + `wp_active == -1` → `at_base`: disarm + MANUAL, then immediately `_upload_resume_mission()` which builds the resume route (base station → stopped position → remaining WPs) and broadcasts to GQC via `_internal_upload_mission`. This ensures GQC's proximity check passes (WP0 = base = rover's current position).
- `at_base` + ARM → `_resource_state = 'normal'`. Resume mission is already loaded — navigator starts when SET_MODE AUTO arrives. GQC START sends ARM + 500 ms + SET_MODE.
- `_internal_upload_mission` does NOT publish `mission_clear` — seq=0 resets navigator path. DDS inter-topic ordering is not guaranteed; a separate clear races with wp[0].

**Planned path overlay:** navigator publishes rerouted path via TUNNEL → GQC renders as green dotted line. Bypass segments in rover color (orange RV1, cyan RV2). Fires for all missions (mavlink_bridge always publishes `mission_fence` on ACK).

**Map:** default view Jalisco field (20.727715, -103.566782, zoom 18). Satellite default. Markers: red=RV1, blue=RV2; centre dot green=disarmed, orange=armed, yellow=AUTO. Route: green=walked, red=pending. Waypoint dots erased as rover reaches each WP (driven by WP_ACT).

---

## Simulator — dead-reckoning GPS

Runs on a **third Jetson Orin Nano**. Reads `/dev/ttyACM0` (ppm_decoder RP2040 at 50 Hz). Sends `$GNGGA` + `$GNVTG` @ 10 Hz over UDP to rover Jetsons.

Secondary GGA offset `antenna_baseline_m` (default 1.00 m) ahead → `gps_driver` recovers heading via `atan2(dlon × cos(lat), dlat)`. The `cos(lat)` correction is required — omitting it causes heading drift.

**Physics (skid/differential steer):**
```python
speed = (throttle_ppm - 1500) / 500 * max_speed_mps
steer = (steering_ppm - 1500) / 500
omega = turn_scale * 2 * steer * max_speed_mps / wheelbase_m  # rad/s
heading += omega * dt
lat += speed * cos(heading) * dt / 111320
lon += speed * sin(heading) * dt / (111320 * cos(lat_rad))
```
`turn_scale` default **1.0** (full differential, matches MPC model). Pass `--turn-scale` or set in `simulator_params.yaml`.

---

## NTRIP pitfalls (rtk_forwarder.py)

- **Use HTTP/1.0** — NTRIP v1 casters respond `ICY 200 OK\r\n` (single CRLF); HTTP/1.1 confuses them.
- **Do not send GGA unconditionally** — single-base casters stream immediately after HTTP request; sending GGA stops them. Only send if `--approx-lat/--approx-lon` given (VRS/network-RTK only).
- **Emlid:** responds `ICY 200 OK\r\n`, starts RTCM3 immediately, no GGA needed.
- **Disconnect reasons logged:** `no data for 30 s` = base offline; `stream closed by caster (EOF)` = caster dropped.

---

## Known TODOs

- `sensor_node.py`: stub — wire real I2C sensors (BME280, ultrasonic tank)
- `firmware/*/main.cpp`: needs `pico_sdk_import.cmake` from `$PICO_SDK_PATH/external/`
- Android GQC: RTSP dual video, settings dialog, STATUSTEXT log screen (see android/AgriRoverGQC/README.md)
- Navigator obstacle avoidance: single-pass per segment — segments intersecting multiple non-adjacent polygons only reroute around the first one.
- DO_SET_SERVO in missions: applied at upload time, re-published at 25 Hz. Precise per-waypoint timing would need MissionWaypoint interface change or new ServoEvent topic.
