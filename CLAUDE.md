# Tecneterra ROS2 Rover — CLAUDE.md

Quick-reference for every session. Read before touching any file.

## Project purpose

Dual autonomous ground rover for agriculture. Corridor-only navigation,
Stanley lateral control, no on-axis turns.

- **RV1 (master):** Jetson Orin Nano, HM30 air unit, RP2040 SBUS→SX1278 TX
- **RV2 (slave):** Jetson Orin Nano, RP2040 SX1278 RX→PPM
- **GQC:** SIYI MK32 Android GCS (Kotlin, Google Maps, rosbridge primary transport)
- **Upstream fork:** `C:\ros_agri_rover` — the working production tree. This `tecneterra_ros_rover` fork is a rewrite for final security audit.

---

## Repo layout

```
ros2_ws/src/
  agri_rover_interfaces/   ← custom msgs/srvs/actions (ament_cmake)
  agri_rover_rp2040/       ← USB serial bridge node
  agri_rover_gps/          ← dual NMEA GPS driver
  agri_rover_navigator/    ← Stanley path follower on corridor polylines
  agri_rover_sensors/      ← agricultural sensor node (stub)
  agri_rover_video/        ← GStreamer RTSP streamer
  agri_rover_simulator/    ← dead-reckoning GPS sim + on-Jetson SIL harness
  agri_rover_bringup/      ← launch files + per-rover YAML configs
firmware/rc_link_sx1278/{master,slave}/  ← RP2040 SBUS/SX1278/PPM
android/TecneterraGCS/                   ← Android GQC app
tools/
  simulator.py          ← standalone GPS sim (3rd Jetson, no ROS2)
  nmea_wifi_rx.py       ← UDP NMEA → PTY virtual serial ports
  rtk_forwarder.py      ← NTRIP RTCM3 → u-blox serial (legacy; RF path replaces it)
  sim_navigator.py      ← SIL: Stanley corridor navigator
  mission_planner.py    ← web editor+SIL (HTTP :8089)
  monitor.py            ← terminal dashboard + Leaflet map (HTTP :8088)
  dynamics_collector.py ← manual-drive data logger (PPM+GPS→CSV)
  start_rover{1,2}_sim.sh / start_sim_harness.sh ← sim launchers
```

---

## Communication layers

| Layer     | Protocol         | Rate   | Notes |
|-----------|------------------|--------|-------|
| RC        | SBUS→SX1278 LoRa | 25 Hz  | MK32→HM30→RP2040(master)→SX1278→RP2040(slave) |
| rosbridge | WebSocket :9090  | varies | **Primary transport** — missions, telemetry, commands |
| Video     | RTSP             | 30 fps | rv1 :8554, rv2 :8555 |
| ROS2      | DDS/WiFi         | varies | Jetson↔Jetson same domain |

MAVLink is no longer used by this fork. The legacy `agri_rover_mavlink`
package, `mission_uploader.py`, and UDP :14550 listeners have all been
deleted. GQC commands arrive via rosbridge.

---

## ROS2 packages & topics

**Namespaces:** `/rv1/` master, `/rv2/` slave. Same `ROS_DOMAIN_ID`.

| Node | Key topics (under /rvN/) |
|------|--------------------------|
| rp2040_bridge | publishes `rc_input` (RCInput), `mode` (String); subscribes `cmd_override` |
| gps_driver | publishes `fix`, `fix_front` (NavSatFix), `heading` (Float32), `rtk_status`, `hacc` (Float32) |
| navigator | publishes `cmd_override`, `wp_active` (Int32), `xte` (Float32), `rerouted_path`, `path_version`, `nav_status`, `nav_params`; subscribes `fix`, `fix_front`, `heading`, `mode`, `armed`, `corridor_mission`, `mission_clear`, `hacc`, `sensors`, `status`, `servo_state` |
| sensor_node | publishes `sensors` (SensorData) — stub |
| video_streamer | GStreamer RTSP server |
| sim_harness | SIL: DiffDriveState physics → fix/fix_front/heading feedback loop |

**Cross-namespace (inter-rover proximity):** `/rv1/fix`, `/rv2/fix`, `/rv1/heading`, `/rv2/heading` (each navigator subscribes to the other rover). `/rv1/mode`, `/rv2/mode` (cross-publish MANUAL for e-stop).

**Special wp_active values:** -1 = mission complete (auto-disarm).

---

## Firmware (RP2040)

**Pins:** SBUS RX=GP4, PPM OUT=GP15, SX1278 SPI0: MISO=GP16, NSS=GP17, SCK=GP18, MOSI=GP19, RST=GP20, DIO0=GP21. **3.3 V only.**

**PIO:** Master SM0=sbus_rx (inverted UART 8x oversample), SM1=ppm_tx (DMA 20-word buffer, 20 ms cycle). Slave SM0=ppm_tx only.

**SX1278 LoRa:** 433 MHz, SF7, BW500kHz, CR4/5, CRC on, +17dBm. TX 25 Hz. **Payload = raw `sbus_ch[]` only** (never `out_ch[]` — would double-map). Slave calls `apply_ppm_map()` once.

**Mode logic:**
```
Master: SBUS lost/CH4<1700 → EMERGENCY
  CH9>1750 → RELAY | CH9 mid + CH5<1700 + HB+cmd → AUTONOMOUS | CH9<1250 → MANUAL
Slave:  RF lost/CH4<1700 → EMERGENCY
  CH9<1250 → IDLE | CH9 mid + CH5<1700 + HB+cmd → AUTONOMOUS | CH9>1750 → MODE_RF
```
CH9: Low=RV1 manual, Mid=AUTO valid, High=RV2 manual.

**Serial protocol:**
```
RP2040→Jetson: CH:1500,...,1500 MODE:MANUAL  |  [SBUS_OK]/[SBUS_LOST]  |  <HB:N+1>
Jetson→RP2040: <HB:N> (< 300 ms)  |  <J:c0,c1,...,c7> (8-ch µs)
```

**Servo mapping:** cmd_override indices 4–7 → PPM CH5–CH8 → servo 5–8. Navigator re-publishes servo state every tick.

---

## Build & config

```bash
# Docker (Jetson)
docker build -t tecneterra_rover:latest .
bash tools/start_rover1_sim.sh          # simulation
docker compose up rover1                # production
# Firmware
cd firmware/rc_link_sx1278/master && cmake -B build -DPICO_SDK_PATH=$PICO_SDK_PATH && cmake --build build
# Tools
python tools/monitor.py                 # map → http://<host>:8088/monitor_map.html
python tools/mission_planner.py         # editor → http://<host>:8089
```

**Config files:** `agri_rover_bringup/config/rover{1,2,3}_params.yaml` (ports, IPs, speeds), `agri_rover_navigator/config/navigator_params.yaml` (lookahead, speed, stiction floors), `agri_rover_video/config/gstreamer.yaml`.

**YAML key format — critical:** Must include fully-qualified namespace or ROS2 silently ignores:
```yaml
# WRONG: gps_driver:        CORRECT: /rv1/gps_driver:
```
Config is volume-mounted — edits take effect on restart, no rebuild.

---

## Conventions

- **Python nodes:** rclpy, class inherits `Node`, parameters in `__init__`, timers not threads
- **GPS port retry:** `gps_driver._start_reader()` retries every 5 s — `nmea_wifi_rx.py` starts after gps_driver in sim
- **PTY + Docker:** PTYs on host are NOT visible in container. Run `nmea_wifi_rx.py` via `docker exec -d`; symlinks at `/tmp/rv1_gps_{pri,sec}`
- **PPM buffer:** `out_ch[]` (volatile) written by main loop, read by DMA IRQ. No mutex needed (uint16_t aligned on M0+)
- **SBUS decode:** validate `frame[0]==0x0F && frame[24]==0x00`, re-sync on every `0x0F`

---

## Isaac ROS / Docker

ROS2 Jazzy (aarch64) on each Jetson. Docker base: NVIDIA Isaac ROS (Ubuntu 24.04). Dockerfile uses dynamic distro detection; pip uses `--break-system-packages`. Python nodes + config YAML volume-mounted — live-edit without rebuild.

**Video (CSI):** ArgusMonoNode + EncoderNode share `component_container_mt` for zero-copy NITROS → H264 → RTSP via GstRtspServer. **USB fallback:** `_start_fallback_rtsp()` uses Python GI.

---

## Android GQC

`android/TecneterraGCS/` — Kotlin, Google Maps SDK, XML layouts (no Compose).

**Key classes:** `MainActivity` (UI/map/modes), `RoverPositionManager` (rosbridge I/O).

**Telemetry (NAMED_VALUE_FLOAT):** `SBUS_OK`/`RF_OK` @10Hz, `RTK` @1Hz (6=FIX,5=FLT,4=DGPS,3=3D,0=NONE), `STATUS` @1Hz (0=NA,1=MSL,2=ARM), `WP_ACT` @1Hz+event, `MSN_ID` @1Hz (CRC24 hash for mission sync).

**Modes:** PLANNER (default, record corridors) | AUTO (ARM+START, mission progress).

**Mission recording:** REC records corridor centerline (GPS samples @500 ms). ADD button + turn-point / lane-tag UI were removed in the corridor-only rewrite.

**Safety:** E-STOP triple-disarm via rosbridge. ARM gate: mission loaded + STATUS!=NA + CH9 selects rover.

**Mission sync (MSN_ID):** Navigator publishes path as JSON via `rerouted_path`; CRC24 hash over that JSON becomes `MSN_ID`. GQC auto-downloads on mismatch.

**Mission auto-disarm:** `wp_active=-1` → auto-disarm + clear + MANUAL.

**Map:** Jalisco field default (20.727715, -103.566782, zoom 18). Red=RV1, blue=RV2.

---

## Corridor navigation

**Corridor mode** = continuous path-following. A mission is one or more
centerline polylines with a half-width, joined by headland arcs.
Stanley lateral controller tracks CTE to the polyline; XTE telemetry
and CTE alarm (`stanley_cte_alarm_m`) enforce the lane.

**Data model** (`tools/corridor.py` + `ros2_ws/.../corridor.py`): `Corridor` (id, centerline, width, speed) → `corridors_to_path()` → `(lat, lon, speed, half_width, is_turn, servo)` polyline. `auto_split_corridors()` detects sharp turn markers in a raw recording to split it into multiple corridors.

**Mission load flow:** `corridor_mission` topic (JSON) → `_cb_corridor_mission` parses → `corridors_to_path` → **mission validator** (rejects any arc with radius < `min_turn_radius_m`, default 2.0 m) → `_path` populated → Stanley takes over.

**Control loop** (`_control_loop_corridor`):
- Stanley CTE on the polyline, lookahead clamped to the next turn point
- Decel ramp as the rover approaches each turn (linear over `2 × lookahead`)
- When `s_nearest` passes a turn index, advance `_path_idx` to the post-turn point — no on-axis spin
- Align-spin only for large initial heading errors (`align_threshold`); once within deadband, Stanley takes over forever

**Headland turns are arc-only.** The rover physically cannot execute corners tighter than the configured radius — the validator rejects the mission on load.

---

## Simulator

**Standalone (3rd Jetson):** Reads PPM via `/dev/ttyACM0` @50 Hz. Sends GNGGA+GNVTG @10 Hz over UDP. Secondary antenna offset `antenna_baseline_m` (1.0 m) ahead → heading via `atan2(dlon*cos(lat), dlat)`. The `cos(lat)` correction is required.

**On-Jetson SIL (sim_harness):** Runs real navigator inside Docker with `DiffDriveState` physics replacing GPS. `sim_harness.launch.py` starts navigator + sim_harness. Runs via `docker exec` into a running rover container. `diff_drive.py` — pure-Python physics shared by sim_harness and sim_navigator.

```bash
bash tools/start_sim_harness.sh [timeout_seconds]
```

---

## RTCM corrections

Legacy NTRIP via `rtk_forwarder.py` is still usable but the target
deployment pipes RTCM directly over a dedicated SX1278 link (see the
parent tree's `project_rc_links_design.md` memory). Known NTRIP
gotchas if you do use a caster:

- **Use HTTP/1.0** — NTRIP v1 responds `ICY 200 OK\r\n`; HTTP/1.1 breaks it
- **No GGA to single-base** — sending GGA stops stream; only VRS needs GGA

---

## Known TODOs

- `sensor_node.py`: stub — wire real I2C sensors (BME280, ultrasonic tank)
- `firmware/*/main.cpp`: needs `pico_sdk_import.cmake` from `$PICO_SDK_PATH/external/`
- `tools/sim_navigator.py`: still carries waypoint/pivot code — needs corridor-only rewrite
- `tools/mission_planner.py`: waypoint-drawing UI, mission generators, obstacle polygon UI still present — needs strip
- Android GQC: ADD button + REC turn-point detection removal still pending
- Mission validator: currently pre-checks arc radius. Later: live CTE alarm + recovery.
- Resource management (battery/tank → base return): removed; will be rebuilt corridor-native when needed.
