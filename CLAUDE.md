# AgriRover ROS2 System — CLAUDE.md

Quick-reference for every session. Read before touching any file.

## Project purpose

Dual autonomous ground rover for agriculture.
- **RV1 (master):** Jetson Orin Nano, HM30 air unit, RP2040 SBUS→SX1278 TX
- **RV2 (slave):** Jetson Orin Nano, RP2040 SX1278 RX→PPM
- **GQC:** SIYI MK32 Android GCS (Kotlin, Google Maps, rosbridge primary transport)
- **Prior codebase:** `C:\agri_rover` — reference only, do not modify

---

## Repo layout

```
ros2_ws/src/
  agri_rover_interfaces/   ← custom msgs/srvs/actions (ament_cmake)
  agri_rover_rp2040/       ← USB serial bridge node
  agri_rover_gps/          ← dual NMEA GPS driver
  agri_rover_mavlink/      ← MAVLink ↔ ROS2 bridge (deprecated — not launched, sources kept)
  agri_rover_navigator/    ← Stanley path follower + pivot turns + obstacle detection
  agri_rover_sensors/      ← agricultural sensor node (stub)
  agri_rover_video/        ← GStreamer RTSP streamer
  agri_rover_simulator/    ← dead-reckoning GPS sim + on-Jetson SIL harness
  agri_rover_bringup/      ← launch files + per-rover YAML configs
firmware/rc_link_sx1278/{master,slave}/  ← RP2040 SBUS/SX1278/PPM
android/AgriRoverGQC/                   ← Android GQC app
tools/
  simulator.py          ← standalone GPS sim (3rd Jetson, no ROS2)
  nmea_wifi_rx.py       ← UDP NMEA → PTY virtual serial ports
  rtk_forwarder.py      ← NTRIP/E610 RTCM3 → u-blox serial
  sim_navigator.py      ← SIL: Stanley+pivot+obstacle; reads rover1_params.yaml
  mission_planner.py    ← web editor+SIL (HTTP :8089); Gen/Shift/Survey/Loop
  monitor.py            ← terminal dashboard + Leaflet map (HTTP :8088)
  mission_uploader.py   ← CSV waypoints → MAVLink upload
  dynamics_collector.py ← manual-drive data logger (PPM+GPS→CSV)
  start_rover{1,2}_sim.sh / start_sim_harness.sh ← sim launchers
```

---

## Communication layers

| Layer     | Protocol         | Rate   | Notes |
|-----------|------------------|--------|-------|
| RC        | SBUS→SX1278 LoRa | 25 Hz  | MK32→HM30→RP2040(master)→SX1278→RP2040(slave) |
| rosbridge | WebSocket :9090  | varies | **Primary transport** — missions, telemetry, commands, station updates |
| MAVLink   | UDP :14550       | —      | **Deprecated** — nodes not launched; GQC has dead-letter fallback |
| Video     | RTSP             | 30 fps | rv1 :8554, rv2 :8555 |
| ROS2      | DDS/WiFi         | varies | Jetson↔Jetson same domain |

---

## ROS2 packages & topics

**Namespaces:** `/rv1/` master, `/rv2/` slave. Same `ROS_DOMAIN_ID`.

| Node | Key topics (under /rvN/) |
|------|--------------------------|
| rp2040_bridge | publishes `rc_input` (RCInput), `mode` (String); subscribes `cmd_override` |
| gps_driver | publishes `fix`, `fix_front` (NavSatFix), `heading` (Float32), `rtk_status` |
| navigator | publishes `cmd_override`, `wp_active` (Int32), `xte` (Float32); subscribes `fix`, `fix_front`, `heading`, `mode`, `armed`, `mission`, `mission_fence`, `servo_state`, `station_update` |
| mavlink_bridge | publishes `mode`, `armed` (Bool), `mission` (MissionWaypoint), `servo_state`, `mission_fence`, `station_update`; subscribes everything for telemetry relay |
| sensor_node | publishes `sensors` (SensorData) — stub |
| video_streamer | GStreamer RTSP server |
| sim_harness | SIL: DiffDriveState physics → fix/fix_front/heading feedback loop |

**Cross-namespace (inter-rover proximity):** `/rv1/fix`, `/rv2/fix`, `/rv1/heading`, `/rv2/heading` (each navigator subscribes to the other rover). `/rv1/mode`, `/rv2/mode` (cross-publish MANUAL for e-stop).

**Special wp_active values:** -1=mission complete (auto-disarm), -2=going to base, -3=at base, -(seq+1000)=waiting point.

---

## MAVLink & network constants

| Item | Value |
|------|-------|
| RV1/RV2 sysid | 1 / 2 (both bind :14550 — do not change) |
| GQC sysid | 255 (WiFi broadcast) |

**Critical MAVLink rules:**
- Always `os.environ['MAVLINK20'] = '1'` before importing pymavlink
- Use `xxx_encode()` + `_send()` — never `xxx_send()` (fails silently on udpin)
- `_send()` uses dedicated `self._udp_sock` (SOCK_DGRAM) — never pymavlink's internal socket
- `gqc_host` = broadcast address (e.g. `192.168.100.255`); unicast copy sent if `_gqc_unicast` known

**Mission upload (streaming):** GQC pushes MISSION_COUNT + 150 ms + all items @ 20 ms gaps. No per-item ACK. Retry timer (0.5 s) sends REQUEST_INT if packet lost. WiFi power-save disabled at startup (`iw dev <iface> set power_save off`).

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

**Servo mapping:** cmd_override indices 4–7 → PPM CH5–CH8 → servo 5–8. `DO_SET_SERVO` (cmd=183): param1=servo, param2=PWM. Navigator re-publishes servo state every tick.

---

## Build & config

```bash
# Docker (Jetson)
docker build -t agri_rover:latest .
bash tools/start_rover1_sim.sh          # simulation
docker compose up rover1                # production
# Firmware
cd firmware/rc_link_sx1278/master && cmake -B build -DPICO_SDK_PATH=$PICO_SDK_PATH && cmake --build build
# Tools
python tools/monitor.py                 # map → http://<host>:8088/monitor_map.html
python tools/mission_planner.py         # editor → http://<host>:8089
python tools/mission_uploader.py missions/field.csv --rover 1 --host 192.168.100.19
```

**Config files:** `agri_rover_bringup/config/rover{1,2}_params.yaml` (ports, IPs, speeds), `navigator/config/navigator_params.yaml` (lookahead, speed, obstacles), `video/config/gstreamer.yaml`.

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

`android/AgriRoverGQC/` — Kotlin, Google Maps SDK, java-mavlink, XML layouts (no Compose).

**Key classes:** `MainActivity` (UI/map/modes), `RoverPositionManager` (MAVLink UDP + rosbridge I/O).

**API:** `sendCommand(sysId, cmd, p1, p2)` — single shot. `sendCriticalCommand` — 3× at 100 ms (E-STOP). `sysId=0` → broadcast.

**Telemetry (NAMED_VALUE_FLOAT):** `SBUS_OK`/`RF_OK` @10Hz, `RTK` @1Hz (6=FIX,5=FLT,4=DGPS,3=3D,0=NONE), `STATUS` @1Hz (0=NA,1=MSL,2=ARM), `WP_ACT` @1Hz+event, `MSN_ID` @1Hz (CRC24 hash for mission sync).

**Modes:** PLANNER (default, record/upload) | AUTO (ARM+START, mission progress).

**Mission recording:**
- **ADD** — inserts rover's GPS position. **REC** — samples GPS @500ms, records servo changes. **OBS** — tap obstacle polygon vertices.
- **Upload** — DISARM×3 → MISSION_COUNT. Gate: rejects if mission already loaded (must CLEAR first).
- **Wait points** (`holdSecs=-1`): rover disarms on arrival, re-arm to continue.
- **Turn points** (`speed=-1`): throttle near neutral during REC. `_collapse_spin_clusters()` merges consecutive turns. Preserved in CSV save/load.

**MissionAction:** `Waypoint(lat, lon, speed, holdSecs)` | `ServoCmd(servo, pwm)`. Speed sent as NAV_WAYPOINT `z` field.

**Safety:** E-STOP → `sendCriticalCommand` DISARM all. `checkLinkMismatch()` → auto-disarm after 2s. ARM gate: mission loaded + STATUS!=NA + CH9 selects rover. Approach path shown with Confirm/Cancel dialog.

**Approach path:** >1.5m from WP[0] → cubic Bezier (heading-aware, obstacle-checked) → fallback A* grid. Approach WPs (seq=-99) prepended with speed ramp. Navigator disarms after planning for user review.

**Resource management** (`navigator._check_resources`, 1 Hz): `normal→going_to_base→at_base→normal`. Battery/tank < threshold → save mission state → Dijkstra lane path (or confirmed direct path) → arrive at base → build resume mission → disarm (STATUS=MSL). Re-arm to resume. Test injection via `station_update` topic (`test_tank`, `test_batt`) or ROS2 params.

**Mission sync (MSN_ID):** Navigator publishes path as JSON via `rerouted_path`. mavlink_bridge computes CRC24 → `MSN_ID` @1Hz. GQC auto-downloads on mismatch.

**Mission auto-disarm:** `wp_active=-1` → auto-disarm + clear + MANUAL.

**Map:** Jalisco field default (20.727715, -103.566782, zoom 18). Red=RV1, blue=RV2. Centre dot: green=disarmed, orange=armed, yellow=AUTO. Route: green=walked, red=pending.

**Base-return confirmation:** No lane map → navigator publishes `confirm_message` + `reroute_pending`. GQC shows Confirm/Reject dialog via rosbridge.

---

## Simulator

**Standalone (3rd Jetson):** Reads PPM via `/dev/ttyACM0` @50Hz. Sends GNGGA+GNVTG @10Hz over UDP. Secondary antenna offset `antenna_baseline_m` (1.0m) ahead → heading via `atan2(dlon*cos(lat), dlat)`. The `cos(lat)` correction is required.

**On-Jetson SIL (sim_harness):** Runs real navigator inside Docker with `DiffDriveState` physics replacing GPS. `sim_harness.launch.py` starts mavlink_bridge + navigator + sim_harness. Runs via `docker exec` into running rover1 container. `diff_drive.py` — pure-Python physics shared by sim_harness and sim_navigator.

```bash
bash tools/start_sim_harness.sh [timeout_seconds]
```

**mission_planner integration:** "Sim" button → upload → SSH docker exec sim_harness → SCP fetch `/tmp/rover_runs/` → 4-layer viz.

---

## NTRIP pitfalls

- **Use HTTP/1.0** — NTRIP v1 responds `ICY 200 OK\r\n`; HTTP/1.1 breaks it
- **No GGA to single-base** — sending GGA stops stream. Only for VRS/network-RTK
- **Emlid:** `ICY 200 OK`, RTCM3 immediate, no GGA needed

---

## Corridor navigation

**Corridor mode** = continuous path-following (CTE minimization), replacing waypoint-advance. Corridors = polyline centerline + width, connected by arc/spin turns at headlands.

**Data model** (`tools/corridor.py` + `navigator/corridor.py`): `Corridor` (id, centerline, width, speed, turn_type) → `corridors_to_path()` → `(lat, lon, speed, half_width, is_turn)` polyline. `auto_split_corridors()` uses turn markers (speed<0) to split recorded polylines. `generate_corridor_grid()` for serpentine rows.

**MAVLink:** `cmd=50100` (custom corridor vertex). mavlink_bridge groups by corridor_id → JSON to `corridor_mission` topic.

**Control loop:** Stanley CTE on polyline, lookahead clamped to turn point, decel ramp, overshoot-based turn trigger, pivot spin with bearing to >=2m into next corridor. `max_steering=1.0`.

---

## Mission planner features

**GPS Survey:** u-blox USB → NMEA reader. Modes: Perimeter (fence wall strips, configurable width) or Obstacle (polygon). Averaging 1–60 fixes/vertex. Pulsing dot (green=RTK, yellow=GPS, red=none).

**Path Shift:** N offset copies perpendicular to path direction. Obstacle-aware (shrinks offset in 5cm steps). Chaikin smoothing (preserves first/last 2 points + turn markers).

**Mission Queue:** Auto Upload & Arm checkbox. Sends ARM twice (2nd at +3s for approach-path review). 15s hard cooldown + `wp_active>=0` soft gate prevents race condition. Mode set by RC CH9 — queue only sends ARM, never MODE.

**Beacon enrichment:** `docker-entrypoint.sh` subscribes `wp_active` + `armed` via DDS loopback → JSON broadcast.

**4-layer viz:** Raw (white dots + turn markers), Optimized (green line), Real (CTE-colored GPS track), Sim. Data fetched via SSH.

**CSV import/export:** Columns: `lat,lon,speed,hold_secs,ch5,ch6,ch7,ch8`. Servo columns dense-filled with 1500. `mission_uploader.py` uses its own format `lat,lon,speed,acceptance_radius`.

**Loop mission:** Repeats N laps, joins last→first point. Preserves speeds, servos, lane tags.

---

## Known TODOs

- `sensor_node.py`: stub — wire real I2C sensors (BME280, ultrasonic tank)
- `firmware/*/main.cpp`: needs `pico_sdk_import.cmake` from `$PICO_SDK_PATH/external/`
- Android GQC: RTSP dual video, settings dialog, STATUSTEXT log screen, corridor editor
- Corridor resource management: adapt base-return to save/restore corridor position
