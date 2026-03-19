# AgriRover ROS2 System — CLAUDE.md

Quick-reference for every session. Covers architecture, file map, constants,
build commands, and conventions. Read this before touching any file.

---

## Project purpose

Dual autonomous ground rover platform for agriculture.
- **RV1 (master):** Jetson Orin Nano, HM30 air unit, RP2040 with SBUS→SX1278 TX
- **RV2 (slave):**  Jetson Orin Nano, RP2040 with SX1278 RX→PPM
- **GQC:** SIYI MK32 Android GCS, MAVLink over WiFi
- **Prior codebase:** `C:\agri_rover` (standalone Python, nRF24) — reference only, do not modify

---

## Repo layout

```
ros_agri_rover/
├── CLAUDE.md                        ← this file
├── README.md                        ← human-facing overview
├── ros2_ws/src/
│   ├── agri_rover_interfaces/       ← custom msgs/srvs/actions (ament_cmake)
│   ├── agri_rover_rp2040/           ← USB serial bridge node
│   ├── agri_rover_gps/              ← dual NMEA GPS driver node
│   ├── agri_rover_mavlink/          ← MAVLink ↔ ROS2 bridge node
│   ├── agri_rover_navigator/        ← Stanley path follower + cross-track error publisher
│   ├── agri_rover_sensors/          ← agricultural sensor node (stub)
│   ├── agri_rover_video/            ← GStreamer RTSP streamer node
│   ├── agri_rover_simulator/        ← dead-reckoning GPS simulator (separate Jetson)
│   └── agri_rover_bringup/          ← launch files + per-rover YAML configs
├── firmware/rc_link_sx1278/
│   ├── master/                      ← RP2040: SBUS→PIO decode, PPM→PIO+DMA, SX1278 TX
│   └── slave/                       ← RP2040: SX1278 RX, PPM→PIO+DMA, Jetson serial
├── android/AgriRoverGQC/            ← Android GQC app (Kotlin + Google Maps, fully implemented)
└── tools/
    ├── simulator.py                 ← standalone GPS simulator (no ROS2 required)
    ├── nmea_wifi_rx.py              ← UDP NMEA → PTY virtual serial ports
    ├── rtk_forwarder.py             ← NTRIP/E610 RTCM3 → u-blox serial
    ├── start_rover1_sim.sh          ← single-command RV1 simulation launcher
    ├── start_rover2_sim.sh          ← single-command RV2 simulation launcher
    ├── sim_navigator.py             ← software-in-the-loop simulation of navigator.py (full-path Stanley + MPC + pivot turns + obstacle avoidance); importable by monitor.py or standalone CLI; reads nav params from rover1_params.yaml at import time (falls back to hardcoded defaults)
    ├── mission_planner.py           ← web-based mission route editor + SIL simulator (HTTP :8089); satellite map, waypoint drag, obstacle polygon drawing, simulate overlays path + pivot/bypass-pivot markers
    ├── monitor.py                   ← terminal MAVLink dashboard; rover IPs, SBUS/PPM, XTE stats; snoops GQC missions, runs SIL sim, serves Leaflet map on :8088 (HTTP) — features: auto-fit toggle, persistent zoom/center via localStorage, 5s auto-refresh
    └── mission_uploader.py          ← CSV waypoints → MAVLink mission upload
```

---

## Communication layers

| Layer    | Protocol            | Hardware path                          | Rate    | Purpose                   |
|----------|---------------------|----------------------------------------|---------|---------------------------|
| RC       | SBUS → SX1278 LoRa  | MK32→HM30→RP2040(master)→SX1278→RP2040(slave) | 25 Hz | Manual control, safety |
| MAVLink  | UDP / WiFi          | Android GQC ↔ Each Jetson              | 10 Hz   | Telemetry, missions, cmds |
| Video    | RTSP / WiFi         | Each Jetson → GQC                      | 30 fps  | Camera feeds              |
| ROS2     | DDS / WiFi          | Jetson ↔ Jetson (same domain)          | varies  | Autonomy, coordination    |

---

## ROS2 packages

| Package                  | Type         | Node executable    | Key responsibility                    |
|--------------------------|--------------|--------------------|---------------------------------------|
| agri_rover_interfaces    | ament_cmake  | —                  | RCInput, SensorData, MissionWaypoint msgs; SetMode, ArmDisarm srvs; FollowMission action |
| agri_rover_rp2040        | ament_python | rp2040_bridge      | USB serial ↔ ROS2: reads CH: lines, sends \<HB:\> \<J:\> |
| agri_rover_gps           | ament_python | gps_driver         | Dual NMEA serial → NavSatFix + heading Float32 |
| agri_rover_mavlink       | ament_python | mavlink_bridge     | ROS2 topics ↔ MAVLink UDP to GQC     |
| agri_rover_navigator     | ament_python | navigator          | Full-path Stanley/MPC + pivot turns (incl. bypass corners) + obstacle pre-reroute; publishes XTE |
| agri_rover_sensors       | ament_python | sensor_node        | Tank/temp/humidity/pressure (stub)    |
| agri_rover_video         | ament_python | video_streamer     | GStreamer RTSP server                 |
| agri_rover_simulator     | ament_python | simulator          | Dead-reckoning GPS simulator (runs on separate Jetson) |
| agri_rover_bringup       | ament_cmake  | —                  | rover1.launch.py, rover2.launch.py    |

**ROS2 namespaces:** `/rv1/` for master, `/rv2/` for slave. Same `ROS_DOMAIN_ID`.

---

## ROS2 topic map (per rover, prefix /rvN/)

| Topic           | Type                            | Producer       | Consumers                  |
|-----------------|---------------------------------|----------------|----------------------------|
| rc_input        | agri_rover_interfaces/RCInput   | rp2040_bridge  | mavlink_bridge             |
| mode            | std_msgs/String                 | rp2040_bridge, mavlink_bridge | navigator, mavlink_bridge |
| cmd_override    | agri_rover_interfaces/RCInput   | navigator                   | rp2040_bridge |
| servo_state     | agri_rover_interfaces/RCInput   | mavlink_bridge              | navigator     |
| armed           | std_msgs/Bool                   | mavlink_bridge              | navigator     |
| wp_active       | std_msgs/Int32                  | navigator      | mavlink_bridge             |
| xte             | std_msgs/Float32                | navigator      | mavlink_bridge             |
| fix             | sensor_msgs/NavSatFix           | gps_driver     | navigator, mavlink_bridge  |
| fix_front       | sensor_msgs/NavSatFix           | gps_driver     | navigator                  |
| heading         | std_msgs/Float32                | gps_driver     | navigator, mavlink_bridge  |
| rtk_status      | std_msgs/String                 | gps_driver     | mavlink_bridge             |
| sensors         | agri_rover_interfaces/SensorData | sensor_node   | mavlink_bridge             |
| mission         | agri_rover_interfaces/MissionWaypoint | mavlink_bridge | navigator         |
| mission_fence   | std_msgs/String (JSON)          | mavlink_bridge | navigator          |

---

## MAVLink & network constants

| Item             | Value                  | Notes                                   |
|------------------|------------------------|-----------------------------------------|
| RV1 sysid        | 1                      | bind port :14550                        |
| RV2 sysid        | 2                      | bind port :14550 (same as RV1 — different Jetson, no conflict) |
| GQC sysid        | 255                    | sends to WiFi broadcast (subnet .255)   |
| RV1 video        | rtsp://rv1-ip:8554/stream |                                      |
| RV2 video        | rtsp://rv2-ip:8555/stream |                                      |
| MAVLink version  | v2 (MAVLINK20=1)       | set in all Python node env              |

GQC identifies rovers by **sysid in HEARTBEAT**, not by port.
`mavlink_bridge` uses a dedicated outbound `socket.SOCK_DGRAM` (`self._udp_sock`) created in `__init__` — pymavlink's internal socket attribute varies by version and must not be accessed directly. Never use `xxx_send()` convenience methods on a `udpin:` connection — they call `mav.file.write()` which fails silently. Use `xxx_encode()` + `_send()` instead.
`gqc_host` in params must match the **broadcast address of the rover's WiFi subnet** (e.g. `192.168.100.255` for a `192.168.100.x` network).

**Broadcast vs unicast:**
- `_send()` always uses `self._gqc_addr` (broadcast) so passive tools (`monitor.py`, `simulator.py`) continue to receive telemetry without registering their IPs.
- If `self._gqc_unicast` is known, `_send()` **also** sends a second copy to that unicast address to bypass WiFi AP DTIM buffering (~100 ms latency).
- `_send_mission_request()` uses `self._gqc_unicast or self._gqc_addr` for the handshake.
- `_gqc_unicast` is updated only from **sysid 255** packets via raw `sock.recvfrom()`.
- Unicast target port is forced to `gqc_port` (14550) to ensure delivery even if GQC uses a random source port.

**Mission upload — streaming protocol:**
- GQC calls `streamMission()`: sends MISSION_COUNT, waits 150 ms, then pushes all items sequentially with 20 ms inter-packet gap (no waiting for REQUEST_INT per item).
- Rover `_on_mission_item` accepts items as they arrive in order; does **not** send REQUEST_INT for the next item after each receipt.
- `_mission_retry` timer fires every 0.5 s; sends REQUEST_INT only if item has not arrived within **250 ms** (packet-loss fallback).
- First REQUEST_INT (seq=0) is delayed 150 ms after MISSION_COUNT to avoid race condition where rover responds before GQC's coroutine is listening.
- Per-item RTT is logged: `ITEM seq=N RTT=Xms` — useful for diagnosing network delays.
- Root causes found and fixed: Jetson WiFi power-save (100–900 ms/item) + per-item DTIM round-trip (66–150 ms/item). Startup scripts now run `iw dev <iface> set power_save off` before the container starts.

---

## Firmware — RP2040 pin assignments

| Signal       | GPIO | Pico Pin | Direction | Notes                              |
|--------------|------|----------|-----------|------------------------------------|
| SBUS RX      | GP4  | 6        | IN        | Inverted 100kbaud 8E2 from HM30 (master only) |
| PPM OUT      | GP15 | 20       | OUT       | To motor controllers / ESC (both)  |
| SX1278 MISO  | GP16 | 21       | IN        | SPI0 RX                            |
| SX1278 NSS   | GP17 | 22       | OUT       | SPI0 chip select (active LOW)      |
| SX1278 SCK   | GP18 | 24       | OUT       | SPI0 SCK                           |
| SX1278 MOSI  | GP19 | 25       | OUT       | SPI0 TX                            |
| SX1278 RST   | GP20 | 26       | OUT       | Hardware reset (active LOW pulse)  |
| SX1278 DIO0  | GP21 | 27       | IN        | TxDone (master) / RxDone (slave)   |

### SX1278 module wiring (same for master and slave)

Wires the RP2040 Pico to a standard Ra-02 / RA-01S style SX1278 breakout.
The module runs at **3.3 V** — do not connect to 5 V.

| SX1278 module pin | RP2040 GPIO | Pico Pin | Notes                          |
|-------------------|-------------|----------|--------------------------------|
| VCC               | —           | 36 (3V3) | 3.3 V supply from Pico onboard regulator |
| GND               | —           | 38 (GND) | Common ground                  |
| SCK               | GP18        | 24       | SPI clock                      |
| MISO              | GP16        | 21       | Master-in / slave-out          |
| MOSI              | GP19        | 25       | Master-out / slave-in          |
| NSS (CS)          | GP17        | 22       | Chip select, idle HIGH         |
| RST               | GP20        | 26       | Reset — pulled HIGH after init |
| DIO0              | GP21        | 27       | IRQ: TxDone (master) or RxDone (slave); no pull-up needed, module drives it |

**Master-specific:** after `sx1278_init()`, `RegDioMapping1 = 0x40` → DIO0 signals TxDone.
**Slave-specific:** `sx1278_start_rx()` sets `RegDioMapping1 = 0x00` → DIO0 signals RxDone; slave polls via `sx1278_recv()` in main loop.

No other DIOx pins (DIO1–DIO5) are used — leave them unconnected.

---

## Firmware — PIO design

**Master RP2040:**
- `PIO0 SM0` — `sbus_rx.pio`: inverted UART RX, 8× oversampling, clkdiv=156.25
  - `wait 1 pin 0` (HIGH = inverted start bit), samples 8 bits, `push noblock`
  - CPU reads: `byte = (pio_sm_get(pio0, 0) >> 24) ^ 0xFF`
- `PIO0 SM1` — `ppm_tx.pio`: 1 µs/cycle (clkdiv=125), side-set drives PPM pin
  - DMA feeds 20-word buffer: alternating `[HIGH_COUNT=298, LOW_CHn]` pairs
  - DMA IRQ0 fires at end of each 20 ms frame → `ppm_buf_update()` → restart DMA
  - `LOW_CHn = channel_µs - 302`, `LOW_SYNC = 20000 - Σchannels - 302`

**Slave RP2040:**
- `PIO0 SM0` — same `ppm_tx.pio`, same DMA setup
- No SBUS PIO — SX1278 feeds `rf_ch[]` directly from `sx1278_recv()`

**SX1278 LoRa config:** 433 MHz (0x6C8000), SF7, BW500kHz, CR4/5, explicit header, CRC on, +17dBm.
**TX rate:** 25 Hz (every 2nd PPM frame) — LoRa ToA ≈ 12.9 ms, 40 ms period = 32% duty cycle.
**Payload:** 16 × uint16_t = 32 bytes, little-endian, raw SBUS channels in µs.
**Important:** master always transmits `sbus_ch[]` (raw SBUS), never `out_ch[]`. Slave applies `apply_ppm_map()` once on reception. Using `out_ch` would double-map channels and send neutral (1500) in RELAY mode.

---

## Mode logic (both RP2040s)

```
── Master (RV1) ──────────────────────────────────────────────────────────────
SBUS lost/timeout (200ms)          → EMERGENCY
CH4 (SWA) < 1700                   → EMERGENCY  ← only emergency trigger besides signal loss
CH9 > 1750                         → RELAY  (slave selected: master neutral, LoRa still forwarding)
CH9 in [1250, 1750]:
  CH5 (SWB) > 1700:
    + no Jetson heartbeat          → AUTO-NO-HB  (neutral)
    + heartbeat but no cmd         → AUTO-TIMEOUT (neutral)
    + heartbeat + fresh cmd        → AUTONOMOUS  (Jetson controls master)
  CH5 <= 1700                      → RELAY  (master neutral, raw SBUS forwarded to slave)
CH9 < 1250                         → MANUAL (master moves, raw SBUS passthrough)

── Slave (RV2) ───────────────────────────────────────────────────────────────
RF lost/timeout (500ms)            → EMERGENCY
CH4 (from RF packet) < 1700        → EMERGENCY  ← only emergency trigger besides signal loss
CH9 (from RF packet) < 1250        → IDLE  (master selected: slave neutral, not an alarm)
CH9 in [1250, 1750]:
  CH5 > 1700 + HB alive + fresh cmd → AUTONOMOUS (Jetson controls slave)
  else                               → IDLE  (neither rover selected manually, not an alarm)
CH9 > 1750                         → MODE_RF (slave moves, RC values from master relay)
```

The 3-position CH9 switch selects which rover moves in manual:
- **Low  (< 1250):** RV1 manual, RV2 neutral
- **Middle (1250–1750):** both neutral in manual; AUTO valid for each Jetson independently
- **High  (> 1750):** RV2 manual (via RF relay), RV1 neutral

---

## RP2040 ↔ Jetson serial protocol

```
RP2040 → Jetson:
  CH:1500,1500,1700,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500 MODE:MANUAL  (every frame, 16 channels)
  [SBUS_OK] / [SBUS_LOST]                                        (on change)
  [RF_LINK_OK] / [RF_LINK_LOST]                                  (slave, on change)
  <HB:N+1>                                                       (heartbeat echo)

Jetson → RP2040:
  <HB:N>                 heartbeat (must arrive < 300 ms for AUTO to stay active)
  <J:c0,c1,c2,c3,c4,c5,c6,c7>   8-channel autonomous command (µs)
```

### Servo command flow (DO_SET_SERVO)

`MAV_CMD_DO_SET_SERVO` (cmd=183): `param1` = servo number (5–8), `param2` = PWM µs.

```
GQC COMMAND_LONG / MISSION_ITEM_INT
  → mavlink_bridge._apply_servo_cmd()
  → servo_state topic  (RCInput, channels 4-7 set, channels 0-3 = 0)
  → navigator._cb_servo_state() → self._servo_ch[0..3] updated
  → navigator._publish_cmd() includes _servo_ch in channels 4-7 at 10 Hz
  → cmd_override topic (all 8 channels)
  → rp2040_bridge → <J:c0,...,c7>
```

Navigator continuously re-publishes servo state with every cmd_override tick (10 Hz),
so values are re-applied even if the RP2040 resets or a packet is lost.

| Channel  | PPM ch | Servo number |
|----------|--------|--------------|
| index 4  | CH5    | servo 5      |
| index 5  | CH6    | servo 6      |
| index 6  | CH7    | servo 7      |
| index 7  | CH8    | servo 8      |

Handled in `mavlink_bridge._apply_servo_cmd()` — called from both `_on_command_long`
(direct GQC command) and `_on_mission_item` (DO_SET_SERVO in uploaded mission).

---

## Build commands

```bash
# ── Isaac ROS Docker (preferred on Jetson) ────────────────────────────────────
# Build image (once, or after Dockerfile/dependency changes)
docker build -t agri_rover:latest .

# Simulation (single command — handles PTY, container, GPS)
bash tools/start_rover1_sim.sh    # RV1 Jetson
bash tools/start_rover2_sim.sh    # RV2 Jetson

# Production (real hardware)
docker compose up rover1          # master Jetson (camera_source=csi by default)
docker compose up rover2          # slave Jetson

# Override camera source
CAMERA_SOURCE=usb docker compose up rover1

# ── Inside running container ──────────────────────────────────────────────────
docker exec -it agri_rover_rv1 bash
source /opt/ros/*/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 topic list
ros2 param get /rv1/gps_driver primary_port

# ── Firmware (dev machine, Pico SDK required) ─────────────────────────────────
cd firmware/rc_link_sx1278/master
cmake -B build -DPICO_SDK_PATH=$PICO_SDK_PATH && cmake --build build

# ── Tools ─────────────────────────────────────────────────────────────────────
python tools/monitor.py                        # map → http://<host>:8088/monitor_map.html
python tools/monitor.py --map-port 9000        # override HTTP port
python tools/mission_uploader.py missions/field.csv --rover 1 --host 192.168.100.19
```

---

## Config files

| File                                              | What to edit                              |
|---------------------------------------------------|-------------------------------------------|
| `ros2_ws/src/agri_rover_bringup/config/rover1_params.yaml` | Ports, IPs, speeds for RV1   |
| `ros2_ws/src/agri_rover_bringup/config/rover2_params.yaml` | Ports, IPs, speeds for RV2   |
| `ros2_ws/src/agri_rover_navigator/config/navigator_params.yaml` | Lookahead, speed limits, `obstacle_clearance_m` |
| `ros2_ws/src/agri_rover_video/config/gstreamer.yaml` | Camera source, resolution, bitrate |

**YAML key format — critical:** Parameter YAML keys must use the **fully-qualified node name** including namespace, or ROS2 silently ignores them and nodes use code defaults. Example:
```yaml
# WRONG — does not match namespaced node /rv1/gps_driver
gps_driver:
  ros__parameters:
    primary_port: /dev/ttyUSB0

# CORRECT
/rv1/gps_driver:
  ros__parameters:
    primary_port: /dev/ttyUSB0
```
The config directory is volume-mounted into the container (`docker-compose.yml`) so edits on the host take effect on the next container restart — no image rebuild needed.

---

## Conventions

- **Python nodes:** rclpy, class inherits `Node`, parameters declared in `__init__`, timers not threads where possible.
- **Namespacing:** all topics are relative (no leading `/`), launch files set `namespace='/rv1'` or `'/rv2'`. YAML param keys must include the namespace (`/rv1/node_name:`) — see Config files section.
- **MAVLink:** always `os.environ['MAVLINK20'] = '1'` before importing pymavlink. Use `xxx_encode()` + `_send()` — never `xxx_send()` convenience methods. `_send()` uses a dedicated `self._udp_sock` (broadcast SOCK_DGRAM created in `__init__`) — do not access pymavlink's internal socket attributes, they vary by version.
- **GPS port retry:** `gps_driver._start_reader()` retries opening the serial port every 5 s. This is intentional — in simulation mode, `nmea_wifi_rx.py` starts inside the container after `gps_driver` and the port doesn't exist immediately.
- **PTY and Docker devpts:** Docker containers have their own `devpts` filesystem. PTY devices created on the host are NOT visible inside the container even with `/dev:/dev` mounted. Always run `nmea_wifi_rx.py` inside the container (via `docker exec -d`) so its PTYs live in the container's `/dev/pts`.
- **PPM buffer:** only updated inside `dma_irq_handler()` (IRQ context). Main loop writes `out_ch[]` (volatile), IRQ reads it. No mutex needed on M0+ for uint16_t aligned writes.
- **SX1278 TX:** fire-and-forget. Check `RegIrqFlags.TxDone` before next TX. Returns `false` if still busy — caller skips that frame.
- **LoRa payload = raw SBUS only:** master transmits `sbus_ch[]`, never `out_ch[]`. `out_ch` is PPM-remapped (and neutralised in RELAY mode) — sending it would double-map on the slave and break RELAY forwarding. Slave calls `apply_ppm_map()` exactly once on the received `rf_ch[]`.
- **SBUS decode:** validate `frame[0]==0x0F && frame[24]==0x00` before accepting. Re-sync on every `0x0F` byte regardless of position.

---

## Isaac ROS — architecture

The ROS2 workspace runs inside NVIDIA Isaac ROS (Jazzy, aarch64) on each Jetson.

| Component            | Isaac ROS role                                              |
|----------------------|-------------------------------------------------------------|
| `agri_rover_video`   | CSI path uses NITROS composable pipeline (see below)        |
| All other nodes      | Standard rclpy — no NITROS changes needed                   |
| Docker base image    | `nvcr.io/nvidia/isaac/ros:isaac_ros_740c8500df2685ab1f4a4e53852601df-arm64-jetpack` (Ubuntu 24.04 noble) |

**Video pipeline (CSI camera, `camera_source:=csi`):**
```
[ComposableNodeContainer  /rvN/video_container]
  isaac_ros_argus_camera::ArgusMonoNode   → camera/image_raw  (NITROS Image)
  isaac_ros_h264_encoder::EncoderNode     → image_compressed  (NITROS H264)
[VideoStreamerNode  /rvN/video_streamer]
  subscribes to image_compressed → GstRtspServer appsrc → RTSP clients (GQC)
```
`ArgusMonoNode` and `EncoderNode` must share a `component_container_mt` to use
zero-copy NITROS transport. `VideoStreamerNode` is a separate Python node that
subscribes to the output topic (`sensor_msgs/CompressedImage`, format `"h264"`).

**Fallback (USB / test camera):** `video_streamer._start_fallback_rtsp()` uses
`GstRtspServer` Python GI directly (same library already imported). There is no
`gst-rtsp-server` binary in the container — do not use subprocess for this.

**Docker files:**
- `Dockerfile` — extends Isaac ROS base, installs deps, builds workspace. Uses dynamic ROS distro detection (`ls /opt/ros/*/setup.bash | head -1`) since the base image may ship Humble or Jazzy. pip installs use `--break-system-packages` (Python 3.12 PEP 668).
- `docker-entrypoint.sh` — sources ROS2 + project overlay dynamically
- `docker-compose.yml` — runs as `user: root`; volume mounts: `/dev`, `/tmp` (PTY symlinks), `./ros2_ws/src` (live nodes), `./tools` (sim scripts), config dir + launch dir overrides (both bypass cmake copy so edits take effect on restart without rebuild)

**Live-edit without rebuild:** Python node files and config YAML are volume-mounted so changes on the host take effect immediately. Restart the container (`docker compose down rover1 && bash tools/start_rover1_sim.sh`) — no `docker build` needed.

---

## Android GQC — status

Implemented at `android/AgriRoverGQC/`. Stack: Kotlin, Google Maps SDK, java-mavlink (io.dronefleet), AppCompat. No Jetpack Compose — standard XML layouts.

### Key classes

| Class | Responsibility |
|-------|----------------|
| `MainActivity` | UI, map, mode switching, listeners |
| `RoverPositionManager` | All MAVLink UDP I/O (send + receive) |

### RoverPositionManager API

- `startListening()` / `stopListening()` — bind UDP :14550, start heartbeat + watchdog loops
- `sendCommand(sysId, commandId, p1, p2)` — single COMMAND_LONG fire-and-forget
- `sendCriticalCommand(sysId, commandId, p1, p2)` — **3× at 100 ms intervals** — use for E-STOP and disarm (UDP packet loss must not prevent stopping)
- `sendRcChannelsOverride(sysId, channels)` — RC_CHANNELS_OVERRIDE 8×PPM µs
- `uploadMission(sysId, waypoints)` — plain waypoint-only MAVLink mission upload
- `uploadRecordedMission(sysId, actions)` — uploads a mixed `List<MissionAction>` containing both `MissionAction.Waypoint` and `MissionAction.ServoCmd` items as interleaved `NAV_WAYPOINT` + `DO_SET_SERVO` MISSION_ITEM_INT messages

**sysId=0 → broadcast to 255.255.255.255:14550** (both rovers).

Rover IPs are auto-discovered from first incoming packet of each sysid — no manual IP configuration needed.

### MAVLink ports (both rovers)
Both rovers bind :14550 (same port, different Jetsons on the network).
GQC identifies rovers by **sysid** in HEARTBEAT, not by port.
GQC always sends to broadcast 192.168.100.255:14550 for discovery; unicasts to known rover IP once discovered.
**Do not change RV2 bind_port back to 14551** — GQC hardcodes port 14550 for all outbound packets.

### Modes

| Mode | Bottom bar | Behaviour |
|------|------------|-----------|
| MANUAL | (none) | Physical RC sticks → HM30 → SBUS → RP2040. App shows map + status only. |
| PLANNER | Add-point FABs, ADD, REC, CLEAR, UPLOAD | Finger-draw or record route on satellite map → upload mission |
| AUTO | R1/R2 toggle, START, PAUSE, CLEAR | ARM + set AUTO mode; mission progress shown on map |

Physical RC control is always active regardless of app mode. The app does NOT send RC override in MANUAL mode (virtual D-pad was removed).

### Mission recording (PLANNER mode)

Two recording methods in PLANNER toolbar:

**ADD button** — tap once to insert the selected rover's current GPS position as a single waypoint.

**REC button** — continuous recording:
- Samples rover GPS position every 500 ms → appends `MissionAction.Waypoint`
- Monitors PPM CH5-CH8 (auxiliary switches) from incoming `RC_CHANNELS`; any channel change > 100 µs → appends `MissionAction.ServoCmd(servo, pwm)` immediately
- Button label toggles `⏺ REC` ↔ `⏹ STOP`

**OBS button** — draws obstacle exclusion polygons:
- Press OBS → enters drawing mode (button turns red, label changes to `✓ DONE`)
- Tap the map to add vertices one at a time
- Press DONE → polygon is closed (minimum 3 vertices) and rendered as a red semi-transparent overlay
- Multiple polygons can be added before upload
- CLEAR removes all obstacles

**UPLOAD** — sends DISARM (cmd 400, p1=0) × 3 at 100 ms intervals before `MISSION_COUNT` as a safety measure, then streams all items.
- If `obstaclePolygons` is non-empty: calls `uploadMissionWithObstacles()` — fence vertices (cmd=5003) are prepended before nav/servo items in a single `streamMission()` call.
- Else if `recordedMission` contains `ServoCmd` entries: calls `uploadRecordedMission()`.
- Otherwise: calls plain `uploadMission()`.

```kotlin
sealed class MissionAction {
    data class Waypoint(val lat: Double, val lon: Double) : MissionAction()
    data class ServoCmd(val servo: Int, val pwm: Int)     : MissionAction()
}
```

Servo numbers 5–8 correspond to PPM CH5–CH8 (auxiliary motors, e.g. sprayer pump).
Servo state is recorded relative to the **PPM channel values** as reported in `RC_CHANNELS`
from the rover (post firmware PPM remap — CH5 = SBUS CH11, CH6 = SBUS CH12, CH7 = ~SBUS CH7, CH8 = ~SBUS CH8).

### Screen always-on
`window.addFlags(FLAG_KEEP_SCREEN_ON)` is set in `onCreate` — screen never dims while app is running.

### Safety
- E-STOP button broadcasts DISARM (cmd 400, p1=0) to all rovers via `sendCriticalCommand` (3× UDP retry).
- `checkLinkMismatch()` detects RC switch ≠ slave HEARTBEAT mismatch after 2 s and auto-disarms via `sendCriticalCommand`.

### Map
- Default view: Jalisco field (`20.727715, -103.566782`, zoom 18)
- Satellite map by default; toggle to standard via layers FAB
- Per-rover markers: red=RV1, blue=RV2. Centre dot: green=disarmed, orange=armed, yellow=AUTO.
- White ring around selected rover marker.
- Mission route shown as two-color polylines: green = walked segment, red = pending segment. Updated from `onMissionProgress` callback via `roverNextWaypointIndex`.

---

## Simulator — dead-reckoning GPS

Runs on a **third Jetson Orin Nano** (not RV1, not RV2). Replaces real u-blox ZED-X20P hardware during development.

### Hardware connections

| Port on simulator Jetson | Purpose |
|--------------------------|---------|
| `/dev/ttyACM0` (USB)     | ppm_decoder RP2040 — reads `RV1:ch0..7 RV2:ch0..7\n` at 50 Hz |

NMEA is sent over **WiFi (UDP)** — no USB serial connections to rover Jetsons needed.

### Architecture

```
Simulator Jetson Orin Nano                 WiFi (UDP)          Rover Jetson Nano
  tools/simulator.py                  ──────────────────►  tools/nmea_wifi_rx.py
  reads ppm_decoder USB                rv1-ip:5000 (pri)        │
  dead-reckoning physics               rv1-ip:5001 (sec)    /dev/pts/N  ← gps_driver primary
  $GNGGA + $GNVTG @ 10 Hz             rv2-ip:5000 (pri)    /dev/pts/M  ← gps_driver secondary
                                       rv2-ip:5001 (sec)

  RTK corrections (real hardware only):
  NTRIP caster (internet) ──────────────────────────────►  tools/rtk_forwarder.py
  E610 DTU base station (Ethernet) ─────────────────────►       │
                                                          /dev/ttyUSB0 → u-blox primary
                                                          /dev/ttyUSB1 → u-blox secondary
```

- `nmea_wifi_rx.py` creates two PTY virtual serial ports on the rover Jetson.
- `gps_driver.py` reads PTYs as if they were physical GPS modules — no code changes.
- Secondary GGA is offset `antenna_baseline_m` (default 1.00 m, set to match real rover antenna separation) ahead along rover heading → `gps_driver` recovers heading via `atan2(dlon × cos(lat), dlat)`, identical to real hardware.
- The `cos(lat)` correction is required because `secondary_pos()` scales `dlon` by `1/cos(lat)` to convert metres to longitude degrees; omitting it causes heading to drift as the rover moves and latitude changes.

### Physics model (skid/differential steer)

```python
speed = (throttle_ppm - 1500) / 500 * max_speed_mps          # m/s, negative = reverse
steer = (steering_ppm - 1500) / 500                           # -1..+1
omega = turn_scale * 2 * steer * max_speed_mps / wheelbase_m  # rad/s, independent of forward speed
heading += omega * dt
lat += speed * cos(heading) * dt / 111320
lon += speed * sin(heading) * dt / (111320 * cos(lat_rad))
```

`turn_scale` (0.0–1.0, default **1.0**): scales the maximum turn rate relative to full differential.
- `1.0` → full differential (≈ 360° in 1.7 s at full steer) — default, matches MPC kinematic model
- `0.1` → ~10% of max (≈ 360° in 17 s at full steer) — slow testing
- Pass `--turn-scale` to `tools/simulator.py` or set `turn_scale` in `simulator_params.yaml`.

### Startup order (simulation)

```bash
# ── Each rover Jetson — single command handles everything ─────────────────────
bash tools/start_rover1_sim.sh    # RV1 Jetson
bash tools/start_rover2_sim.sh    # RV2 Jetson
# Starts container (detached), execs nmea_wifi_rx.py inside container,
# follows logs. Ctrl-C cleanly stops container and removes it.

# ── Simulator Jetson (run after rover containers are up) ─────────────────────
python3 tools/simulator.py \
  --rv1-ip 192.168.100.19 --rv2-ip 192.168.100.20 \
  --ppm-port /dev/ttyACM0
```

**Why nmea_wifi_rx.py runs inside the container:** Docker containers have their own
`devpts` mount. PTYs created on the host are not visible inside the container even
with `/dev:/dev`. Running `nmea_wifi_rx.py` via `docker exec -d` inside the container
creates PTYs in the container's `/dev/pts`. Symlinks appear at `/tmp/rv1_gps_{pri,sec}`
(shared via `/tmp:/tmp` mount). `gps_driver` retries port open every 5 s and connects
once the symlinks are ready.

### Startup order (real hardware + RTK corrections)

```bash
# On each rover Jetson — forwards RTCM3 to u-blox modules on ttyUSB0/ttyUSB1
# Source option 1: NTRIP internet caster
python3 tools/rtk_forwarder.py --source ntrip \
  --ntrip-host <caster-ip> --ntrip-port 2101 \
  --mountpoint <mount> --ntrip-user <user> --ntrip-pass <pass> \
  --gps-ports /dev/ttyUSB0 /dev/ttyUSB1 \
  --log-file /tmp/rtk.log

# Source option 2: E610 DTU base station (Jetson Ethernet port → E610 LAN port)
python3 tools/rtk_forwarder.py --source e610 \
  --e610-host 192.168.1.20 --e610-port 9000 \
  --gps-ports /dev/ttyUSB0 /dev/ttyUSB1
```

### ROS2 simulator node (alternative — requires Isaac ROS)

```bash
ros2 launch agri_rover_simulator simulator.launch.py
ros2 topic echo /sim/rv1/fix
ros2 topic echo /sim/rv2/fix
```

---

## Tools

All scripts under `tools/` are pure Python 3 + pyserial. No ROS2 required.

| Script | Runs on | Purpose |
|--------|---------|---------|
| `simulator.py` | Simulator Jetson | Dead-reckoning physics → NMEA over UDP WiFi |
| `nmea_wifi_rx.py` | Each rover Jetson | Receives UDP NMEA → PTY virtual serial ports for gps_driver |
| `rtk_forwarder.py` | Each rover Jetson | NTRIP/E610 RTCM3 → u-blox serial (real hardware) |
| `sim_navigator.py` | Any (no ROS2) | SIL simulation of navigator.py — Stanley/MPC, pivot turns (incl. bypass corners), obstacle avoidance; reads params from `rover1_params.yaml` at import; standalone CLI or importable. Diagnostics → `tools/obstacle_debug.log` |
| `mission_planner.py` | Dev machine | Web mission editor + SIL (HTTP :8089); satellite map, drag waypoints, draw obstacle polygons, Simulate shows path + orange pivot / purple bypass-pivot markers |
| `monitor.py` | Dev machine / RPi (SSH) | Terminal dashboard + Leaflet map (HTTP :8088); snoops GQC mission uploads and auto-simulates via sim_navigator.py |
| `mission_uploader.py` | Dev machine | CSV waypoints → MAVLink mission upload |

### NTRIP pitfalls (rtk_forwarder.py)

- **Use HTTP/1.0** — NTRIP v1 casters (including Emlid) respond with `ICY 200 OK\r\n` (single CRLF). HTTP/1.1 persistent-connection semantics confuse them.
- **Do not send GGA unconditionally** — single-base casters start streaming immediately after the HTTP request. Sending a GGA sentence into a v1 stream causes the caster to stop sending. Only send GGA if `--approx-lat`/`--approx-lon` are given (needed for VRS/network-RTK casters only).
- **Emlid caster behaviour**: responds `ICY 200 OK\r\n` (no double CRLF), starts RTCM3 immediately after connect, no GGA needed.
- **Disconnect reason** is logged: `no data for 30 s` = base offline; `stream closed by caster (EOF)` = caster dropped connection.
- **Test without the script**: `str2str -in ntrip://user:pass@host:port/mount | xxd | head -20`

---

## Known TODOs

- `sensor_node.py`: stub returns hardcoded values — wire real I2C sensors (BME280, ultrasonic tank)
- `firmware/*/main.cpp`: SX1278 driver is complete but needs `pico_sdk_import.cmake` copied from `$PICO_SDK_PATH/external/`
- `ppm_tx.pio`: slave uses SM0 (not SM1) — one less state machine needed vs master
- Android GQC: RTSP dual video screen, settings dialog, STATUSTEXT log screen still TODO (see android/AgriRoverGQC/README.md)
- Navigator obstacle avoidance: single-pass per segment — a segment intersecting multiple non-adjacent polygons is rerouted around the first one only; complex overlapping layouts may need manual adjustment. Polygons spanning waypoints (cross-segment case) and the rover-start → wp[0] leg are both handled correctly.
- DO_SET_SERVO in missions: applied at upload time, re-published continuously by navigator at 25 Hz. For precise per-waypoint timing during replay (servo fires when rover physically reaches GPS position), the navigator would need to sequence through mixed mission items (requires MissionWaypoint interface change or a new ServoEvent topic).
