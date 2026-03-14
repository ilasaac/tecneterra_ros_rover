# AgriRover ROS2 System — CLAUDE.md

Quick-reference for every session. Covers architecture, file map, constants,
build commands, and conventions. Read this before touching any file.

---

## Project purpose

Dual autonomous ground rover platform for agriculture.
- **RV1 (master):** Jetson Nano, HM30 air unit, RP2040 with SBUS→SX1278 TX
- **RV2 (slave):**  Jetson Nano, RP2040 with SX1278 RX→PPM
- **GQC:** custom Android app, MAVLink over WiFi
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
│   ├── agri_rover_navigator/        ← pure-pursuit autonomous navigator node
│   ├── agri_rover_sensors/          ← agricultural sensor node (stub)
│   ├── agri_rover_video/            ← GStreamer RTSP streamer node
│   └── agri_rover_bringup/          ← launch files + per-rover YAML configs
├── firmware/rc_link_sx1278/
│   ├── master/                      ← RP2040: SBUS→PIO decode, PPM→PIO+DMA, SX1278 TX
│   └── slave/                       ← RP2040: SX1278 RX, PPM→PIO+DMA, Jetson serial
├── android/AgriRoverGQC/            ← Android GQC app (Kotlin scaffold, not yet built)
└── tools/
    ├── monitor.py                   ← terminal MAVLink dashboard (both rovers)
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
| agri_rover_navigator     | ament_python | navigator          | Pure-pursuit waypoint follower        |
| agri_rover_sensors       | ament_python | sensor_node        | Tank/temp/humidity/pressure (stub)    |
| agri_rover_video         | ament_python | video_streamer     | GStreamer RTSP server                 |
| agri_rover_bringup       | ament_cmake  | —                  | rover1.launch.py, rover2.launch.py    |

**ROS2 namespaces:** `/rv1/` for master, `/rv2/` for slave. Same `ROS_DOMAIN_ID`.

---

## ROS2 topic map (per rover, prefix /rvN/)

| Topic           | Type                            | Producer       | Consumers                  |
|-----------------|---------------------------------|----------------|----------------------------|
| rc_input        | agri_rover_interfaces/RCInput   | rp2040_bridge  | mavlink_bridge             |
| mode            | std_msgs/String                 | rp2040_bridge  | navigator, mavlink_bridge  |
| cmd_override    | agri_rover_interfaces/RCInput   | navigator OR mavlink_bridge | rp2040_bridge |
| fix             | sensor_msgs/NavSatFix           | gps_driver     | navigator, mavlink_bridge  |
| heading         | std_msgs/Float32                | gps_driver     | navigator, mavlink_bridge  |
| rtk_status      | std_msgs/String                 | gps_driver     | mavlink_bridge             |
| sensors         | agri_rover_interfaces/SensorData | sensor_node   | mavlink_bridge             |
| mission         | agri_rover_interfaces/MissionWaypoint | mavlink_bridge | navigator         |

---

## MAVLink & network constants

| Item             | Value                  | Notes                                   |
|------------------|------------------------|-----------------------------------------|
| RV1 sysid        | 1                      | bind port :14550                        |
| RV2 sysid        | 2                      | bind port :14551                        |
| GQC sysid        | 255                    | sends to broadcast 192.168.1.255:14550  |
| RV1 video        | rtsp://rv1-ip:8554/stream |                                      |
| RV2 video        | rtsp://rv2-ip:8555/stream |                                      |
| MAVLink version  | v2 (MAVLINK20=1)       | set in all Python node env              |

GQC identifies rovers by **sysid in HEARTBEAT**, not by port.
`mavlink_bridge` uses `socket.sendto()` directly (not `mav.write()`) — see memory note on pymavlink silent failure.

---

## Firmware — RP2040 pin assignments

| Signal       | GPIO | Direction | Notes                              |
|--------------|------|-----------|------------------------------------|
| SBUS RX      | GP4  | IN        | Inverted 100kbaud 8E2 from HM30    |
| PPM OUT      | GP15 | OUT       | To motor controllers / ESC         |
| SX1278 MISO  | GP16 | IN        | SPI0                               |
| SX1278 SCK   | GP18 | OUT       | SPI0                               |
| SX1278 MOSI  | GP19 | OUT       | SPI0                               |
| SX1278 NSS   | GP17 | OUT       | Chip select (active LOW)           |
| SX1278 RST   | GP20 | OUT       | Hardware reset                     |
| SX1278 DIO0  | GP21 | IN        | TxDone (master) / RxDone (slave)   |

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
**Payload:** 9 × uint16_t = 18 bytes, little-endian, channels in µs.

---

## Mode logic (both RP2040s)

```
SBUS lost/timeout (200ms)     → EMERGENCY (neutral)
CH4 (SWA) < 1700              → EMERGENCY
CH8 (CH9) in (1250, 1750)     → RELAY (local neutral, slave sees packets)
CH5 (SWB) > 1700:
  + no Jetson heartbeat       → AUTO-NO-HB (neutral)
  + heartbeat but no cmd      → AUTO-TIMEOUT (neutral)
  + heartbeat + fresh cmd     → AUTONOMOUS (Jetson controls)
else                          → MANUAL (raw SBUS passthrough)
```

Slave additionally gates on CH9 rover-select: ignores packets when not in relay window.

---

## RP2040 ↔ Jetson serial protocol

```
RP2040 → Jetson:
  CH:1500,1500,1700,1500,1500,1500,1500,1500,1500 MODE:MANUAL   (every frame)
  [SBUS_OK] / [SBUS_LOST]                                        (on change)
  [RF_LINK_OK] / [RF_LINK_LOST]                                  (slave, on change)
  <HB:N+1>                                                       (heartbeat echo)

Jetson → RP2040:
  <HB:N>                 heartbeat (must arrive < 300 ms for AUTO to stay active)
  <J:c0,c1,c2,c3,c4,c5,c6,c7>   8-channel autonomous command (µs)
```

---

## Build commands

```bash
# Build ROS2 workspace (run on each Jetson)
cd C:/ros_agri_rover/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Launch (run the appropriate one on each Jetson)
ros2 launch agri_rover_bringup rover1.launch.py   # master Jetson
ros2 launch agri_rover_bringup rover2.launch.py   # slave Jetson

# Build firmware (run on dev machine with Pico SDK installed)
cd firmware/rc_link_sx1278/master
cmake -B build -DPICO_SDK_PATH=$PICO_SDK_PATH && cmake --build build

# Monitor (dev machine, both rovers on WiFi)
python tools/monitor.py

# Upload a mission
python tools/mission_uploader.py missions/field.csv --rover 1 --host 192.168.1.10
```

---

## Config files

| File                                              | What to edit                              |
|---------------------------------------------------|-------------------------------------------|
| `ros2_ws/src/agri_rover_bringup/config/rover1_params.yaml` | Ports, IPs, speeds for RV1   |
| `ros2_ws/src/agri_rover_bringup/config/rover2_params.yaml` | Ports, IPs, speeds for RV2   |
| `ros2_ws/src/agri_rover_navigator/config/navigator_params.yaml` | Lookahead, speed limits |
| `ros2_ws/src/agri_rover_video/config/gstreamer.yaml` | Camera source, resolution, bitrate |

---

## Conventions

- **Python nodes:** rclpy, class inherits `Node`, parameters declared in `__init__`, timers not threads where possible.
- **Namespacing:** all topics are relative (no leading `/`), launch files set `namespace='/rv1'` or `'/rv2'`.
- **MAVLink:** always `os.environ['MAVLINK20'] = '1'` before importing pymavlink. Send via `socket.sendto()`, not `mav.write()` (silent failure bug — see memory).
- **PPM buffer:** only updated inside `dma_irq_handler()` (IRQ context). Main loop writes `out_ch[]` (volatile), IRQ reads it. No mutex needed on M0+ for uint16_t aligned writes.
- **SX1278 TX:** fire-and-forget. Check `RegIrqFlags.TxDone` before next TX. Returns `false` if still busy — caller skips that frame.
- **SBUS decode:** validate `frame[0]==0x0F && frame[24]==0x00` before accepting. Re-sync on every `0x0F` byte regardless of position.

---

## Android GQC — status

Scaffold only at `android/AgriRoverGQC/`. Not yet built. See `android/AgriRoverGQC/README.md`.
Planned stack: Kotlin, Jetpack Compose, java-mavlink, ExoPlayer (RTSP), OSMDroid.

---

## Known TODOs

- `sensor_node.py`: stub returns hardcoded values — wire real I2C sensors (BME280, ultrasonic tank)
- `firmware/*/main.cpp`: SX1278 driver is complete but needs `pico_sdk_import.cmake` copied from `$PICO_SDK_PATH/external/`
- `ppm_tx.pio`: slave uses SM0 (not SM1) — one less state machine needed vs master
- Android GQC: all screens need implementing (see README checklist)
- Navigator: no obstacle avoidance — pure pursuit only
