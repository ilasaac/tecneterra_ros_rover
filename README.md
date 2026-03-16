# AgriRover ROS2 System

Dual autonomous ground rover platform with ROS2, MAVLink, and custom Android GQC.

## Repository Layout

```
ros_agri_rover/
├── ros2_ws/                        ROS2 colcon workspace
│   └── src/
│       ├── agri_rover_interfaces/  Custom msgs, srvs, actions
│       ├── agri_rover_rp2040/      RP2040 USB serial bridge node
│       ├── agri_rover_gps/         Dual-GPS NMEA driver node
│       ├── agri_rover_mavlink/     MAVLink ↔ ROS2 bridge node
│       ├── agri_rover_navigator/   Autonomous mission follower node
│       ├── agri_rover_sensors/     Agricultural sensor node
│       ├── agri_rover_video/       GStreamer RTSP streamer node
│       ├── agri_rover_simulator/   Dead-reckoning GPS simulator node
│       └── agri_rover_bringup/     Launch files + per-rover configs
├── firmware/
│   └── rc_link_sx1278/
│       ├── master/                 RP2040: SBUS decode → SX1278 TX
│       └── slave/                  RP2040: SX1278 RX → PPM out
├── android/
│   └── AgriRoverGQC/               Android GQC app (Kotlin)
├── tools/
│   ├── monitor.py                  Terminal MAVLink dashboard (both rovers)
│   ├── mission_uploader.py         CSV mission → MAVLink upload
│   ├── simulator.py                Standalone GPS simulator (no ROS2 required)
│   ├── nmea_wifi_rx.py             WiFi→PTY bridge for rover Jetsons
│   └── rtk_forwarder.py            NTRIP/E610 RTK corrections → u-blox serial
└── docs/
```

## Communication Architecture

```
[MK32] ──RF──► [HM30 Air Unit] ──SBUS──► [RP2040 Master] ──SX1278 LoRa──► [RP2040 Slave]
                                                │                                  │
                                          USB serial                          USB serial
                                                │                                  │
                                         [Jetson RV1]                       [Jetson RV2]
                                          ROS2 nodes                         ROS2 nodes
                                                │                                  │
                                          WiFi (MAVLink UDP, RTSP, ROS2 DDS)      │
                                                └──────────────────────────────────┘
                                                                │
                                                      [Android GQC App]
```

| Layer    | Protocol          | Rate  | Purpose                        |
|----------|-------------------|-------|-------------------------------|
| RC       | SBUS → SX1278 LoRa| 25 Hz | Manual control, safety-critical|
| MAVLink  | UDP/WiFi          | 10 Hz | Telemetry, commands, missions  |
| Video    | RTSP/WiFi         | 30fps | Camera feeds                   |
| ROS2     | DDS/WiFi          | varies| Autonomy, sensor fusion        |

## Quick Start

### Build ROS2 workspace (on Jetson)
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch Rover 1 (Master)
```bash
ros2 launch agri_rover_bringup rover1.launch.py
```

### Launch Rover 2 (Slave)
```bash
ros2 launch agri_rover_bringup rover2.launch.py
```

### Monitor from dev machine
```bash
python tools/monitor.py
```

### Upload a mission
```bash
python tools/mission_uploader.py missions/field_a.csv --rover 1 --host 192.168.1.10
```

---

## Development Simulation (no real GPS hardware)

A third Jetson Orin Nano runs a dead-reckoning GPS simulator that replaces the
four u-blox ZED-X20P modules during indoor development.

### Architecture

```
[RP2040 ppm_decoder] ──USB──► [Simulator Jetson (Orin Nano)]
                                    simulator.py / simulator_node
                                    │                │
                              UDP :5000/:5001   UDP :5000/:5001
                              (RV1 primary/sec)  (RV2 primary/sec)
                                    │                │
                              [RV1 Jetson]      [RV2 Jetson]
                              nmea_wifi_rx.py   nmea_wifi_rx.py
                              /dev/pts/2 (pri)  /dev/pts/2 (pri)
                              /dev/pts/3 (sec)  /dev/pts/3 (sec)
                                    │                │
                              gps_driver.py     gps_driver.py
```

The simulator reads PPM channel values for both rovers from a shared RP2040
ppm_decoder, runs bicycle-kinematic dead reckoning at 10 Hz, and sends
`$GNGGA` + `$GNVTG` sentences over UDP WiFi. Each rover Jetson receives
those sentences and exposes them as virtual serial ports (PTY) that
`gps_driver.py` reads without any modification.

### Startup order

**Step 1 — Rover Jetsons** (run before bringup):
```bash
# On each rover Jetson — receives UDP NMEA and creates /dev/pts/N virtual ports
python3 tools/nmea_wifi_rx.py --pri-port 5000 --sec-port 5001
# Note the printed PTY paths, e.g. /dev/pts/2 and /dev/pts/3
```

**Step 2 — Configure gps_driver** with the printed PTY paths:
```yaml
# ros2_ws/src/agri_rover_bringup/config/rover1_params.yaml
gps_driver:
  ros__parameters:
    primary_port:   /dev/pts/2
    secondary_port: /dev/pts/3
```

**Step 3 — Simulator Jetson**:
```bash
# Standalone (no ROS2/Isaac ROS needed)
python3 tools/simulator.py \
    --rv1-ip 192.168.1.10 --rv2-ip 192.168.1.20 \
    --ppm-port /dev/ttyACM0

# OR as a ROS2 node (edit config/simulator_params.yaml first)
ros2 launch agri_rover_simulator simulator.launch.py
```

**Step 4 — Rover Jetsons**:
```bash
ros2 launch agri_rover_bringup rover1.launch.py   # on RV1 Jetson
ros2 launch agri_rover_bringup rover2.launch.py   # on RV2 Jetson
```

### Simulator physics model

```
speed  = (throttle_ppm - 1500) / 500 × max_speed_mps
steer  = (steering_ppm - 1500) / 500          # –1..+1
omega  = speed × steer / wheelbase_m          # rad/s
heading += omega × dt
lat    += speed × cos(heading) × dt / 111320
lon    += speed × sin(heading) × dt / (111320 × cos(lat_rad))
```

Secondary GPS position is offset `antenna_baseline_m` ahead of primary along
the rover heading, so `gps_driver.py` recovers heading from the baseline vector
exactly as it does with real hardware.

---

## RTK Corrections

In real hardware deployments, RTK RTCM3 corrections are forwarded to the
u-blox ZED-X20P modules on each rover so they achieve RTK Fixed accuracy (~2 cm).

### Sources

| Source        | How                              | When to use                        |
|---------------|----------------------------------|------------------------------------|
| NTRIP caster  | Internet/WiFi (e.g. Emlid Caster)| Nearest base station is far away   |
| E610 DTU      | Ethernet port on rover Jetson    | Dedicated local base station       |

### Running `rtk_forwarder.py`

```bash
# On each rover Jetson — forwards corrections to both u-blox modules

# From NTRIP caster (Emlid Caster example):
python3 tools/rtk_forwarder.py \
    --source ntrip \
    --ntrip-host caster.emlid.com --ntrip-port 2101 \
    --mountpoint MY_BASE \
    --username user --password pass \
    --rv1-pri-port /dev/ttyUSB0 --rv1-sec-port /dev/ttyUSB1

# From E610 DTU (local base station over Ethernet):
python3 tools/rtk_forwarder.py \
    --source e610 \
    --e610-host 192.168.1.200 --e610-port 9000 \
    --rv1-pri-port /dev/ttyUSB0 --rv1-sec-port /dev/ttyUSB1

# Include approximate rover coordinates only for VRS/network RTK casters
# (standard single-base casters like Emlid Caster do NOT need this):
python3 tools/rtk_forwarder.py ... --approx-lat 40.1234 --approx-lon -3.5678

# Write a log file:
python3 tools/rtk_forwarder.py ... --log-file /tmp/rtk.log
```

### NTRIP protocol notes

- Use **HTTP/1.0** for the GET request — Emlid Caster and most single-base
  NTRIP v1 casters respond with `ICY 200 OK\r\n` and start streaming
  immediately.
- **Do not send GGA to a single-base caster** — they don't need rover position
  and any data sent by the client after the initial GET disrupts the stream.
  Only send GGA (`--approx-lat`/`--approx-lon`) when connecting to a VRS or
  network RTK caster that requires rover position.
- To verify NTRIP connectivity independently, install RTKLIB and run:
  ```bash
  str2str -in ntrip://user:pass@caster.emlid.com:2101/MY_BASE -out null://
  ```
  If `str2str` works but the script does not, compare the raw HTTP request
  bytes — the most common issue is GGA being sent when it shouldn't be.

## ROS2 Topic Map (per rover, namespace /rv1 or /rv2)

| Topic              | Type            | Producer      | Consumers             |
|--------------------|-----------------|---------------|-----------------------|
| ~/rc_input         | RCInput         | rp2040_bridge | mavlink_bridge        |
| ~/mode             | String          | rp2040_bridge | navigator, mavlink    |
| ~/cmd_override     | RCInput         | navigator/mavlink | rp2040_bridge     |
| ~/fix              | NavSatFix       | gps_driver    | navigator, mavlink    |
| ~/heading          | Float32         | gps_driver    | navigator, mavlink    |
| ~/rtk_status       | String          | gps_driver    | mavlink_bridge        |
| ~/sensors          | SensorData      | sensor_node   | mavlink_bridge        |
| ~/mission          | MissionWaypoint | mavlink_bridge| navigator             |

## MAVLink IDs

| Component  | sysid | UDP port | Video RTSP          |
|------------|-------|----------|---------------------|
| RV1 Jetson | 1     | :14550   | :8554/stream        |
| RV2 Jetson | 2     | :14551   | :8555/stream        |
| Android GQC| 255   | ephemeral| consumer            |

## Firmware

RP2040 firmware uses Pico SDK, custom PIO state machines, and an SX1278 LoRa driver
for inter-rover RC relay at 25 Hz. See `firmware/rc_link_sx1278/`.

**Master** (RV1): decodes SBUS from HM30 air unit → transmits raw 16-channel payload
over LoRa → outputs PPM locally via DMA+PIO → streams channel data to Jetson over USB CDC.

**Slave** (RV2): receives LoRa packet → applies PPM channel map → outputs PPM via DMA+PIO
→ streams channel data to Jetson over USB CDC.

**CH9 rover-select (RELAY mode):** CH9 centred (1250–1750 µs) → master PPM goes neutral,
slave receives and acts on raw RC stick values. CH9 outside that window → slave ignores
all RF packets (MODE_EMERGENCY, neutral output).

## Android GQC

See `android/AgriRoverGQC/README.md` for build instructions and TODO list.
