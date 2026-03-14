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
│       └── agri_rover_bringup/     Launch files + per-rover configs
├── firmware/
│   └── rc_link_sx1278/
│       ├── master/                 RP2040: SBUS decode → SX1278 TX
│       └── slave/                  RP2040: SX1278 RX → PPM out
├── android/
│   └── AgriRoverGQC/               Android GQC app (Kotlin)
├── tools/
│   ├── monitor.py                  Terminal dashboard (both rovers)
│   └── mission_uploader.py         CSV mission → MAVLink upload
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
| RC       | SBUS → SX1278 LoRa| 50 Hz | Manual control, safety-critical|
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

RP2040 firmware uses Pico SDK + SX1278 LoRa for inter-rover RC relay.
See `firmware/rc_link_sx1278/` — the SX1278 driver stubs must be completed
before flashing (use RadioLib or a custom SPI driver).

## Android GQC

See `android/AgriRoverGQC/README.md` for build instructions and TODO list.
"# ros_agri_rover" 
