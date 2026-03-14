# AgriRover GQC — Android Ground Control App

Custom Android ground control station for the AgriRover dual-rover system.

## Stack

- **Language:** Kotlin
- **MAVLink:** [java-mavlink](https://github.com/DroneFleet/mavlink) or hand-rolled UDP parser
- **Video:** ExoPlayer with RTSP source
- **Map:** OSMDroid (offline tiles) or Google Maps SDK
- **UI:** Jetpack Compose

## Screens

| Screen | Purpose |
|--------|---------|
| Dashboard | Split map showing both rovers, battery, mode, GPS fix |
| Video | Dual RTSP feeds (RV1 + RV2 side by side or switchable) |
| Control | Virtual joystick → RC_CHANNELS_OVERRIDE MAVLink |
| Mission | Tap-to-place waypoints on map → upload via MAVLink MISSION protocol |
| Alerts | STATUSTEXT log with severity colour coding |
| Settings | IPs, ports, video quality |

## MAVLink connections

| Rover | MAVLink sysid | UDP port | Video |
|-------|--------------|----------|-------|
| RV1   | 1            | :14550   | rtsp://rv1-ip:8554/stream |
| RV2   | 2            | :14551   | rtsp://rv2-ip:8555/stream |

The app sends to broadcast (192.168.1.255:14550) and listens on :14550/:14551.
Rovers are identified by sysid in HEARTBEAT.

## Build

1. Open in Android Studio Hedgehog or later
2. Set `local.properties` with SDK path
3. `./gradlew assembleDebug`

## TODO

- [ ] MAVLink UDP socket handler (dual-rover)
- [ ] ExoPlayer RTSP integration
- [ ] Virtual joystick composable
- [ ] OSMDroid map with dual rover markers
- [ ] Mission editor (tap to add waypoints)
- [ ] MAVLink MISSION upload protocol
- [ ] Alert notification system
