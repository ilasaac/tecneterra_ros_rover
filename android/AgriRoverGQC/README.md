# AgriRover GQC — Android Ground Control App

Custom Android GCS for the dual-rover AgriRover system.

## Stack

- **Language:** Kotlin
- **UI:** XML layouts + AppCompat (no Jetpack Compose)
- **MAVLink:** [java-mavlink / DroneFleet](https://github.com/DroneFleet/mavlink)
- **Map:** Google Maps SDK (satellite default, Jalisco field origin)

## Architecture

```
MainActivity
  └── RoverPositionManager   (all MAVLink UDP I/O on Dispatchers.IO)
        ├── heartbeatLoop()  1 Hz GCS heartbeat → broadcast :14550
        ├── watchdogLoop()   3 s rover disconnection detection
        └── receiveLoop()    parse incoming MAVLink, dispatch callbacks
```

## Network

- App binds UDP **:14550** — same port used by both rovers (different Jetsons).
- Broadcasts GCS heartbeat to **255.255.255.255:14550** at 1 Hz.
- Rover IPs are **auto-discovered** from the first inbound packet of each sysid.
- GQC identifies rovers by **sysid in HEARTBEAT** (RV1=1, RV2=2), not by port.

## App Modes

| Mode | Toolbar | Notes |
|------|---------|-------|
| MANUAL | (none visible) | Physical RC sticks active. App shows map + telemetry only. |
| PLANNER | FABs, ADD, REC, CLEAR, UPLOAD | Finger-draw or record route on satellite map; upload mission with optional servo commands. |
| AUTO | R1/R2 toggle, START, PAUSE, CLEAR | ARM + AUTO mode; waypoint progress shown on map. |

## Key behaviours

- **Screen always on:** `FLAG_KEEP_SCREEN_ON` set in `onCreate`.
- **E-STOP:** broadcasts DISARM (COMMAND_LONG cmd=400 p1=0) **3× at 100 ms intervals** via `sendCriticalCommand` — a single dropped UDP packet must never prevent stopping.
- **Link mismatch detector:** if the RC switch state (SWA/SWB from master RC_CHANNELS) disagrees with slave HEARTBEAT for >2 s, the app auto-disarms all rovers and shows a blocking alert.
- **Mission upload:** standard MAVLink MISSION protocol (COUNT → REQUEST_INT → ITEM_INT handshake). Uses `uploadRecordedMission()` when servo commands are present (mixed `NAV_WAYPOINT` + `DO_SET_SERVO` items), or plain `uploadMission()` for waypoint-only routes.
- **Rover markers:** red=RV1, blue=RV2; centre dot green=disarmed / orange=armed / yellow=AUTO; white ring = selected rover.

## Mission recording (PLANNER mode)

**ADD** — inserts the rover's current GPS position as a single waypoint.

**REC / STOP** — continuous recording at 500 ms:
- Saves current rover GPS position every 500 ms
- Detects PPM CH5-CH8 switch changes > 100 µs and appends `ServoCmd(servo, pwm)` entries immediately

The recorded list is a `List<MissionAction>` (`Waypoint` or `ServoCmd`). On UPLOAD, if any servo commands are present the full interleaved sequence is uploaded; the rover applies servo commands as it receives them.

## MAVLink commands sent

| Command | ID | Params | Used for |
|---------|----|--------|---------|
| MAV_CMD_COMPONENT_ARM_DISARM | 400 | p1=1 arm / p1=0 disarm | START, E-STOP |
| MAV_CMD_DO_SET_MODE | 176 | p2=0 manual / p2=3 auto | START (AUTO), PAUSE (MANUAL) |
| MAV_CMD_DO_SET_SERVO | 183 | p1=servo(5-8), p2=pwm µs | Servo/pump control in recorded missions |

## Build

1. Open `android/AgriRoverGQC/` in Android Studio Hedgehog or later.
2. Add a Google Maps API key to `app/src/main/res/values/secrets.xml` (see Maps SDK docs).
3. `./gradlew assembleDebug`

## TODO

- [ ] ExoPlayer RTSP dual video feed screen
- [ ] Per-rover settings dialog (IP override, video quality)
- [ ] Alert/STATUSTEXT log screen
- [ ] Wire real I2C sensors to SensorData (BME280 + tank)
