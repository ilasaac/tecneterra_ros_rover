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
- **WiFi lock:** `WIFI_MODE_FULL_LOW_LATENCY` (API 29+) WifiLock acquired on `startListening()`, falling back to `WIFI_MODE_FULL_HIGH_PERF` on older devices. Prevents Android WiFi power-save from batching UDP packets. Released on `stopListening()`. `WIFI_MODE_FULL_HIGH_PERF` was deprecated in API 29 and silently less effective on Android 12+.

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

Recorded missions are a `List<MissionAction>`:

```kotlin
sealed class MissionAction {
    data class Waypoint(val lat: Double, val lon: Double,
                        val speed: Float = 0f,
                        val holdSecs: Float = 0f) : MissionAction()
    data class ServoCmd(val servo: Int, val pwm: Int)  : MissionAction()
}
```

**ADD** — inserts the selected rover's current GPS position as a single waypoint (speed and holdSecs = 0).

**REC / STOP** — continuous recording at 500 ms intervals:
- Movement threshold `MIN_RECORD_DIST = 0.3 m` — GPS jitter below this is ignored.
- When rover moves ≥ 0.3 m: flushes accumulated `pendingHoldSecs` into the **previous** waypoint's `holdSecs`, then records the new position with the current throttle PPM mapped to `speed` in m/s (`(ppm - 1500) / 500 × MAX_SPEED_MPS`).
- When rover is stationary: increments `pendingHoldSecs += 0.5 s` every tick.
- PPM CH5-CH8 monitoring: any channel change > 100 µs since last sample → appends `ServoCmd(servo, pwm)` immediately.
- **At recording start:** 4 `ServoCmd` items are prepended for the current state of CH5-CH8 so the rover always starts with known servo positions.

The result is an exact recording of the route: positions, speeds, servo switch events, and stationary waits — all replayed identically during autonomous navigation.

**CLEAR** — stops any active recording, clears all route points and the recorded mission list, and uploads an empty mission to the rover to cancel any in-progress autonomous route.

**UPLOAD** — if `recordedMission` contains any `ServoCmd` entries, calls `uploadRecordedMission()`; otherwise uses plain `uploadMission()`. Both funnel into `streamMission()`.

### MAVLink encoding of waypoint fields

| `MissionAction.Waypoint` field | `MISSION_ITEM_INT` field | Notes |
|---|---|---|
| `lat`, `lon` | `x`, `y` (×1e7 int) | standard NAV_WAYPOINT position |
| `speed` (m/s) | `z` | altitude field repurposed (ground vehicle) |
| `holdSecs` | `param1` | MAVLink hold time field |

The rover decodes these back in `mavlink_bridge._on_mission_item()` and populates `MissionWaypoint.speed` and `MissionWaypoint.hold_secs`.

## Autonomous navigation (rover side)

The navigator uses a **Stanley lateral controller**:
```
δ = θ_e + arctan(k × e_cte / v)
```
- Follows the recorded route at the exact recorded speed per segment.
- Corrects both heading error and cross-track (lateral) drift simultaneously.
- Replays `holdSecs` exactly: rover halts at the waypoint for the recorded duration before advancing.
- Cross-track error (XTE) is published as MAVLink `NAMED_VALUE_FLOAT 'XTE'` at 1 Hz for monitoring.

## Mission upload protocol — streaming

Both `uploadMission()` and `uploadRecordedMission()` call `streamMission()`:

```
GQC                          Rover
  ── MISSION_COUNT ──────────►
  (wait 150 ms)
  ── MISSION_ITEM_INT(0) ────►
  (wait 20 ms)
  ── MISSION_ITEM_INT(1) ────►
  ...
  ── MISSION_ITEM_INT(N-1) ──►
                              ◄── MISSION_ACK (when all received)
  (if item lost: rover retry sends REQUEST_INT after 250 ms, GQC replies from pendingMissions)
```

**Why streaming instead of request/response:**
- Per-item REQUEST_INT → ITEM_INT round trip hits Android WiFi DTIM (~100–150 ms each) because `WIFI_MODE_FULL_LOW_LATENCY` WifiLock is not fully honoured on all devices.
- Streaming sends all items to the AP in one burst; AP delivers them to the Jetson in a single DTIM window.
- Result: worst-case ~350 ms total for any mission size, vs 150 ms × N with request/response.

**Jetson WiFi power-save:** if not disabled, the AP buffers incoming `MISSION_ITEM_INT` for 100–900 ms per item. `start_rover{1,2}_sim.sh` runs `sudo iw dev <iface> set power_save off` automatically before each container start.

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
