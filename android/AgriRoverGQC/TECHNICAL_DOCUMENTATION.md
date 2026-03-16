# AgriRoverGCS — Technical Documentation

## Overview

AgriRoverGCS is a Kotlin/Android ground station application for the AgriRover tracked robot.
It communicates with the rover's onboard Raspberry Pi controller (`rover_controller/`) over
MAVLink 2.0. The link is carried over the **SIYI RF radio system**: the Android device connects
to the SIYI MK32 Wi-Fi, exchanges MAVLink packets with the SIYI ground unit
(`192.168.144.11:14550`), and the ground unit relays them to the SIYI air unit on the rover
via the RF link. The rover and the Android device do **not** need to be on the same IP network.

The app presents a Google Maps interface for manual driving, mission planning, and autonomous
mission execution.

---

## Project Structure

```
AgriRoverGCS/
├── app/
│   ├── src/main/
│   │   ├── AndroidManifest.xml
│   │   ├── java/com/siyi/agrirover/
│   │   │   ├── MainActivity.kt          ← UI controller and mode state machine
│   │   │   └── RoverPositionManager.kt  ← MAVLink UDP network layer
│   │   └── res/
│   │       ├── layout/
│   │       │   └── activity_main.xml    ← Single-activity full-screen layout
│   │       ├── drawable/                ← ic_rover and icon assets
│   │       └── values/
│   │           ├── colors.xml
│   │           ├── strings.xml
│   │           └── themes.xml
│   └── build.gradle.kts
├── TECHNICAL_DOCUMENTATION.md  ← this file
└── USER_MANUAL.md
```

---

## Build Configuration

**File:** `app/build.gradle.kts`

| Setting | Value |
|---|---|
| `applicationId` | `com.siyi.agrirover` |
| `minSdk` | 28 (Android 9 Pie) |
| `targetSdk` / `compileSdk` | 36 |
| Language | Kotlin |
| Orientation | Landscape (locked via `AndroidManifest.xml`) |

### Key Dependencies

| Library | Version | Purpose |
|---|---|---|
| `com.google.android.gms:play-services-maps` | 19.0.0 | Google Maps satellite view |
| `io.dronefleet.mavlink:mavlink` | 1.1.9 | MAVLink 2.0 message encode/decode |
| `org.jetbrains.kotlinx:kotlinx-coroutines-android` | 1.7.3 | Async network I/O |
| `androidx.appcompat` | — | AppCompatActivity, AlertDialog |
| `com.google.android.material` | — | FloatingActionButton |

### Required Permissions (`AndroidManifest.xml`)

| Permission | Reason |
|---|---|
| `INTERNET` | UDP socket to rover |
| `ACCESS_NETWORK_STATE` | Network availability checks |
| `WRITE_EXTERNAL_STORAGE` | Save mission CSV files |
| `READ_EXTERNAL_STORAGE` | Load mission CSV files |

---

## Architecture

### Network topology

```
Android device (AgriRoverGCS)
│
│  Wi-Fi: connected to SIYI MK32 access point
│  UDP socket  →  192.168.144.11 : 14550
│
▼
SIYI MK32 ground unit  (192.168.144.11)
│
│  SIYI proprietary RF link (900 MHz or 2.4 GHz)
│
▼
SIYI air unit on rover
│
│  UART 57600 baud  →  /dev/ttyAMA0
│
▼
rover_controller/ on Pi / Jetson Nano
```

MAVLink packets are bridged transparently across the RF link. The Android app sends and
receives standard MAVLink 2.0 UDP packets; the SIYI system handles all RF framing and
routing internally.

### Software architecture

```
MainActivity.kt
│
│  creates
│  ├── GoogleMap            (satellite imagery, markers, polylines)
│  └── RoverPositionManager (MAVLink UDP comms, callbacks → Main thread)
│
│  state machine
│  ├── AppMode.MANUAL   → shows virtualJoystickLayout
│  ├── AppMode.PLANNER  → shows plannerToolbar, enables touch drawing
│  └── AppMode.AUTO     → shows autoToolbar
│
│  per-rover data (keyed by MAVLink system ID)
│  ├── roverPositions   HashMap<Int, LatLng>
│  ├── roverModes       HashMap<Int, AppMode>
│  ├── roverArmed       HashMap<Int, Boolean>
│  ├── roverMissions    HashMap<Int, List<LatLng>>
│  └── roverMarkers     HashMap<Int, Marker>

RoverPositionManager.kt
│
│  coroutines (Dispatchers.IO, SupervisorJob)
│  ├── receiveLoop()    — DatagramSocket.receive() → dispatchMessage()
│  ├── heartbeatLoop()  — sends GCS HEARTBEAT to targetIp every 1 000 ms
│  └── watchdogLoop()   — checks roverLastHb every 1 000 ms, 3 s timeout
│
│  callbacks → Dispatchers.Main
│  ├── onPositionUpdate(sysId, lat, lon, hdg)    GLOBAL_POSITION_INT
│  ├── onMissionAck(message)                      MISSION_ACK
│  ├── onSensorUpdate(bat, tank, temp, pressure)  SYS_STATUS / custom
│  ├── onArmState(sysId, armed)                   HEARTBEAT base_mode bit 7
│  ├── onMissionProgress(sysId, seq)              MISSION_CURRENT
│  ├── onCommandAck(sysId, cmd, result)           COMMAND_ACK
│  └── onConnectionChange(sysId, connected)       watchdog
```

---

## Class Reference

### `MainActivity`

**Package:** `com.siyi.agrirover`
**Extends:** `AppCompatActivity`, `OnMapReadyCallback`

#### Enums

```kotlin
enum class AppMode    { MANUAL, PLANNER, AUTO }
enum class PlannerTool { NONE, ADD, DELETE, SET_WATER, SET_BATTERY }
```

#### State fields

| Field | Type | Description |
|---|---|---|
| `currentMode` | `AppMode` | Active UI mode |
| `currentTool` | `PlannerTool` | Active planner tool |
| `selectedRoverId` | `Int` | Which rover receives commands (default 1) |
| `roverMarkers` | `HashMap<Int, Marker>` | Map marker per rover system ID |
| `roverPositions` | `HashMap<Int, LatLng>` | Last known GPS position per rover |
| `roverModes` | `HashMap<Int, AppMode>` | GCS-side mode tracking per rover |
| `roverArmed` | `HashMap<Int, Boolean>` | Arm state received from HEARTBEAT |
| `roverMissions` | `HashMap<Int, List<LatLng>>` | Uploaded waypoint list per rover |
| `roverMissionVisible` | `HashMap<Int, Boolean>` | Whether each rover's mission overlays are shown |
| `routePoints` | `ArrayList<LatLng>` | Waypoints in the active planner drawing |
| `nextWaypointIndex` | `Int` | Current waypoint index from MISSION_CURRENT telemetry |
| `waterPoint` | `LatLng?` | Water refill station marker (persisted) |
| `batteryPoint` | `LatLng?` | Battery swap station marker (persisted) |
| `isFollowingRover` | `Boolean` | Camera follows selected rover when true |

#### Key methods

| Method | Trigger | Description |
|---|---|---|
| `onCreate()` | Activity start | Binds views, restores saved IP, starts `roverManager.startListening()` |
| `onDestroy()` | Activity stop | Calls `roverManager.stopListening()` |
| `onMapReady()` | Maps SDK callback | Configures satellite view, touch overlay, camera follow |
| `setupListeners()` | `onCreate` | Wires all button click/touch listeners |
| `setupVirtualJoystick()` | `onCreate` | Configures four D-pad buttons as a 10 Hz joystick sender |
| `showConnectionDialog()` | `btnConnect` | AlertDialog with IP EditText; calls `setTargetAddress()` |
| `updateConnectionDot()` | `onConnectionChange` callback | Sets `txtConnection` colour (green/red) |
| `setMode(AppMode)` | Mode menu | Hides/shows toolbar panels |
| `redrawMap()` | Route edits, `onMissionProgress` | Clears and redraws polylines and WP markers |
| `updateRover()` | `onPositionUpdate` callback | Creates/moves Google Maps marker; calls `createRoverBitmap()` |
| `createRoverBitmap()` | `updateRover` | Draws rover icon with tinted body + status dot + selection ring |
| `uploadMission()` | `btnUpload` | Copies `routePoints` to `roverMissions[id]`, calls `roverManager.uploadMission()` |
| `saveMissionFile()` | Save dialog | Writes `Latitude,Longitude\n…` CSV to external storage |
| `parseCSV()` | Load dialog | Reads CSV, populates `routePoints`, calls `redrawMap()` |

#### Rover icon colours

| Body tint | Rover ID |
|---|---|
| Red | System ID 1 |
| Blue | System ID 2 |

| Centre dot colour | State |
|---|---|
| Green | Disarmed, manual |
| Orange | Armed, manual |
| Yellow | AUTO mode |

| Ring | Meaning |
|---|---|
| White ring | Currently selected rover |
| No ring | Not selected |

#### Waypoint marker colours

| Marker colour | Meaning |
|---|---|
| Red | Not yet reached |
| Cyan | Current active target (from MISSION_CURRENT) |
| Green polyline | Already-passed segment |
| Red polyline | Remaining route |

#### Persisted data (`SharedPreferences`)

| Key | Value |
|---|---|
| `rover_ip` | Last successfully connected Pi IP address |
| `w_lat` / `w_lon` | Water station latitude / longitude |
| `b_lat` / `b_lon` | Battery station latitude / longitude |

---

### `RoverPositionManager`

**Package:** `com.siyi.agrirover`

Single class that owns the UDP socket and all MAVLink encode/decode logic.
Runs entirely on `Dispatchers.IO`; all callbacks are dispatched back to `Dispatchers.Main`.

#### Constructor parameters (all lambdas)

| Parameter | Signature | Triggered by |
|---|---|---|
| `onPositionUpdate` | `(sysId: Int, lat: Double, lon: Double, hdg: Float) -> Unit` | `GLOBAL_POSITION_INT` (#33) |
| `onMissionAck` | `(message: String) -> Unit` | `MISSION_ACK` (#47) |
| `onSensorUpdate` | `(bat: Int, tank: Float, temp: Float, press: Float) -> Unit` | `SYS_STATUS`, `SCALED_PRESSURE`, `NAMED_VALUE_FLOAT` |
| `onArmState` | `(sysId: Int, armed: Boolean) -> Unit` | `HEARTBEAT` base_mode bit 7 |
| `onMissionProgress` | `(sysId: Int, seq: Int) -> Unit` | `MISSION_CURRENT` (#42) |
| `onCommandAck` | `(sysId: Int, cmd: Int, result: Int) -> Unit` | `COMMAND_ACK` (#77) |
| `onConnectionChange` | `(sysId: Int, connected: Boolean) -> Unit` | Watchdog: 3 s heartbeat timeout |

#### Public methods

| Method | Description |
|---|---|
| `setTargetAddress(ip, port)` | Sets the rover Pi's IP. Saved in `targetIp`. Heartbeat loop starts sending to this address immediately. |
| `startListening()` | Creates `DatagramSocket(14550)`, launches three coroutines: `receiveLoop`, `heartbeatLoop`, `watchdogLoop`. Safe to call from main thread. |
| `stopListening()` | Sets `isRunning = false`, closes socket, cancels coroutine scope. |
| `sendManualControl(sysId, x, y, z, r, buttons)` | Encodes and sends `MANUAL_CONTROL` (#69). `x` = throttle, `y` = steering (−1000…+1000). |
| `sendCommand(sysId, commandId, p1, p2)` | Encodes and sends `COMMAND_LONG` (#76). |
| `uploadMission(sysId, waypoints)` | Starts the MAVLink mission upload handshake: sends `MISSION_COUNT`, then serves `MISSION_ITEM_INT` items on demand when `MISSION_REQUEST_INT` arrives. |

#### Network details

| Property | Value |
|---|---|
| Transport | UDP |
| Listen port | 14550 |
| GCS source system ID | 255 |
| GCS source component | 0 |
| MAVLink version | 2 (via `connection.send2()`) |
| Buffer size | 2 048 bytes per datagram |

#### Address discovery

`targetIp` starts as `null`. It is set in one of two ways:
1. **Manual** — user enters the Pi's IP via the Connect dialog → `setTargetAddress()`.
2. **Auto-discover** — if `targetIp` is still `null` when the first datagram arrives, the sender's address is stored.

The heartbeat loop only sends when `targetIp != null`, so the connection is always GCS-initiated when an IP is configured.

---

## MAVLink Protocol

### GCS → Rover (sent by this app)

| ID | Message | When sent | Key fields |
|---|---|---|---|
| #0 | `HEARTBEAT` | Every 1 000 ms | `type=6` (GCS), `system_status=4` (ACTIVE) |
| #44 | `MISSION_COUNT` | On upload start | `count` = number of waypoints |
| #73 | `MISSION_ITEM_INT` | On each `MISSION_REQUEST_INT` | `frame=6` (GLOBAL_RELATIVE_ALT_INT), `command=16` (NAV_WAYPOINT), `x/y` = lat/lon × 1e7 |
| #69 | `MANUAL_CONTROL` | Every 100 ms (joystick active) | `x` = throttle, `y` = steering (−1000…+1000) |
| #76 | `COMMAND_LONG` | Button press | See command table below |

### Rover → GCS (received and displayed by this app)

| ID | Message | Callback triggered | What it updates |
|---|---|---|---|
| #0 | `HEARTBEAT` | `onArmState`, `onConnectionChange` | Arm dot colour, `txtConnection` |
| #1 | `SYS_STATUS` | `onSensorUpdate` | `txtBattery` |
| #33 | `GLOBAL_POSITION_INT` | `onPositionUpdate` | Rover map marker position and heading |
| #42 | `MISSION_CURRENT` | `onMissionProgress` | `nextWaypointIndex`, cyan WP marker |
| #47 | `MISSION_ACK` | `onMissionAck` | Toast ("Upload Complete!" or error) |
| #62 | `NAV_CONTROLLER_OUTPUT` | — | Not yet wired to UI |
| #77 | `COMMAND_ACK` | `onCommandAck` | Toast on non-zero result |
| — | `SCALED_PRESSURE` | `onSensorUpdate` | `txtTemp` |
| — | `NAMED_VALUE_FLOAT` ("TANK") | `onSensorUpdate` | `txtTank` |

### COMMAND_LONG commands sent

| MAV_CMD | ID | param1 | param2 | Button |
|---|---|---|---|---|
| `MAV_CMD_COMPONENT_ARM_DISARM` | 400 | `1` = arm | `0` | START (step 1) |
| `MAV_CMD_COMPONENT_ARM_DISARM` | 400 | `0` = disarm | `0` | E-STOP |
| `MAV_CMD_DO_SET_MODE` | 176 | `1` | `3` = AUTO | START (step 2, 500 ms after arm) |
| `MAV_CMD_DO_SET_MODE` | 176 | `1` | `0` = MANUAL | PAUSE |

### Mission upload sequence

```
GCS  ──MISSION_COUNT(N)─────────────────────────► Rover Pi
Rover ──MISSION_REQUEST_INT(seq=0)──────────────► GCS
GCS  ──MISSION_ITEM_INT(seq=0, lat, lon, …)─────► Rover Pi
Rover ──MISSION_REQUEST_INT(seq=1)──────────────► GCS
      … repeated for each waypoint …
Rover ──MISSION_ACK(result=0  ACCEPTED)─────────► GCS
GCS  displays toast "Upload Complete!"
```

If the rover replies `MISSION_ACK` with a non-zero result, the toast shows the numeric error code.

---

## UI Layout (`activity_main.xml`)

The layout is a single `ConstraintLayout` in **landscape** orientation.

```
┌────────────────────────────────────────────────────────────────────────────┐
│ [MODE ▼]  [space]  [CONNECT]  [E-STOP]  [space]  [●] B:xx% T:xx% C:xx°  │  topBar
├────────────────────────────────────────────────────────────────────────────┤
│                                                            [🗺] (layers)   │
│                   Google Maps (full screen)                [📍] (follow)   │
│                                                                            │
│ [▲]                                                                        │  virtualJoystickLayout
│[◀][▶]   (MANUAL mode only)                                                │
│ [▼]                                                                        │
├────────────────────────────────────────────────────────────────────────────┤
│ [+] [🗑] [💧] [💾] [📂] [CLEAR]  [UPLOAD]          (PLANNER mode only)  │  plannerToolbar
│  or                                                                        │
│ [R1] [R2] [START] [PAUSE] [CLEAR]                   (AUTO mode only)     │  autoToolbar
└────────────────────────────────────────────────────────────────────────────┘
```

### View ID reference

| ID | Type | Description |
|---|---|---|
| `map` | `SupportMapFragment` | Full-screen Google Maps |
| `touchOverlay` | `View` | Transparent overlay capturing touch during planner drawing |
| `topBar` | `LinearLayout` | Always-visible status/control bar |
| `btnModeMenu` | `Button` | Opens mode popup (Manual / Planner / Auto) |
| `btnConnect` | `Button` | Opens Pi IP address dialog |
| `btnEStop` | `Button` | Broadcasts DISARM to all rovers immediately |
| `txtConnection` | `TextView` | `●` dot: orange=no IP set, green=connected, red=HB lost |
| `txtBattery` | `TextView` | Battery % from SYS_STATUS |
| `txtTank` | `TextView` | Tank level % from NAMED_VALUE_FLOAT "TANK" |
| `txtTemp` | `TextView` | Temperature °C from SCALED_PRESSURE |
| `btnLayers` | `FloatingActionButton` | Toggle satellite ↔ standard map |
| `btnCenter` | `FloatingActionButton` | Toggle camera follow mode |
| `plannerToolbar` | `LinearLayout` | Bottom bar, PLANNER mode |
| `btnAddPoint` | `FloatingActionButton` | Activate draw-route tool |
| `btnDelPoint` | `FloatingActionButton` | Activate delete-point tool |
| `btnRecord` | `FloatingActionButton` | Station menu (water / battery) |
| `btnSave` | `FloatingActionButton` | Save route to CSV |
| `btnLoad` | `FloatingActionButton` | Load route from CSV |
| `btnClearMap` | `Button` | Clear all drawn waypoints |
| `btnUpload` | `Button` | Upload route to selected rover |
| `autoToolbar` | `LinearLayout` | Bottom bar, AUTO mode |
| `btnToggleR1` | `Button` | Show/hide Rover 1 mission overlays |
| `btnToggleR2` | `Button` | Show/hide Rover 2 mission overlays |
| `btnStart` | `Button` | Arm selected rover + set AUTO mode |
| `btnStop` | `Button` | Set selected rover to MANUAL (pause) |
| `btnClear` | `Button` | Clear route + upload empty mission |
| `virtualJoystickLayout` | `RelativeLayout` | D-pad, MANUAL mode |
| `btnUp/Down/Left/Right` | `Button` | Throttle / steering inputs |

---

## Mission File Format

Missions are saved as plain CSV to the app's external files directory
(`/sdcard/Android/data/com.siyi.agrirover/files/`).

```
Latitude,Longitude
34.266079,-119.081079
34.266200,-119.081100
34.266350,-119.081050
```

The file extension is always `.csv`. Files are listed in the Load dialog.

---

## Known Limitations

| Limitation | Impact |
|---|---|
| Single UDP socket shared by all rovers | Two rovers on different IPs both receive all outbound messages, but the app sends to only one target IP (the one set in Connect dialog or auto-discovered first). For multi-rover with different Pi IPs, only one connection is active at a time. |
| `MISSION_REQUEST_INT` handling ignores sequence errors | If the rover re-requests a sequence out of order (e.g. on partial upload retry), the app serves whatever seq is requested; it does not restart the upload. |
| `NAV_CONTROLLER_OUTPUT` not displayed | Bearing and distance to waypoint are received but not wired to any UI element. |
| No IMU / ATTITUDE display | Roll, pitch, yaw data is not used even if the rover sends it. |
| Mission upload does not set `z` (altitude) | `z = 0f` is sent; the rover controller ignores altitude for ground navigation. |
| Storage permissions | On Android 10+ scoped storage applies; files are written to app-scoped external storage so no permission dialog is shown, but files are not visible in generic file managers without developer access. |
