# AgriRoverGCS — User Manual

**Version 1.0 | SIYI MK32 Ground Station**

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Screen Layout](#2-screen-layout)
3. [First-Time Setup — Connecting to the Rover](#3-first-time-setup--connecting-to-the-rover)
4. [Status Bar Reference](#4-status-bar-reference)
5. [Manual Mode](#5-manual-mode)
6. [Mission Planner Mode](#6-mission-planner-mode)
7. [Auto Mode — Running a Mission](#7-auto-mode--running-a-mission)
8. [Emergency Stop](#8-emergency-stop)
9. [Map Controls](#9-map-controls)
10. [Saving and Loading Missions](#10-saving-and-loading-missions)
11. [Multi-Rover Operation](#11-multi-rover-operation)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. System Requirements

| Requirement | Detail |
|---|---|
| Device | SIYI MK32 ground station (Android tablet/handheld) |
| Android version | 9 (Pie) or newer |
| Link | SIYI air unit on rover + SIYI MK32 RF link (no shared IP network needed) |
| Rover controller | Pi running `rover_controller/main.py` in autopilot or manual mode |
| Internet | Required for Google Maps satellite tiles (first load) |

---

## 2. Screen Layout

The app runs in **landscape orientation** only.

```
┌────────────────────────────────────────────────────────────────────────────┐
│  [MODE ▼]  ··  [CONNECT]  [E-STOP]  ··  [●]  B:--% T:--% C:--°C         │
├────────────────────────────────────────────────────────────────────────────┤
│                                                              [🗺]           │
│                                                              [📍]           │
│                        MAP                                                 │
│                                                                            │
│  [▲]                                                                       │
│ [◀][▶]                                          (MANUAL mode joystick)    │
│  [▼]                                                                       │
├────────────────────────────────────────────────────────────────────────────┤
│  Toolbar area — changes depending on active mode (see sections 5, 6, 7)   │
└────────────────────────────────────────────────────────────────────────────┘
```

**Top bar (always visible):**

| Element | Description |
|---|---|
| `MODE ▼` | Tap to switch between Manual, Planner, and Auto modes |
| `CONNECT` | Tap to enter the rover Pi's IP address |
| `E-STOP` | Emergency stop — immediately disarms all rovers |
| `●` | Connection dot — shows rover link status |
| `B: xx%` | Rover battery percentage |
| `T: xx%` | Tank level percentage |
| `C: xx°C` | Temperature sensor reading |

**Right-side floating buttons (always visible):**

| Button | Function |
|---|---|
| 🗺 Map icon | Toggle between satellite and standard map view |
| 📍 Location icon | Toggle camera follow mode (follows selected rover) |

---

## 3. First-Time Setup — Connecting to the Rover

Communication between the app and the rover uses the **SIYI RF radio link**.
The rover Pi connects to the SIYI air unit via UART; the MK32 connects to the SIYI
ground unit over its internal Wi-Fi. MAVLink packets travel across the RF link
automatically — the rover and the Android device do **not** need to share the same IP network.

### Step 1 — Power on the SIYI hardware

1. Power on the **SIYI air unit** on the rover (it should be wired to the Pi's UART).
2. Power on the **SIYI MK32** ground station.
3. Confirm the RF link indicator on the MK32 shows a solid link (not blinking/disconnected).

### Step 2 — Start the rover controller on the Pi

SSH into the Raspberry Pi (via any available network — e.g. hotspot or direct ethernet)
and run:

```bash
# Autopilot mode (recommended for field use)
python rover_controller/main.py --mode autopilot

# Manual mode only (no mission support)
python rover_controller/main.py --mode manual
```

The rover controller opens a serial connection to the air unit at 57600 baud and waits
for a GCS heartbeat.

### Step 3 — Connect the MK32 to its internal Wi-Fi

On the Android device (or MK32 tablet):

1. Open **Wi-Fi Settings**.
2. Connect to the **SIYI MK32 access point** (SSID shown on the MK32 screen or listed in
   the SIYI manual for your hardware version).
3. The SIYI ground unit is reachable at **`192.168.144.11`** on this network.

### Step 4 — Connect the app

1. Open AgriRoverGCS.
2. The app pre-fills `192.168.144.11` as the default target IP.
   On first launch, or after clearing settings, tap **CONNECT** in the top bar.
3. The dialog shows two options:

   | Button | Action |
   |---|---|
   | **Connect** | Send to the IP shown in the text field (default `192.168.144.11`) |
   | **Auto-discover** | Clear the stored IP and wait for the ground unit to announce itself (first incoming packet sets the reply address) |

4. Tap **Connect** (or **Auto-discover** if you prefer).

The app immediately starts sending MAVLink heartbeats to the SIYI ground unit.
The ground unit relays them over RF to the air unit → Pi.
The rover controller receives the heartbeat, learns the GCS address, and starts sending telemetry.

### Step 5 — Confirm connection

Watch the **●** dot in the top bar:

| Dot colour | Meaning |
|---|---|
| Orange | Target IP configured, waiting for first rover heartbeat |
| Green `● R1` | Rover 1 heartbeat received — RF link active |
| Red `● LOST` | Heartbeat lost — check air unit power, RF link, and Pi process |

Once the dot turns green, the rover is connected and ready.

> **The IP address is saved automatically.** On the next app launch, the app
> connects to `192.168.144.11` immediately without showing the dialog.
> If the SIYI ground unit is at a different IP (custom setup), tap **CONNECT**
> to change it.

---

## 4. Status Bar Reference

### Connection dot (`●`)

| Colour | State |
|---|---|
| Orange | IP set, no heartbeat yet received |
| Green | Rover heartbeat received within the last 3 seconds |
| Red | Heartbeat lost — check network and Pi process |

### Rover map marker

The rover appears on the map as a **coloured arrow icon** that rotates with the rover's heading.

| Feature | Meaning |
|---|---|
| **Red body** | Rover with system ID 1 |
| **Blue body** | Rover with system ID 2 |
| **Green dot** in centre | Disarmed, manual mode |
| **Orange dot** in centre | Armed, manual mode |
| **Yellow dot** in centre | AUTO mode (mission running) |
| **White ring** around icon | This rover is currently selected |

### Battery / Tank / Temperature

These fields update automatically from the rover's telemetry:

- **B:** Battery remaining (%) from the rover's `SYS_STATUS` message.
- **T:** Tank level (%) sent by the rover as a custom `NAMED_VALUE_FLOAT "TANK"` message.
- **C:** Temperature in °C from the rover's pressure sensor.

All fields show `--` until the first data arrives.

---

## 5. Manual Mode

Manual mode lets you drive the selected rover using the on-screen D-pad.

### Activating Manual Mode

Tap **MODE ▼** → select **Manual Mode**.

A D-pad control appears in the **bottom-left corner** of the screen.

### Driving the Rover

| Button | Action |
|---|---|
| ▲ (Up) | Throttle forward |
| ▼ (Down) | Throttle reverse |
| ◀ (Left) | Steer left |
| ▶ (Right) | Steer right |

- **Press and hold** a button to continuously move.
- **Release** to send a zero command — the rover controller's spring-return
  logic brings throttle and steering back to neutral over ~200 ms.
- Diagonal motion is not currently supported — each button controls one axis.

### How it works

Each button press sends a `MANUAL_CONTROL` MAVLink message every **100 ms** while held.
The rover Pi receives it in `modes/manual.py` and converts it to UART PPM commands for the RP2040.

> **Tip:** The MK32's physical joysticks (if configured in a companion RC app) can also
> send `MANUAL_CONTROL` messages. The rover Pi prioritises MAVLink joystick input over
> its own keyboard fallback.

### Selecting a rover to drive

In Manual mode, all commands go to the **selected rover** (shown with a white ring on the map).
Tap a rover marker on the map to select it. A toast confirms the selection.

---

## 6. Mission Planner Mode

Mission Planner lets you draw a route on the map and upload it to a rover as a set of GPS waypoints.

### Activating Planner Mode

Tap **MODE ▼** → select **Mission Planner**.

The **Planner Toolbar** appears at the bottom of the screen:

```
[+] [🗑] [💧] [💾] [📂] [CLEAR] [UPLOAD]
```

### Drawing a Route

1. Tap the **+** (Add Point) button — it turns **green** when active.
2. **Draw on the map** by dragging your finger along the desired path.
   Points are recorded as you draw (minimum 2 m spacing between points to avoid
   redundant waypoints).
3. The route appears as a **red polyline** with red markers at each waypoint.
4. Tap **+** again to deactivate the draw tool.

### Deleting Points

1. Tap the **🗑** (Delete) button — it turns **red** when active.
2. Touch or drag near any waypoint marker to remove it.
3. Points within 5 metres of your touch are deleted.

### Setting Stations

Stations are fixed map markers that persist across sessions (saved to device storage).

1. Tap the **💧** (Station) button.
2. Choose **Set Water Station** or **Set Battery Station**.
3. Tap the map at the desired location.
4. The station marker appears:
   - Water station — cyan/blue marker
   - Battery station — green marker

### Uploading a Mission

Once your route is drawn:

1. Tap **UPLOAD**.
2. The app sends the waypoints to the **currently selected rover** using the MAVLink
   mission upload protocol.
3. A toast confirms: `"Uploading N WPs to Rover X…"`
4. When the rover acknowledges: `"Rover X: Upload Complete!"` is shown.

> **The rover does not start moving after upload.** You must switch to Auto Mode and
> press START to begin execution.

### Clearing the Route

- **CLEAR** (planner toolbar) — removes all drawn waypoints from the map and the app's memory.
  Does not affect the mission already uploaded to the rover.

---

## 7. Auto Mode — Running a Mission

Auto mode controls the execution of an uploaded mission on the rover.

### Activating Auto Mode

Tap **MODE ▼** → select **Auto Mode**.

The **Auto Toolbar** appears at the bottom:

```
[R1] [R2] [START] [PAUSE] [CLEAR]
```

### Selecting a Rover

- **R1** — selects Rover 1 (red) as the command target
- **R2** — selects Rover 2 (blue) as the command target

The button for the selected rover becomes fully opaque; unselected rovers fade slightly.
Tapping R1 or R2 also toggles the visibility of that rover's mission path on the map.

### Starting a Mission

> A mission must have been uploaded in Planner Mode before this step.

1. Tap **START**.
2. The app first sends **ARM** (`MAV_CMD_COMPONENT_ARM_DISARM`, param1=1).
3. 500 ms later, it sends **AUTO mode** (`MAV_CMD_DO_SET_MODE`, param2=3).
4. A toast confirms: `"Rover X: Armed & AUTO"`
5. The rover begins driving to the first waypoint.

### Watching Progress

As the rover moves:

- The rover **icon moves and rotates** on the map to follow GPS position.
- The **current target waypoint** turns **cyan** on the map.
- The **completed route segment** turns **green**; the remaining route stays **red**.
- If camera follow is on (📍 button), the map pans with the rover.

### Pausing a Mission

Tap **PAUSE** to send a `DO_SET_MODE MANUAL` command (param2=0). The rover stops its
mission executor and holds position. The rover stays armed.

To resume, tap **START** again (this re-sends ARM + AUTO).

### Clearing a Mission

Tap **CLEAR** to:
1. Delete all waypoints from the planner.
2. Upload an empty (zero-waypoint) mission to the rover, which clears its mission queue.

---

## 8. Emergency Stop

The **E-STOP** button is visible in the top bar **at all times**, regardless of mode.

### What it does

Tapping **E-STOP** broadcasts a `MAV_CMD_COMPONENT_ARM_DISARM` command
with `param1 = 0` (disarm) to **all system IDs** (MAVLink broadcast address 0).

On the rover Pi, this immediately:
1. Stops the mission executor (rover halts in place)
2. Sets `armed = False`
3. Sends zero-speed commands to the RP2040

### When to use it

- Rover is driving toward an obstacle and you need it to stop immediately
- Unexpected rover behaviour during autonomous mission
- Any situation requiring an instant halt

> **Hardware override:** The rover's physical emergency switch (SWA on the RP2040,
> Channel 3) is independent and cannot be overridden by software. Pulling SWA low
> also halts the rover even if the Pi continues running.

---

## 9. Map Controls

### Satellite / Standard Toggle

Tap the **🗺** (map layer) button on the right side of the screen.
- **Satellite** — aerial imagery (default, best for field planning)
- **Standard** — road/terrain view

### Camera Follow Mode

Tap the **📍** (location) button to toggle follow mode.

| Button state | Camera behaviour |
|---|---|
| White background | Fixed — map stays where you left it |
| Green background | Follow — map pans to keep the selected rover centred |

Follow mode disables automatically if you manually drag the map.

### Zooming and Panning

Use standard Android pinch-to-zoom and drag gestures anywhere on the map.
The map starts at **zoom level 20** (approximately rooftop level) over a default
location. Zoom in/out with the Google Maps zoom controls (shown at the bottom-right
of the map).

### Clicking a Rover Marker

Tap any rover icon on the map to **select that rover**. All subsequent commands
(manual drive, upload, start, stop) target the selected rover.

---

## 10. Saving and Loading Missions

Routes drawn in Planner Mode can be saved as CSV files and reloaded in future sessions.

### Saving

1. Draw a route in Planner Mode.
2. Tap **💾** (Save).
3. Enter a name for the mission file (e.g. `field_a_north`).
4. Tap **Save**. The file is saved as `field_a_north.csv`.

### Loading

1. Tap **📂** (Load).
2. A list of saved `.csv` files appears.
3. Tap a file name to load it. The waypoints are drawn on the map immediately.

### File location

Files are stored on the device at:
```
/sdcard/Android/data/com.siyi.agrirover/files/
```

### CSV format

```
Latitude,Longitude
34.266079,-119.081079
34.266200,-119.081100
```

Files can be created or edited on a PC and transferred to the device for loading.

---

## 11. Multi-Rover Operation

The app supports up to two rovers simultaneously (system IDs 1 and 2).

### How rovers are identified

Each rover's Raspberry Pi is configured with a unique MAVLink system ID in
`rover_controller/config.py` (`MAV_SYSTEM_ID`). The app uses the system ID
to separate their positions, missions, and arm states.

### Rover selection

The **selected rover** (white ring on the icon) receives:
- Manual joystick commands
- Mission uploads
- START / PAUSE / CLEAR commands

All other rovers are visible on the map but not commanded.

To select a rover:
- **Tap its icon on the map** — a toast confirms the selection
- **Tap R1 or R2** in Auto mode toolbar

### Network note

The app sends all outbound MAVLink messages to a single IP address (the one entered
in the Connect dialog). For multi-rover operation, both Pi units must be reachable at
the same address (e.g. via a shared access point or router). Each message is addressed
to a specific system ID so the rover with a matching ID acts on it.

The emergency stop uses **system ID 0 (broadcast)** so both rovers disarm simultaneously.

---

## 12. Troubleshooting

### Connection dot stays orange

**Symptom:** The `●` dot is orange after tapping Connect.

**Causes and fixes:**

| Cause | Fix |
|---|---|
| SIYI air unit not powered on | Power on the air unit before starting the rover controller |
| SIYI RF link not established | Check RF indicator on the MK32 — wait for a solid link |
| Android not connected to MK32 Wi-Fi | Open Wi-Fi settings, connect to the SIYI MK32 access point |
| Wrong ground unit IP | Default is `192.168.144.11`; check SIYI documentation if customised |
| Pi's `rover_controller/main.py` is not running | SSH to Pi and start the controller |
| Pi UART not connected to air unit | Check TX/RX wiring (Pi GPIO 14/15); check Bluetooth is disabled on Pi |
| Firewall blocking UDP port 14550 | Run `sudo ufw allow 14550/udp` on the Pi |

---

### Connection dot turns red during operation

**Symptom:** The dot turns red mid-operation with `● LOST`.

**Causes:**
- SIYI RF link lost (range, obstruction, interference) — check RF signal on MK32
- Pi process crashed — check SSH terminal
- Air unit power interrupted on the rover

**Recovery:** The connection restores automatically when heartbeats resume.
If the rover is in AUTO mode and the mission executor is running, **it continues
driving** even while the GCS link is lost. Use the hardware SWA switch to stop it
if needed.

---

### COMMAND_ACK error toasts appear after pressing START

**Symptom:** Toast shows `"Rover X cmd 176: Denied"` or `"Unsupported"`.

| Toast message | Likely cause |
|---|---|
| `cmd 400: Denied` | Rover is already armed, or safety switch (SWA) is in emergency position |
| `cmd 176: Denied` | No mission loaded on the rover — upload one first |
| `cmd 176: Unsupported` | Rover Pi is running in `--mode manual` (no autopilot) |

---

### Rover marker not appearing on the map

**Symptom:** Connection dot is green but no rover icon appears.

**Causes:**
- Rover Pi has no GPS fix — `GLOBAL_POSITION_INT` sends `lat=0, lon=0` which is filtered out
- Pi's `TelemetryReporter.lat` is `None` (GPS reader not connected to the Pi)

**Fix:** Ensure a GPS module is connected to the Pi and the `GPS_DEVICE` path in
`rover_controller/config.py` is correct. In the meantime, manually navigate the map
to the rover's known location.

---

### Waypoint markers are not turning cyan

**Symptom:** The rover is moving through the mission but the map still shows all
waypoints in red.

**Cause:** The `MISSION_CURRENT` message is not being sent by the rover, or UDP
packets are being dropped.

**Check:** On the Pi terminal, look for `[AUTO] MISSION WP1…` status updates. If
they are showing, the Pi is sending `MISSION_CURRENT`. Packet loss may be causing
the GCS to miss them.

---

### Joystick buttons are unresponsive

**Symptom:** Pressing ▲/▼/◀/▶ has no effect on the rover.

**Causes:**

| Cause | Check |
|---|---|
| Not in Manual mode | Tap MODE ▼ → Manual Mode |
| Rover is in AUTO mode (armed) | Send PAUSE first; AUTO mode suppresses MANUAL_CONTROL |
| Connection not established | Check `●` dot; verify SIYI RF link and ground unit IP in Connect dialog |
| Pi running `--mode autopilot` only listens to COMMAND_LONG, not MANUAL_CONTROL during active mission | Send PAUSE (DO_SET_MODE manual) to interrupt the mission first |

---

### E-STOP does not stop the rover

**Symptom:** Tapping E-STOP produces a toast but the rover keeps moving.

**Causes:**
- Network packet lost — try tapping E-STOP again
- Rover's hardware SWA switch is in autonomous-override position — physically move the switch

The hardware SWA switch (Channel 3 on the RP2040) is the ultimate override and cannot
be defeated by software. If E-STOP over network fails, use the SWA switch.

---

### Uploaded mission disappears after reconnecting

**Symptom:** Mission was uploaded successfully but after reconnecting, the rover
reports `"Denied"` when START is pressed.

**Cause:** The rover Pi's mission is stored in memory (not flash). Restarting `main.py`
clears the loaded mission.

**Fix:** After reconnecting, re-upload the mission from Planner Mode (load from CSV
if you saved it, then tap UPLOAD).
