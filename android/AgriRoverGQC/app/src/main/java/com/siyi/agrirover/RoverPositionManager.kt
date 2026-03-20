package com.siyi.agrirover

import android.content.Context
import android.net.wifi.WifiManager
import android.util.Log
import io.dronefleet.mavlink.MavlinkConnection
import io.dronefleet.mavlink.common.*
import io.dronefleet.mavlink.util.EnumValue
import kotlinx.coroutines.*
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream

/**
 * A recorded mission item — either a GPS waypoint or a servo state change.
 * Built by MainActivity during recording and passed to uploadRecordedMission().
 */
sealed class MissionAction {
    /** speed: m/s at record time (0f = use navigator default max_speed).
     *  holdSecs: seconds to wait after arriving (0f = no hold). */
    data class Waypoint(val lat: Double, val lon: Double,
                        val speed: Float = 0f, val holdSecs: Float = 0f) : MissionAction()
    /** servo: 5–8 (PPM CH5–CH8).  pwm: µs value to set on that channel. */
    data class ServoCmd(val servo: Int, val pwm: Int) : MissionAction()
}

/**
 * Manages all MAVLink UDP communication with rover(s) over the master rover's WiFi hotspot.
 *
 * Discovery:
 *   The GCS heartbeat is broadcast to 255.255.255.255:14550.  All rovers on the hotspot
 *   receive it, learn the GCS address, and start replying.  The first packet from each
 *   rover is used to record its IP so subsequent commands reach the right machine.
 *
 * Callbacks (all delivered on the Main thread):
 *   onPositionUpdate(sysId, lat, lon, heading)              — GLOBAL_POSITION_INT (#33)
 *   onMissionAck(message)                                   — MISSION_ACK (#47) upload result
 *   onSensorUpdate(sysId, bat%, tempC, tank%, humid%)       — SYS_STATUS, SCALED_PRESSURE, NAMED_VALUE_FLOAT
 *   onArmState(sysId, isArmed)                              — HEARTBEAT (#0) base_mode bit 7
 *   onMissionProgress(sysId, waypointIndex)                 — MISSION_CURRENT (#42)
 *   onCommandAck(sysId, commandId, result)                  — COMMAND_ACK (#77)  result=0 → ACCEPTED
 *   onConnectionChange(sysId, isConnected)                  — rover heartbeat watchdog (3 s timeout)
 *   onRcChannels(sysId, channels)                           — RC_CHANNELS (#65) logical/PPM order µs [0..8]; [2]=SWA emergency, [3]=SWB autonomous, [8]=rover select
 *
 * Sending to rovers:
 *   sendRcChannelsOverride(sysId, channels)                 — RC_CHANNELS_OVERRIDE (#70) 8×PPM µs
 *   sendManualControl(sysId, x, y, z, r, buttons)          — MANUAL_CONTROL (#69) legacy axes
 *   sendCommand(sysId, commandId, p1, p2)                   — COMMAND_LONG (#76)
 *   uploadMission(sysId, waypoints)                         — mission upload protocol
 */
class RoverPositionManager(
    private val onPositionUpdate:   (Int, Double, Double, Float) -> Unit,
    private val onMissionAck:       (String) -> Unit,
    private val onSensorUpdate:     (Int, Float, Float, Float, Float) -> Unit,
    private val onArmState:         (Int, Boolean) -> Unit,
    private val onMissionProgress:  (Int, Int) -> Unit,
    private val onCommandAck:       (Int, Int, Int) -> Unit,
    private val onConnectionChange: (Int, Boolean) -> Unit,
    private val onRcChannels:       (Int, IntArray) -> Unit,
    /** Called when RC switch state (from master) disagrees with slave reported mode. */
    private val onLinkMismatch:     (String) -> Unit,
    /**
     * Called when the rover's rerouted path is received.
     * path: list of (lat, lon, isBypass) triples for each waypoint in order.
     * An empty list means the path was cleared.
     */
    private val onReroutedPath:     (Int, List<Triple<Double, Double, Boolean>>) -> Unit,
    /**
     * Called when GPS_RAW_INT (#24) is received.
     * fixType: 0=NO_GPS  1=NO_FIX  2=2D  3=3D  4=DGPS  5=RTK_FLOAT  6=RTK_FIXED
     */
    private val onGpsStatus:        (Int, Int) -> Unit,
) {
    private val PORT        = 14550
    private val MASTER_SYSID = 1
    private val SLAVE_SYSID  = 2

    // Thresholds — must match config.py values
    private val EMERGENCY_THRESHOLD  = 1700
    private val AUTONOMOUS_THRESHOLD = 1700

    private var isRunning  = false
    private val scope      = CoroutineScope(Dispatchers.IO + SupervisorJob())
    private var socket: DatagramSocket? = null
    private var wifiLock: WifiManager.WifiLock? = null

    // Broadcast address — reaches all rovers on the hotspot LAN without knowing their IPs
    private val broadcastAddress: InetAddress = InetAddress.getByName("255.255.255.255")

    // Per-rover IP addresses, auto-discovered from incoming packets
    private val roverAddresses = HashMap<Int, InetAddress>()

    // Persistent MAVLink 2 output connection — keeps packet sequence counter
    // advancing correctly across sends (required by the MAVLink spec).
    private val txBuf  = ByteArrayOutputStream()
    private val txConn = MavlinkConnection.create(null, txBuf)

    // Per-rover last-heartbeat timestamp for the connection watchdog
    private val roverLastHb    = HashMap<Int, Long>()
    private val roverConnected = HashMap<Int, Boolean>()

    // Per-rover RC_CHANNELS PPM values — used by checkLinkMismatch
    private val roverPpmChannels = HashMap<Int, IntArray>()

    // Per-rover last HEARTBEAT base_mode and system_status (for mismatch detection)
    private val roverBaseMode   = HashMap<Int, Int>()
    private val roverSysStatus  = HashMap<Int, Int>()
    // Timestamp when a mismatch was first detected (null = no current mismatch)
    private var mismatchStart: Long? = null
    private val MISMATCH_TIMEOUT_MS  = 2000L   // fire callback after 2 s of persistent mismatch

    // Missions being uploaded, keyed by target system ID
    private val pendingMissions = HashMap<Int, List<MissionItemInt>>()

    // TUNNEL rerouted-path reassembly: sysId → map of chunkIdx → byte content
    private data class TunnelAssembly(val total: Int, val chunks: HashMap<Int, ByteArray>)
    private val tunnelBuf = HashMap<Int, TunnelAssembly>()

    // ─── Public API ──────────────────────────────────────────────────────────

    fun startListening(ctx: Context? = null) {
        if (isRunning) return
        isRunning = true

        // Acquire a high-performance WiFi lock to prevent Android power-save mode from
        // buffering incoming UDP packets (which causes 50-100ms latency per mission item).
        ctx?.applicationContext?.let { appCtx ->
            try {
                val wm = appCtx.getSystemService(Context.WIFI_SERVICE) as? WifiManager
                    ?: return@let
                // WIFI_MODE_FULL_LOW_LATENCY (API 29) is more aggressive than HIGH_PERF
                // and still effective on Android 12+. HIGH_PERF was deprecated in API 29
                // and silently does less on newer devices. LOW_LATENCY explicitly requests
                // the AP not to buffer packets, cutting per-item latency from ~100ms to ~1ms.
                val lockMode = if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.Q) {
                    WifiManager.WIFI_MODE_FULL_LOW_LATENCY
                } else {
                    @Suppress("DEPRECATION")
                    WifiManager.WIFI_MODE_FULL_HIGH_PERF
                }
                wifiLock = wm.createWifiLock(lockMode, "AgriRover:UDP")
                    .also { it.acquire() }
                Log.i("RoverMgr", "WiFi lock acquired (mode=$lockMode)")
            } catch (e: Exception) {
                Log.w("RoverMgr", "WiFi lock unavailable: ${e.message}")
            }
        }

        scope.launch {
            try {
                socket = DatagramSocket(PORT).also { it.broadcast = true }
                Log.i("RoverMgr", "Listening on UDP $PORT — broadcasting heartbeats")

                // GCS heartbeat — 1 Hz, broadcast so all rovers learn our address
                launch { heartbeatLoop() }

                // Connection watchdog — marks a rover disconnected after 3 s of no HB
                launch { watchdogLoop() }

                // Receive loop
                receiveLoop()

            } catch (e: Exception) {
                Log.e("RoverMgr", "Socket error: ${e.message}")
            }
        }
    }

    fun stopListening() {
        isRunning = false
        socket?.close()
        scope.cancel()
        wifiLock?.let { if (it.isHeld) it.release() }
        wifiLock = null
    }

    /** Send MANUAL_CONTROL (#69).
     *  x = throttle  (−1000…+1000, forward positive)  → rover CH1
     *  y = steering  (−1000…+1000, right positive)    → rover CH2
     */
    fun sendManualControl(sysId: Int, x: Int, y: Int, z: Int, r: Int, buttons: Int) {
        scope.launch {
            sendMavlinkTo(
                roverIp(sysId),
                ManualControl.builder()
                    .target(sysId)
                    .x(x).y(y).z(z).r(r)
                    .buttons(buttons)
                    .build()
            )
        }
    }

    /**
     * Send RC_CHANNELS_OVERRIDE (#70) with 8 PPM channel values (µs, 1000–2000).
     *
     * This is the primary joystick-to-rover path over WiFi UDP.  Each channel
     * maps directly to a RP2040 PPM output channel:
     *   ch[0] = throttle  ch[1] = steering  ch[2] = SWA (emergency)
     *   ch[3] = SWB (mode: >1700 = AUTONOMOUS)  ch[4..7] = aux/relay/servo
     *
     * The rover applies these channels directly to the RP2040 (master: via
     * emitter firmware; slave: via <J:...> serial fallback).
     *
     * Use sysId=0 to address all rovers simultaneously (broadcast).
     */
    fun sendRcChannelsOverride(sysId: Int, channels: IntArray) {
        scope.launch {
            val ch = IntArray(8) { i -> if (i < channels.size) channels[i] else 1500 }
            sendMavlinkTo(
                roverIp(sysId),
                RcChannelsOverride.builder()
                    .targetSystem(sysId).targetComponent(1)
                    .chan1Raw(ch[0]).chan2Raw(ch[1]).chan3Raw(ch[2]).chan4Raw(ch[3])
                    .chan5Raw(ch[4]).chan6Raw(ch[5]).chan7Raw(ch[6]).chan8Raw(ch[7])
                    .build()
            )
        }
    }

    /** Send COMMAND_LONG (#76).
     *  Supported commands by the rover controller:
     *    400  MAV_CMD_COMPONENT_ARM_DISARM  p1=1 arm / p1=0 disarm
     *    176  MAV_CMD_DO_SET_MODE           p2=0 manual / p2=3 auto
     *    178  MAV_CMD_DO_CHANGE_SPEED       p2=speed m/s
     *    181  MAV_CMD_DO_SET_RELAY          p1=relay(0-3) p2=state(0/1)
     *    183  MAV_CMD_DO_SET_SERVO          p1=servo(5-8) p2=PWM µs
     *  Use sysId=0 for broadcast (e.g. E-STOP to all rovers).
     */
    fun sendCommand(sysId: Int, commandId: Int, p1: Float, p2: Float) {
        scope.launch {
            sendMavlinkTo(
                roverIp(sysId),
                CommandLong.builder()
                    .targetSystem(sysId).targetComponent(1)
                    .command(EnumValue.create(MavCmd::class.java, commandId))
                    .confirmation(0)
                    .param1(p1).param2(p2).param3(0f).param4(0f)
                    .param5(0f).param6(0f).param7(0f)
                    .build()
            )
        }
    }

    /**
     * Send COMMAND_LONG 3× at 100 ms intervals for safety-critical commands
     * (E-STOP, disarm) where a single dropped UDP packet could be fatal.
     * Uses `confirmation` field 0, 1, 2 per MAVLink spec for retries.
     */
    fun sendCriticalCommand(sysId: Int, commandId: Int, p1: Float, p2: Float) {
        scope.launch {
            repeat(3) { attempt ->
                sendMavlinkTo(
                    roverIp(sysId),
                    CommandLong.builder()
                        .targetSystem(sysId).targetComponent(1)
                        .command(EnumValue.create(MavCmd::class.java, commandId))
                        .confirmation(attempt)
                        .param1(p1).param2(p2).param3(0f).param4(0f)
                        .param5(0f).param6(0f).param7(0f)
                        .build()
                )
                if (attempt < 2) delay(100L)
            }
        }
    }

    /** Upload a mission to a specific rover using the MAVLink mission upload protocol. */
    fun uploadMission(sysId: Int, waypoints: List<Pair<Double, Double>>) {
        scope.launch {
            val items = waypoints.mapIndexed { index, (lat, lon) ->
                MissionItemInt.builder()
                    .targetSystem(sysId).targetComponent(1)
                    .seq(index)
                    .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                    .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                    .current(if (index == 0) 1 else 0)
                    .autocontinue(1)
                    .x((lat * 1e7).toInt())
                    .y((lon * 1e7).toInt())
                    .z(0f)
                    .param1(0f).param2(0f).param3(0f).param4(0f)
                    .build()
            }
            streamMission(sysId, items)
        }
    }

    /**
     * Upload a recorded mission that may contain both NAV_WAYPOINT and DO_SET_SERVO items.
     *
     * Waypoints → MAV_CMD_NAV_WAYPOINT (frame GLOBAL_RELATIVE_ALT_INT)
     * ServoCmds → MAV_CMD_DO_SET_SERVO (frame MISSION, param1=servo 5-8, param2=pwm µs)
     *
     * DO_SET_SERVO items are executed by the autopilot when it reaches the preceding
     * waypoint, following standard MAVLink mission DO-command sequencing.
     */
    fun uploadRecordedMission(sysId: Int, actions: List<MissionAction>) {
        scope.launch {
            var seq = 0
            val items = actions.map { action ->
                when (action) {
                    is MissionAction.Waypoint ->
                        MissionItemInt.builder()
                            .seq(seq).targetSystem(sysId).targetComponent(1)
                            .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                            .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                            .current(if (seq++ == 0) 1 else 0).autocontinue(1)
                            .x((action.lat * 1e7).toInt()).y((action.lon * 1e7).toInt())
                            .z(action.speed).param1(action.holdSecs).param2(0f).param3(0f).param4(0f)
                            .build()
                    is MissionAction.ServoCmd ->
                        MissionItemInt.builder()
                            .seq(seq).targetSystem(sysId).targetComponent(1)
                            .frame(MavFrame.MAV_FRAME_MISSION)
                            .command(MavCmd.MAV_CMD_DO_SET_SERVO)
                            .current(if (seq++ == 0) 1 else 0).autocontinue(1)
                            .param1(action.servo.toFloat()).param2(action.pwm.toFloat())
                            .param3(0f).param4(0f).x(0).y(0).z(0f)
                            .build()
                }
            }
            streamMission(sysId, items)
        }
    }

    /**
     * Upload a mission that includes obstacle fence polygons.
     *
     * Fence vertices (MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, cmd=5003) are prepended
     * before navigation items so the navigator can extract them when the mission upload
     * completes.  Each vertex in a polygon becomes one MISSION_ITEM_INT with:
     *   param1 = total vertices in this polygon  (so the receiver can reconstruct it)
     *   x      = latitude  × 1e7
     *   y      = longitude × 1e7
     *
     * @param actions   Full mission sequence (Waypoint + ServoCmd interleaved, same as uploadRecordedMission)
     * @param obstacles List of polygons, each polygon is a list of (lat, lon) pairs
     */
    fun uploadMissionWithObstacles(
        sysId: Int,
        actions: List<MissionAction>,
        obstacles: List<List<Pair<Double, Double>>>,
    ) {
        scope.launch {
            val items = mutableListOf<MissionItemInt>()

            // Fence items first (cmd=5003)
            for (poly in obstacles) {
                if (poly.size < 3) continue
                for ((lat, lon) in poly) {
                    items.add(MissionItemInt.builder()
                        .seq(items.size).targetSystem(sysId).targetComponent(1)
                        .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                        .command(EnumValue.create(MavCmd::class.java, 5003))
                        .current(0).autocontinue(1)
                        .param1(poly.size.toFloat()).param2(0f).param3(0f).param4(0f)
                        .x((lat * 1e7).toInt()).y((lon * 1e7).toInt()).z(0f)
                        .build())
                }
            }

            // Navigation / servo items
            var firstNavWp = true
            for (action in actions) {
                when (action) {
                    is MissionAction.Waypoint -> {
                        val isCurrent = if (firstNavWp) { firstNavWp = false; 1 } else 0
                        items.add(MissionItemInt.builder()
                            .seq(items.size).targetSystem(sysId).targetComponent(1)
                            .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                            .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
                            .current(isCurrent).autocontinue(1)
                            .x((action.lat * 1e7).toInt()).y((action.lon * 1e7).toInt())
                            .z(action.speed).param1(action.holdSecs).param2(0f).param3(0f).param4(0f)
                            .build())
                    }
                    is MissionAction.ServoCmd -> {
                        items.add(MissionItemInt.builder()
                            .seq(items.size).targetSystem(sysId).targetComponent(1)
                            .frame(MavFrame.MAV_FRAME_MISSION)
                            .command(MavCmd.MAV_CMD_DO_SET_SERVO)
                            .current(0).autocontinue(1)
                            .param1(action.servo.toFloat()).param2(action.pwm.toFloat())
                            .param3(0f).param4(0f).x(0).y(0).z(0f)
                            .build())
                    }
                }
            }

            streamMission(sysId, items)
        }
    }

    /**
     * Streaming mission upload: disarm rover, send MISSION_COUNT, then push all items
     * immediately with a small inter-packet gap.  The rover accepts items as they arrive
     * and only falls back to REQUEST_INT handshake (via its retry timer) for missed items.
     *
     * Safety: DISARM×3 is sent before MISSION_COUNT so the rover cannot begin navigating
     * while a new mission is being loaded.  User must press START explicitly afterwards.
     *
     * This avoids the per-item WiFi DTIM round-trip that adds 66–150 ms per item when
     * WifiLock LOW_LATENCY is not fully honoured.  All items are delivered in a single
     * AP DTIM window instead of N separate wake cycles.
     */
    private suspend fun streamMission(sysId: Int, items: List<MissionItemInt>) {
        pendingMissions[sysId] = items
        val ip = roverIp(sysId)
        // Safety: disarm before uploading — rover must not start navigating during upload.
        // 3× retries (confirmation=0,1,2) per MAVLink spec for critical commands.
        repeat(3) { attempt ->
            sendMavlinkTo(ip, CommandLong.builder()
                .targetSystem(sysId).targetComponent(1)
                .command(EnumValue.create(MavCmd::class.java, 400))
                .confirmation(attempt)
                .param1(0f).param2(0f).param3(0f).param4(0f).param5(0f).param6(0f).param7(0f)
                .build())
            if (attempt < 2) delay(100L)
        }
        // Allow rover to process disarm before mission upload begins.
        delay(100L)
        sendMavlinkTo(ip, MissionCount.builder()
            .targetSystem(sysId).targetComponent(1)
            .count(items.size).build())
        // Brief pause so rover processes MISSION_COUNT before items start arriving.
        delay(150L)
        for (item in items) {
            sendMavlinkTo(ip, item)
            delay(20L)   // 20 ms gap: avoids flooding rover's socket buffer
        }
    }

    // ─── Internal loops ───────────────────────────────────────────────────────

    private suspend fun heartbeatLoop() {
        while (isRunning) {
            // Broadcast so every rover on the hotspot receives and learns the GCS IP
            sendMavlinkTo(
                broadcastAddress,
                Heartbeat.builder()
                    .type(EnumValue.create(MavType::class.java, 6))           // MAV_TYPE_GCS
                    .autopilot(EnumValue.create(MavAutopilot::class.java, 8)) // MAV_AUTOPILOT_INVALID
                    .baseMode(EnumValue.create(MavModeFlag::class.java, 0))
                    .customMode(0)
                    .systemStatus(EnumValue.create(MavState::class.java, 4))  // MAV_STATE_ACTIVE
                    .build()
            )
            delay(1000L)
        }
    }

    private suspend fun watchdogLoop() {
        while (isRunning) {
            delay(1000L)
            val now = System.currentTimeMillis()
            for ((sysId, lastHb) in roverLastHb.toMap()) {
                val isNowConnected = (now - lastHb) < 3000L
                val wasConnected   = roverConnected[sysId] ?: false
                if (wasConnected != isNowConnected) {
                    roverConnected[sysId] = isNowConnected
                    withContext(Dispatchers.Main) { onConnectionChange(sysId, isNowConnected) }
                }
            }
        }
    }

    private suspend fun receiveLoop() {
        val buffer = ByteArray(2048)
        val packet = DatagramPacket(buffer, buffer.size)

        while (isRunning) {
            try {
                socket?.receive(packet) ?: break

                // Auto-discover: record the rover's IP so we can send commands back to it
                val stream     = ByteArrayInputStream(packet.data, 0, packet.length)
                val connection = MavlinkConnection.create(stream, null)
                val message    = connection.next() ?: continue
                val senderId   = message.originSystemId

                // Ignore our own heartbeats (GCS sysId = 255)
                if (senderId == 255) continue

                // First packet from this rover — register its address
                if (!roverAddresses.containsKey(senderId)) {
                    roverAddresses[senderId] = packet.address
                    Log.i("RoverMgr", "Discovered Rover $senderId at ${packet.address.hostAddress}")
                }

                dispatchMessage(senderId, message.payload)  // non-blocking: UI callbacks use scope.launch

            } catch (e: Exception) {
                if (isRunning) Log.w("RoverMgr", "Recv: ${e.message}")
            }
        }
    }

    private fun dispatchMessage(senderId: Int, payload: Any?) {
        when (payload) {

            // HEARTBEAT (#0) — track connection, arm state and mode flags
            is Heartbeat -> {
                roverLastHb[senderId]   = System.currentTimeMillis()
                roverBaseMode[senderId] = payload.baseMode().value()
                roverSysStatus[senderId]= payload.systemStatus().value()

                val wasConnected = roverConnected[senderId] ?: false
                if (!wasConnected) {
                    roverConnected[senderId] = true
                    scope.launch(Dispatchers.Main) { onConnectionChange(senderId, true) }
                }
                val armed = (payload.baseMode().value() and 128) != 0
                scope.launch(Dispatchers.Main) { onArmState(senderId, armed) }

                // Check RC ↔ UDP state consistency after every slave heartbeat
                if (senderId == SLAVE_SYSID) checkLinkMismatch()
            }

            // GLOBAL_POSITION_INT (#33) — rover GPS position
            is GlobalPositionInt -> {
                val lat = payload.lat() / 1e7
                val lon = payload.lon() / 1e7
                if (lat == 0.0 && lon == 0.0) return
                val hdg = if (payload.hdg() == 65535) 0f else payload.hdg() / 100f
                scope.launch(Dispatchers.Main) { onPositionUpdate(senderId, lat, lon, hdg) }
            }

            // MISSION_REQUEST_INT — rover asks for a specific waypoint during upload
            is MissionRequestInt -> sendMissionItem(senderId, payload.seq())

            // MISSION_ACK (#47) — upload handshake complete
            is MissionAck -> {
                pendingMissions.remove(senderId)
                val resultText = if (payload.type().value() == 0) "Upload Complete!" else "Upload Failed (${payload.type().value()})"
                scope.launch(Dispatchers.Main) { onMissionAck("Rover $senderId: $resultText") }
            }

            // MISSION_CURRENT (#42) — which waypoint the rover is heading to
            is MissionCurrent -> {
                scope.launch(Dispatchers.Main) { onMissionProgress(senderId, payload.seq()) }
            }

            // COMMAND_ACK (#77) — result of a COMMAND_LONG we sent
            is CommandAck -> {
                val cmd    = payload.command().value()
                val result = payload.result().value()
                scope.launch(Dispatchers.Main) { onCommandAck(senderId, cmd, result) }
            }

            // SYS_STATUS (#1) — battery percentage
            is SysStatus -> {
                scope.launch(Dispatchers.Main) {
                    onSensorUpdate(senderId, payload.batteryRemaining().toFloat(), -1f, -1f, -1f)
                }
            }

            // SCALED_PRESSURE (#137) — temperature + absolute pressure
            is ScaledPressure -> {
                val tempC = payload.temperature() / 100f
                scope.launch(Dispatchers.Main) { onSensorUpdate(senderId, -1f, tempC, -1f, -1f) }
            }

            // NAMED_VALUE_FLOAT (#251) — custom scalar sensors
            is NamedValueFloat -> {
                val name  = payload.name().trimEnd('\u0000')
                val value = payload.value()
                when (name) {
                    "TANK"  -> scope.launch(Dispatchers.Main) {
                        onSensorUpdate(senderId, -1f, -1f, value, -1f)
                    }
                    "HUMID" -> scope.launch(Dispatchers.Main) {
                        onSensorUpdate(senderId, -1f, -1f, -1f, value)
                    }
                }
            }

            // RC_CHANNELS (#65) — raw SBUS channel values from the RP2040 (MK32 order).
            // mavlink_bridge sends all 16 SBUS channels in SBUS/MK32 order.
            // We remap to logical/PPM order so channel indices match the hardware PPM map.
            is RcChannels -> {
                val raw = intArrayOf(
                    payload.chan1Raw(),  payload.chan2Raw(),  payload.chan3Raw(),
                    payload.chan4Raw(),  payload.chan5Raw(),  payload.chan6Raw(),
                    payload.chan7Raw(),  payload.chan8Raw(),  payload.chan9Raw(),
                    payload.chan10Raw(), payload.chan11Raw(), payload.chan12Raw(),
                    payload.chan13Raw(), payload.chan14Raw(), payload.chan15Raw(),
                    payload.chan16Raw(),
                )
                val channels = remapSbusToLogical(raw)
                roverPpmChannels[senderId] = channels
                scope.launch(Dispatchers.Main) { onRcChannels(senderId, channels) }
                // Re-check mismatch whenever master's actual RC state changes
                if (senderId == MASTER_SYSID) checkLinkMismatch()
            }

            // GPS_RAW_INT (#24) — GPS fix type and quality
            is GpsRawInt -> {
                val fixType = payload.fixType().value()
                scope.launch(Dispatchers.Main) { onGpsStatus(senderId, fixType) }
            }

            // TUNNEL (#385) payload_type=0x5250 — rerouted path chunks from navigator
            is Tunnel -> {
                if (payload.payloadType().value() != 0x5250) return
                val raw       = payload.payload()          // ByteArray[128]
                val pLen      = payload.payloadLength()    // bytes actually used
                if (pLen < 2) return
                val chunkIdx  = raw[0].toInt() and 0xFF
                val total     = raw[1].toInt() and 0xFF
                if (total == 0) return
                val data      = raw.slice(2 until pLen).toByteArray()

                // Store chunk; when all arrived, assemble and parse
                val asm = tunnelBuf.getOrPut(senderId) { TunnelAssembly(total, HashMap()) }
                // If this is a new transmission (different total), reset
                val assembly = if (asm.total != total) {
                    val fresh = TunnelAssembly(total, HashMap())
                    tunnelBuf[senderId] = fresh
                    fresh
                } else asm
                assembly.chunks[chunkIdx] = data

                if (assembly.chunks.size == total) {
                    // All chunks received — concatenate in order
                    val json = (0 until total)
                        .mapNotNull { assembly.chunks[it] }
                        .fold(ByteArray(0)) { acc, b -> acc + b }
                        .toString(Charsets.UTF_8)
                    tunnelBuf.remove(senderId)

                    try {
                        val arr  = org.json.JSONArray(json)
                        val path = mutableListOf<Triple<Double, Double, Boolean>>()
                        for (i in 0 until arr.length()) {
                            val pt = arr.getJSONArray(i)
                            path.add(Triple(pt.getDouble(0), pt.getDouble(1), pt.getInt(2) != 0))
                        }
                        scope.launch(Dispatchers.Main) { onReroutedPath(senderId, path) }
                    } catch (e: Exception) {
                        Log.w("RoverMgr", "TUNNEL JSON parse error: ${e.message}")
                    }
                }
            }
        }
    }

    // ─── Mismatch detection ───────────────────────────────────────────────────

    /**
     * Compares the physical RC switch state (from master's RC_CHANNELS) against
     * what the slave rover is actually applying (from slave's HEARTBEAT flags).
     *
     * A mismatch means the safety commands (emergency / mode) are not reaching
     * the slave correctly.  After MISMATCH_TIMEOUT_MS of persistence, the
     * onLinkMismatch callback fires so the app can stop everything and warn the user.
     *
     * Conditions checked:
     *   • Emergency active on master (SWA < 1700) but slave NOT in emergency  [safety-critical]
     *   • Master back to MANUAL (SWB < 1700) but slave still in autonomous    [safety-critical]
     *   NOTE: master→AUTO but slave→IDLE is intentionally NOT flagged — slave is
     *   stopped (safe); it will go autonomous once its navigator connects.
     */
    private fun checkLinkMismatch() {
        val masterChannels = roverPpmChannels[MASTER_SYSID]
        if (masterChannels == null || masterChannels.size < 4) return

        val slaveConnected = (roverLastHb[SLAVE_SYSID] ?: 0L) >
                              System.currentTimeMillis() - 3000L
        // Only check mismatch when slave is connected; disconnection is
        // handled separately by the watchdog → onConnectionChange callback.
        if (!slaveConnected) { mismatchStart = null; return }

        val masterEmergency  = masterChannels[2] < EMERGENCY_THRESHOLD
        val masterAutonomous = masterChannels[3] > AUTONOMOUS_THRESHOLD

        val slaveBase   = roverBaseMode[SLAVE_SYSID]  ?: return
        val slaveStatus = roverSysStatus[SLAVE_SYSID] ?: return
        val slaveEmergency  = slaveStatus == 6                 // MAV_STATE_EMERGENCY
        val slaveAutonomous = (slaveBase and 0x08) != 0        // MAV_MODE_FLAG_GUIDED_ENABLED

        val mismatch: String? = when {
            // Safety-critical: emergency active on master but slave still moving
            masterEmergency && !slaveEmergency ->
                "EMERGENCY switch active but slave is NOT stopped"
            // Safety-critical: master switched back to MANUAL but slave still running autonomous
            !masterEmergency && !masterAutonomous && slaveAutonomous ->
                "Mode mismatch: master→MANUAL, slave still in AUTONOMOUS"
            // master→AUTONOMOUS but slave→IDLE is NOT a mismatch: slave is stopped (safe).
            // It will go autonomous once its own navigator connects and sends commands.
            else -> null
        }

        if (mismatch != null) {
            val now = System.currentTimeMillis()
            if (mismatchStart == null) mismatchStart = now
            if (now - (mismatchStart ?: now) >= MISMATCH_TIMEOUT_MS) {
                mismatchStart = null   // reset so it fires once, not continuously
                scope.launch(Dispatchers.Main) { onLinkMismatch(mismatch) }
            }
        } else {
            mismatchStart = null
        }
    }

    // ─── Channel remapping ────────────────────────────────────────────────────

    /**
     * Reorders raw SBUS channels (MK32 1-indexed order, passed as 0-indexed array) to
     * logical/PPM order matching the firmware apply_ppm_map() table.
     * No inversions are applied — raw µs values are preserved for threshold comparisons.
     *
     * PPM output map (firmware CLAUDE.md):
     *   logical[0] = raw[2]  (SBUS CH3  → PPM CH1, throttle)
     *   logical[1] = raw[0]  (SBUS CH1  → PPM CH2, steering)
     *   logical[2] = raw[4]  (SBUS CH5  → PPM CH3, SWA emergency)
     *   logical[3] = raw[5]  (SBUS CH6  → PPM CH4, SWB autonomous)
     *   logical[4] = raw[10] (SBUS CH11 → PPM CH5)
     *   logical[5] = raw[11] (SBUS CH12 → PPM CH6)
     *   logical[6] = raw[6]  (SBUS CH7  → PPM CH7)
     *   logical[7] = raw[7]  (SBUS CH8  → PPM CH8)
     *   logical[8] = raw[8]  (SBUS CH9  → rover select switch)
     */
    private fun remapSbusToLogical(raw: IntArray): IntArray {
        fun ch(i: Int) = if (i < raw.size) raw[i] else 1500
        return intArrayOf(
            ch(2),   // PPM CH1 ← SBUS CH3  (throttle)
            ch(0),   // PPM CH2 ← SBUS CH1  (steering)
            ch(4),   // PPM CH3 ← SBUS CH5  (SWA emergency)
            ch(5),   // PPM CH4 ← SBUS CH6  (SWB autonomous)
            ch(10),  // PPM CH5 ← SBUS CH11
            ch(11),  // PPM CH6 ← SBUS CH12
            ch(6),   // PPM CH7 ← SBUS CH7
            ch(7),   // PPM CH8 ← SBUS CH8
            ch(8),   // PPM CH9 ← SBUS CH9  (rover select)
        )
    }

    // ─── Send helpers ─────────────────────────────────────────────────────────

    /** Returns the known IP for a rover, or broadcast for sysId=0 or undiscovered rovers. */
    private fun roverIp(sysId: Int): InetAddress =
        if (sysId == 0) broadcastAddress else roverAddresses[sysId] ?: broadcastAddress

    private fun sendMissionItem(targetSysId: Int, seq: Int) {
        val mission = pendingMissions[targetSysId] ?: return
        if (seq in mission.indices) {
            sendMavlinkTo(roverIp(targetSysId), mission[seq])
        }
    }

    private fun sendMavlinkTo(ip: InetAddress, payload: Any) {
        try {
            val data: ByteArray
            synchronized(txConn) {
                txBuf.reset()
                txConn.send2(255, 0, payload)
                data = txBuf.toByteArray()
            }
            socket?.send(DatagramPacket(data, data.size, ip, PORT))
        } catch (e: Exception) {
            Log.e("RoverMgr", "Send error: ${e.message}")
        }
    }
}
