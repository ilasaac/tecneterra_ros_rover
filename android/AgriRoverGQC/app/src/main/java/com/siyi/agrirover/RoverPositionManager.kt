package com.siyi.agrirover

import android.content.Context
import android.net.wifi.WifiManager
import android.util.Log
import com.google.android.gms.maps.model.LatLng
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
     * speeds: parallel list of speed (m/s) per waypoint.
     * An empty list means the path was cleared.
     */
    private val onReroutedPath:     (Int, List<Triple<Double, Double, Boolean>>, List<Float>) -> Unit,
    /**
     * Called when GPS_RAW_INT (#24) is received.
     * fixType: 0=NO_GPS  1=NO_FIX  2=2D  3=3D  4=DGPS  5=RTK_FLOAT  6=RTK_FIXED
     */
    private val onGpsStatus:        (Int, Int) -> Unit,
    /** Called when NAMED_VALUE_FLOAT "STATUS" is received. status: "NA" | "MSL" | "ARM" */
    private val onNavStatus:        (Int, String) -> Unit,
    /** Called when SBUS_OK or RF_OK named float is received. type="SBUS"|"RF", ok=true/false */
    private val onLinkStatus:       (Int, String, Boolean) -> Unit,
    /**
     * Called when a mission download from a rover completes (triggered by MSN_ID change).
     * waypoints: ordered list of (lat, lon, isBypass) for all waypoints in the current path.
     */
    private val onMissionDownloaded: (Int, List<Triple<Double, Double, Boolean>>) -> Unit,
    /**
     * Called when PARAM_VALUE is received from a rover (response to PARAM_REQUEST_LIST or PARAM_SET).
     * sysId: rover system ID.  name: MAVLink param_id (≤16 chars).
     * value: current float value.  index: 0-based index.  count: total param count.
     */
    private val onParamValue: (sysId: Int, name: String, value: Float, index: Int, count: Int) -> Unit,
    /**
     * Called when NAMED_VALUE_FLOAT "HACC" is received.
     * sysId: rover system ID.  haccMm: horizontal accuracy estimate in mm (-1 = unknown).
     */
    private val onHaccStatus: (Int, Float) -> Unit,
    /** Called when NAMED_VALUE_FLOAT "REROUTE" is received. pending=true means rover awaits confirmation. */
    private val onReroutePending: (Int, Boolean) -> Unit,
    /** Called when obstacle polygons are received from the rover (mission_fence topic). */
    private val onObstaclesReceived: (Int, List<List<Pair<Double, Double>>>) -> Unit = { _, _ -> },
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

    // Per-rover last GLOBAL_POSITION_INT time_boot_ms — discard out-of-order packets
    private val roverLastPosTms = HashMap<Int, Long>()

    // Per-rover, per-name last NAMED_VALUE_FLOAT time_boot_ms — discard out-of-order/duplicate packets.
    // Must be per-name: all names in one batch share the same timestamp, so a per-rover guard
    // would discard every name after the first.
    private val roverLastNvfTms = HashMap<Int, HashMap<String, Long>>()

    // Per-rover last RC_CHANNELS time_boot_ms — discard out-of-order packets
    private val roverLastRcTms = HashMap<Int, Long>()

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
    // Active upload coroutine per rover — cancelled before a new upload starts
    private val uploadJobs = HashMap<Int, Job>()

    // TUNNEL rerouted-path reassembly: sysId → map of chunkIdx → byte content
    private data class TunnelAssembly(val total: Int, val chunks: HashMap<Int, ByteArray>)
    private val tunnelBuf = HashMap<Int, TunnelAssembly>()

    // Mission download state (rover → GQC, triggered by MSN_ID change)
    private val roverMsnId           = HashMap<Int, Int>()                        // sysId → last known MSN_ID
    private val missionDownloadItems = HashMap<Int, HashMap<Int, Triple<Double, Double, Boolean>>>() // sysId → seq → (lat, lon, bypass)
    private val missionDownloadCount = HashMap<Int, Int>()                        // sysId → expected item count

    // ─── rosbridge WebSocket telemetry ──────────────────────────────────────
    // Persistent WebSocket per rover — subscribes to ROS2 topics via rosbridge.
    // Auto-connects when rover IP is discovered from MAVLink, reconnects on failure.
    private val rosbridgeWs = HashMap<Int, okhttp3.WebSocket>()
    private val rosbridgeClient = okhttp3.OkHttpClient.Builder()
        .connectTimeout(5, java.util.concurrent.TimeUnit.SECONDS)
        .readTimeout(0, java.util.concurrent.TimeUnit.SECONDS)  // no read timeout for persistent sub
        .build()

    private fun connectRosbridge(sysId: Int, ip: InetAddress) {
        if (rosbridgeWs.containsKey(sysId)) return  // already connected
        val port = 9090  // both rovers on same port (different Jetsons)
        val ns = "/rv$sysId"
        val url = "ws://${ip.hostAddress}:$port"
        Log.e("ROSBRIDGE", "Connecting to $url for RV$sysId telemetry")

        val request = okhttp3.Request.Builder().url(url).build()
        val ws = rosbridgeClient.newWebSocket(request, object : okhttp3.WebSocketListener() {
            override fun onOpen(webSocket: okhttp3.WebSocket, response: okhttp3.Response) {
                Log.e("ROSBRIDGE", "Connected to RV$sysId rosbridge")
                rosbridgeConnected[sysId] = true
                // Subscribe to topics — no type field (rosbridge auto-detects)
                val topics = listOf("center_pos", "heading", "nav_status",
                    "wp_active", "xte", "armed", "rerouted_path",
                    "rc_input", "cmd_override", "rtk_status", "sensors",
                    "reroute_pending", "mission_fence")
                for (t in topics) {
                    webSocket.send("""{"op":"subscribe","topic":"$ns/$t"}""")
                }
                Log.e("ROSBRIDGE", "Subscribed to ${topics.size} topics on $ns")
            }
            override fun onMessage(webSocket: okhttp3.WebSocket, text: String) {
                // Update heartbeat watchdog — any rosbridge message proves rover is alive
                roverLastHb[sysId] = System.currentTimeMillis()
                if (roverConnected[sysId] != true) {
                    roverConnected[sysId] = true
                    scope.launch(Dispatchers.Main) { onConnectionChange(sysId, true) }
                }
                try {
                    val json = org.json.JSONObject(text)
                    val topic = json.optString("topic", "")
                    val msg = json.optJSONObject("msg") ?: run {
                        Log.e("ROSBRIDGE", "No msg in: ${text.take(200)}")
                        return
                    }
                    if (!topic.endsWith("/heading") && !topic.endsWith("/center_pos") && !topic.endsWith("/xte"))
                        Log.e("ROSBRIDGE", "RX: $topic")
                    when {
                        topic.endsWith("/nav_status") -> {
                            val status = msg.optString("data", "NA")
                            scope.launch(Dispatchers.Main) { onNavStatus(sysId, status) }
                        }
                        topic.endsWith("/center_pos") -> {
                            val lat = msg.optDouble("latitude", 0.0)
                            val lon = msg.optDouble("longitude", 0.0)
                            if (lat != 0.0 && lon != 0.0) {
                                val hdg = rosbridgeHeading.getOrDefault(sysId, 0f)
                                scope.launch(Dispatchers.Main) { onPositionUpdate(sysId, lat, lon, hdg) }
                            }
                        }
                        topic.endsWith("/heading") -> {
                            val hdg = msg.optDouble("data", 0.0).toFloat()
                            rosbridgeHeading[sysId] = hdg
                        }
                        topic.endsWith("/wp_active") -> {
                            val wp = msg.optInt("data", -1)
                            scope.launch(Dispatchers.Main) { onMissionProgress(sysId, wp) }
                        }
                        topic.endsWith("/armed") -> {
                            val armed = msg.optBoolean("data", false)
                            roverArmed[sysId] = armed
                            scope.launch(Dispatchers.Main) { onArmState(sysId, armed) }
                        }
                        topic.endsWith("/xte") -> {
                            // XTE available via rosbridge
                        }
                        topic.endsWith("/rc_input") -> {
                            val chArr = msg.optJSONArray("channels")
                            if (chArr != null) {
                                val channels = IntArray(chArr.length()) { chArr.optInt(it, 1500) }
                                val remapped = remapSbusToLogical(channels)
                                // Store CH9 for autonomous detection
                                val ch9 = if (remapped.size > 8) remapped[8] else 1500
                                roverCh9[sysId] = ch9
                                // Show RC channels only when not in autonomous
                                val isAuto = msg.optString("mode", "").contains("AUTO", true)
                                if (!isAuto) {
                                    roverPpmChannels[sysId] = remapped
                                    scope.launch(Dispatchers.Main) { onRcChannels(sysId, remapped) }
                                }
                            }
                            val sbusOk = msg.optBoolean("sbus_ok", true)
                            val rfOk = msg.optBoolean("rf_link_ok", true)
                            scope.launch(Dispatchers.Main) {
                                onLinkStatus(sysId, "SBUS", sbusOk)
                                onLinkStatus(sysId, "RF", rfOk)
                            }
                        }
                        topic.endsWith("/rtk_status") -> {
                            val rtk = msg.optString("data", "")
                            val fixType = when {
                                rtk.contains("RTK_FIX", true) || rtk == "6" -> 6
                                rtk.contains("RTK_FLT", true) || rtk == "5" -> 5
                                rtk.contains("DGPS", true) || rtk == "4" -> 4
                                rtk.contains("3D", true) || rtk == "3" -> 3
                                rtk.contains("GPS", true) || rtk == "1" -> 1
                                else -> 0
                            }
                            scope.launch(Dispatchers.Main) { onGpsStatus(sysId, fixType) }
                        }
                        topic.endsWith("/sensors") -> {
                            // SensorData: tank_level, temperature, humidity, pressure
                            val tank = msg.optDouble("tank_level", 0.0).toFloat()
                            val temp = msg.optDouble("temperature", 0.0).toFloat()
                            val humid = msg.optDouble("humidity", 0.0).toFloat()
                            // battery from tank_level for now
                            scope.launch(Dispatchers.Main) {
                                onSensorUpdate(sysId, 0f, temp, tank, humid)
                            }
                        }
                        topic.endsWith("/reroute_pending") -> {
                            val pending = msg.optBoolean("data", false)
                            scope.launch(Dispatchers.Main) { onReroutePending(sysId, pending) }
                        }
                        topic.endsWith("/mission_fence") -> {
                            // Parse {"polygons": [[[lat,lon],...], ...]}
                            val jsonStr = msg.optString("data", "")
                            if (jsonStr.isNotEmpty()) {
                                try {
                                    val obj = org.json.JSONObject(jsonStr)
                                    val polys = obj.optJSONArray("polygons")
                                    if (polys != null) {
                                        val obstacles = mutableListOf<List<Pair<Double, Double>>>()
                                        for (i in 0 until polys.length()) {
                                            val poly = polys.getJSONArray(i)
                                            val verts = mutableListOf<Pair<Double, Double>>()
                                            for (j in 0 until poly.length()) {
                                                val pt = poly.getJSONArray(j)
                                                verts.add(Pair(pt.getDouble(0), pt.getDouble(1)))
                                            }
                                            obstacles.add(verts)
                                        }
                                        scope.launch(Dispatchers.Main) { onObstaclesReceived(sysId, obstacles) }
                                    }
                                } catch (e: Exception) {
                                    Log.e("ROSBRIDGE", "mission_fence parse: ${e.message}")
                                }
                            }
                        }
                        topic.endsWith("/cmd_override") -> {
                            // Autonomous output — already in PPM order, show when in AUTO
                            val mode = msg.optString("mode", "")
                            if (mode.contains("AUTO", true)) {
                                val chArr = msg.optJSONArray("channels")
                                if (chArr != null && chArr.length() >= 8) {
                                    // Build 9-ch array: cmd channels[0..7] + CH9 from rc_input
                                    val ch = IntArray(9) { if (it < chArr.length()) chArr.optInt(it, 1500) else 1500 }
                                    ch[8] = roverCh9[sysId] ?: 1500  // keep CH9 from RC
                                    roverPpmChannels[sysId] = ch
                                    scope.launch(Dispatchers.Main) { onRcChannels(sysId, ch) }
                                }
                            }
                        }
                        topic.endsWith("/rerouted_path") -> {
                            // Parse [[lat, lon, bypass, speed, hold_secs], ...]
                            val jsonStr = msg.optString("data", "")
                            Log.e("ROSBRIDGE", "rerouted_path data len=${jsonStr.length}")
                            if (jsonStr.isNotEmpty()) {
                                try {
                                    val arr = org.json.JSONArray(jsonStr)
                                    val path = mutableListOf<Triple<Double, Double, Boolean>>()
                                    val speeds = mutableListOf<Float>()
                                    for (j in 0 until arr.length()) {
                                        val pt = arr.getJSONArray(j)
                                        path.add(Triple(pt.getDouble(0), pt.getDouble(1), pt.getInt(2) != 0))
                                        speeds.add(if (pt.length() > 3) pt.getDouble(3).toFloat() else 0f)
                                    }
                                    Log.e("ROSBRIDGE", "rerouted_path parsed: ${path.size} pts")
                                    scope.launch(Dispatchers.Main) { onReroutedPath(sysId, path, speeds) }
                                } catch (e: Exception) {
                                    Log.e("ROSBRIDGE", "rerouted_path parse ERROR: ${e.message}")
                                    Log.e("ROSBRIDGE", "rerouted_path first 200: ${jsonStr.take(200)}")
                                }
                            }
                        }
                    }
                } catch (e: Exception) {
                    Log.e("ROSBRIDGE", "Parse error: ${e.message}")
                }
            }
            override fun onFailure(webSocket: okhttp3.WebSocket, t: Throwable, response: okhttp3.Response?) {
                Log.e("ROSBRIDGE", "RV$sysId connection failed: ${t.message}")
                rosbridgeWs.remove(sysId)
                rosbridgeConnected[sysId] = false
                roverConnected[sysId] = false
                scope.launch(Dispatchers.Main) {
                    onConnectionChange(sysId, false)
                    onLinkStatus(sysId, "SBUS", false)
                    onLinkStatus(sysId, "RF", false)
                }
                // Reconnect after 5s
                scope.launch { delay(5000); roverAddresses[sysId]?.let { connectRosbridge(sysId, it) } }
            }
            override fun onClosed(webSocket: okhttp3.WebSocket, code: Int, reason: String) {
                rosbridgeWs.remove(sysId)
                rosbridgeConnected[sysId] = false
                roverConnected[sysId] = false
                scope.launch(Dispatchers.Main) {
                    onConnectionChange(sysId, false)
                    onLinkStatus(sysId, "SBUS", false)
                    onLinkStatus(sysId, "RF", false)
                }
            }
        })
        rosbridgeWs[sysId] = ws
    }

    // Cache heading from rosbridge (updated faster than position callback)
    private val rosbridgeHeading = HashMap<Int, Float>()
    private val roverCh9 = HashMap<Int, Int>()  // CH9 from rc_input for rover select
    private val roverArmed = HashMap<Int, Boolean>()
    // Per-rover rosbridge connection state — when true, MAVLink GPS/STATUS/WP are ignored
    private val rosbridgeConnected = HashMap<Int, Boolean>()

    // ─── rosbridge command helpers ─────────────────────────────────────────

    /** Publish arm/disarm via rosbridge (TCP reliable). Falls back to MAVLink if not connected. */
    fun rosbridgeArm(sysId: Int, arm: Boolean) {
        val ws = rosbridgeWs[sysId]
        if (ws == null) {
            // Fallback to MAVLink
            if (arm) sendCommand(sysId, 400, 1f, 0f)
            else sendCriticalCommand(sysId, 400, 0f, 0f)
            return
        }
        val ns = "/rv$sysId"
        ws.send("""{"op":"publish","topic":"$ns/armed","type":"std_msgs/msg/Bool","msg":{"data":$arm}}""")
    }

    /** Publish mode via rosbridge (TCP reliable). Falls back to MAVLink if not connected. */
    fun rosbridgeSetMode(sysId: Int, mode: String) {
        val ws = rosbridgeWs[sysId]
        if (ws == null) {
            // Fallback to MAVLink
            val p2 = if (mode == "MANUAL") 0f else 1f
            sendCommand(sysId, 176, 0f, p2)
            return
        }
        val ns = "/rv$sysId"
        ws.send("""{"op":"publish","topic":"$ns/mode","type":"std_msgs/msg/String","msg":{"data":"$mode"}}""")
    }

    /** Clear mission on rover via rosbridge. */
    fun rosbridgeClearMission(sysId: Int) {
        val ws = rosbridgeWs[sysId] ?: return
        val ns = "/rv$sysId"
        ws.send("""{"op":"publish","topic":"$ns/mission_clear","type":"std_msgs/msg/Bool","msg":{"data":true}}""")
        // Also disarm
        rosbridgeArm(sysId, false)
    }

    /** Send reroute confirmation via rosbridge. */
    fun rosbridgeRerouteResponse(sysId: Int, accept: Boolean) {
        val ws = rosbridgeWs[sysId] ?: return
        val ns = "/rv$sysId"
        ws.send("""{"op":"publish","topic":"$ns/reroute_response","type":"std_msgs/msg/Bool","msg":{"data":$accept}}""")
    }

    /** Send station coordinates via rosbridge. */
    fun rosbridgeStationUpdate(sysId: Int, type: String, lat: Double, lon: Double) {
        val ws = rosbridgeWs[sysId] ?: return
        val ns = "/rv$sysId"
        val json = """{"type":"$type","lat":$lat,"lon":$lon}"""
        ws.send("""{"op":"publish","topic":"$ns/station_update","type":"std_msgs/msg/String","msg":{"data":"${json.replace("\"", "\\\"")}"}}""")
    }

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

        // Discovery beacon listener — rovers broadcast JSON on UDP 5555
        scope.launch {
            try {
                val beaconSocket = DatagramSocket(5555).also { it.broadcast = true }
                Log.i("RoverMgr", "Listening for discovery beacons on UDP 5555")
                val buf = ByteArray(256)
                while (isRunning) {
                    val pkt = DatagramPacket(buf, buf.size)
                    beaconSocket.receive(pkt)
                    try {
                        val json = String(pkt.data, 0, pkt.length)
                        val obj = org.json.JSONObject(json)
                        val sysId = obj.optInt("sysid", 0)
                        val rbPort = obj.optInt("rosbridge", 9090)
                        if (sysId > 0 && !roverAddresses.containsKey(sysId)) {
                            roverAddresses[sysId] = pkt.address
                            Log.i("RoverMgr", "Discovered RV$sysId at ${pkt.address.hostAddress} (beacon)")
                            connectRosbridge(sysId, pkt.address)
                            scope.launch(Dispatchers.Main) { onConnectionChange(sysId, true) }
                        }
                    } catch (_: Exception) {}
                }
                beaconSocket.close()
            } catch (e: Exception) {
                Log.e("RoverMgr", "Beacon listener error: ${e.message}")
            }
        }

        // MAVLink UDP — kept for legacy/fallback discovery
        scope.launch {
            try {
                socket = DatagramSocket(PORT).also { it.broadcast = true }
                Log.i("RoverMgr", "Listening on UDP $PORT — MAVLink fallback")

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
        rosbridgeWs.values.forEach { it.close(1000, "stopping") }
        rosbridgeWs.clear()
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
        // Route ALL commands through rosbridge when connected
        val targets = if (sysId == 0) listOf(MASTER_SYSID, SLAVE_SYSID) else listOf(sysId)
        for (t in targets) {
            if (rosbridgeConnected[t] != true) continue
            when (commandId) {
                400 -> { rosbridgeArm(t, p1 >= 1f); return }
                176 -> { rosbridgeSetMode(t, if (p2 == 0f) "MANUAL" else "AUTONOMOUS"); return }
                50001 -> { rosbridgeStationUpdate(t, "recharge", (p1 / 1e5).toDouble(), (p2 / 1e5).toDouble()); return }
                50002 -> { rosbridgeStationUpdate(t, "water", (p1 / 1e5).toDouble(), (p2 / 1e5).toDouble()); return }
                50003 -> { rosbridgeRerouteResponse(t, p1 >= 1f); return }
            }
        }
        // Fallback: MAVLink (only if rosbridge not connected)
        scope.launch {
            val ip = roverIp(sysId)
            repeat(2) { attempt ->
                sendMavlinkTo(ip, CommandLong.builder()
                    .targetSystem(sysId).targetComponent(1)
                    .command(EnumValue.create(MavCmd::class.java, commandId))
                    .confirmation(attempt)
                    .param1(p1).param2(p2).param3(0f).param4(0f)
                    .param5(0f).param6(0f).param7(0f)
                    .build())
                if (attempt < 1) delay(80L)
            }
        }
    }

    /**
     * Send COMMAND_LONG 3× at 100 ms intervals for safety-critical commands
     * (E-STOP, disarm) where a single dropped UDP packet could be fatal.
     * Uses `confirmation` field 0, 1, 2 per MAVLink spec for retries.
     */
    fun sendCriticalCommand(sysId: Int, commandId: Int, p1: Float, p2: Float) {
        // Route ARM/DISARM through rosbridge when connected (TCP = no packet loss)
        val targets = if (sysId == 0) listOf(MASTER_SYSID, SLAVE_SYSID) else listOf(sysId)
        for (t in targets) {
            if (rosbridgeConnected[t] == true && commandId == 400) {
                rosbridgeArm(t, p1 >= 1f)
            }
        }
        // Always also send via MAVLink for safety-critical commands (belt + suspenders)
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
        uploadJobs[sysId]?.cancel()
        uploadJobs[sysId] = scope.launch {
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
        uploadJobs[sysId]?.cancel()
        uploadJobs[sysId] = scope.launch {
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
     * Upload a corridor mission: list of corridor centerlines with per-point speed.
     * Each centerline vertex is sent as cmd=50100 with corridor metadata.
     * Headland crossings are auto-generated by the navigator from the corridor chain.
     *
     * @param corridors  List of corridors, each a list of (LatLng, speed) pairs.
     *                   Speed = recorded speed (m/s); 0 = use navigator default.
     */
    fun uploadCorridorMission(sysId: Int, corridors: List<List<Pair<LatLng, Float>>>, width: Float,
                              servos: Map<Int, Int> = emptyMap(),
                              servoEvents: Map<Int, List<MissionAction.ServoCmd>> = emptyMap()) {
        uploadJobs[sysId]?.cancel()
        uploadJobs[sysId] = scope.launch {
            var seq = 0
            val items = mutableListOf<MissionItemInt>()
            // Prepend initial servo state (CH5-CH8)
            for (servoNum in 5..8) {
                val pwm = servos[servoNum] ?: 1500
                items.add(MissionItemInt.builder()
                    .seq(seq++).targetSystem(sysId).targetComponent(1)
                    .frame(MavFrame.MAV_FRAME_MISSION)
                    .command(MavCmd.MAV_CMD_DO_SET_SERVO)
                    .current(0).autocontinue(1)
                    .param1(servoNum.toFloat()).param2(pwm.toFloat())
                    .param3(0f).param4(0f).x(0).y(0).z(0f)
                    .build())
            }
            var wpIdx = 0
            for ((idx, centerline) in corridors.withIndex()) {
                val nVerts = centerline.size
                val nextId = if (idx < corridors.size - 1) idx + 1 else 65535
                for ((pt, speed) in centerline) {
                    // Corridor vertex first
                    items.add(MissionItemInt.builder()
                        .seq(seq++).targetSystem(sysId).targetComponent(1)
                        .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                        .command(EnumValue.create(MavCmd::class.java, 50100))
                        .current(0).autocontinue(1)
                        .param1(nVerts.toFloat())
                        .param2(width)
                        .param3(speed)
                        .param4(nextId.toFloat())
                        .x((pt.latitude * 1e7).toInt())
                        .y((pt.longitude * 1e7).toInt())
                        .z(idx.toFloat())
                        .build())
                    // Servo commands AFTER the vertex they apply to
                    servoEvents[wpIdx]?.forEach { cmd ->
                        items.add(MissionItemInt.builder()
                            .seq(seq++).targetSystem(sysId).targetComponent(1)
                            .frame(MavFrame.MAV_FRAME_MISSION)
                            .command(MavCmd.MAV_CMD_DO_SET_SERVO)
                            .current(0).autocontinue(1)
                            .param1(cmd.servo.toFloat()).param2(cmd.pwm.toFloat())
                            .param3(0f).param4(0f).x(0).y(0).z(0f)
                            .build())
                    }
                    wpIdx++
                }
            }
            streamMission(sysId, items)
        }
    }

    // ── rosbridge WebSocket upload ─────────────────────────────────────────
    // Publishes corridor JSON directly on the ROS2 topic via rosbridge.
    // TCP reliable delivery — no packet loss, no retries.

    /**
     * Upload raw recorded points via rosbridge WebSocket (TCP).
     * GQC sends only raw data — the rover handles all processing
     * (auto_split_corridors, corridors_to_path, speed smoothing).
     *
     * JSON format published on /rvN/corridor_mission:
     *   {"corridors":[{"corridor_id":0,"centerline":[[lat,lon],...],"width":W,
     *     "speeds":[...],"ch5":[...],"ch6":[...],"ch7":[...],"ch8":[...],
     *     "next_corridor_id":-1}],"min_turn_radius":3.0}
     *
     * Single corridor with all raw points — navigator auto-splits at turns.
     */
    fun uploadRawRecording(
        sysId: Int,
        actions: List<MissionAction>,
        width: Float,
    ) {
        val ns = "/rv$sysId"
        val port = 9090  // both rovers on same port (different Jetsons)
        val addr = roverAddresses[sysId]
        if (addr == null) {
            scope.launch(Dispatchers.Main) { onMissionAck("No rover $sysId IP — connect first") }
            return
        }
        scope.launch {
            try {
                // Extract flat arrays from the interleaved Waypoint+ServoCmd list
                val lats = mutableListOf<Double>()
                val lons = mutableListOf<Double>()
                val speeds = mutableListOf<Float>()
                val ch5 = mutableListOf<Int>(); val ch6 = mutableListOf<Int>()
                val ch7 = mutableListOf<Int>(); val ch8 = mutableListOf<Int>()
                var curServo = intArrayOf(1500, 1500, 1500, 1500)  // running state

                for (action in actions) {
                    when (action) {
                        is MissionAction.Waypoint -> {
                            lats.add(action.lat); lons.add(action.lon)
                            speeds.add(action.speed)
                            // Snapshot current servo state for this point
                            ch5.add(curServo[0]); ch6.add(curServo[1])
                            ch7.add(curServo[2]); ch8.add(curServo[3])
                        }
                        is MissionAction.ServoCmd -> {
                            // Update running servo state (applied to NEXT waypoint,
                            // or retroactively to the last one if it follows immediately)
                            when (action.servo) {
                                5 -> curServo[0] = action.pwm
                                6 -> curServo[1] = action.pwm
                                7 -> curServo[2] = action.pwm
                                8 -> curServo[3] = action.pwm
                            }
                            // Update the last waypoint's servo (ServoCmd follows its Waypoint)
                            if (ch5.isNotEmpty()) {
                                ch5[ch5.lastIndex] = curServo[0]
                                ch6[ch6.lastIndex] = curServo[1]
                                ch7[ch7.lastIndex] = curServo[2]
                                ch8[ch8.lastIndex] = curServo[3]
                            }
                        }
                    }
                }

                if (lats.isEmpty()) {
                    withContext(Dispatchers.Main) { onMissionAck("No waypoints recorded") }
                    return@launch
                }

                // Build single-corridor JSON (rover will auto-split at turn markers)
                val centerline = lats.zip(lons).joinToString(",") { "[${it.first},${it.second}]" }
                val corridorJson = """{"corridors":[{"corridor_id":0,"centerline":[$centerline],"width":$width,"speed":0,"speeds":[${speeds.joinToString(",")}],"ch5":[${ch5.joinToString(",")}],"ch6":[${ch6.joinToString(",")}],"ch7":[${ch7.joinToString(",")}],"ch8":[${ch8.joinToString(",")}],"next_corridor_id":-1,"turn_type":"auto","headland_width":0}],"min_turn_radius":3.0,"headland_width":0}"""

                // Publish via rosbridge WebSocket
                val client = okhttp3.OkHttpClient.Builder()
                    .connectTimeout(5, java.util.concurrent.TimeUnit.SECONDS)
                    .build()
                val request = okhttp3.Request.Builder()
                    .url("ws://${addr.hostAddress}:$port")
                    .build()
                val latch = java.util.concurrent.CountDownLatch(1)
                var connected = false

                client.newWebSocket(request, object : okhttp3.WebSocketListener() {
                    override fun onOpen(webSocket: okhttp3.WebSocket, response: okhttp3.Response) {
                        connected = true
                        val escaped = corridorJson.replace("\\", "\\\\").replace("\"", "\\\"")
                        webSocket.send("""{"op":"publish","topic":"$ns/corridor_mission","type":"std_msgs/msg/String","msg":{"data":"$escaped"}}""")
                        Log.e("ROSBRIDGE", "Published raw recording (${lats.size} pts, ${corridorJson.length} bytes)")
                        scope.launch { delay(500); webSocket.close(1000, "done"); latch.countDown() }
                    }
                    override fun onFailure(webSocket: okhttp3.WebSocket, t: Throwable, response: okhttp3.Response?) {
                        Log.e("ROSBRIDGE", "WebSocket failed: ${t.message}")
                        latch.countDown()
                    }
                })

                latch.await(10, java.util.concurrent.TimeUnit.SECONDS)
                client.dispatcher.executorService.shutdown()

                withContext(Dispatchers.Main) {
                    if (connected) onMissionAck("Uploaded ${lats.size} raw points via rosbridge")
                    else onMissionAck("rosbridge connection failed — is rover running?")
                }
            } catch (e: Exception) {
                Log.e("ROSBRIDGE", "Upload error: ${e.message}")
                withContext(Dispatchers.Main) { onMissionAck("rosbridge error: ${e.message}") }
            }
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
        uploadJobs[sysId]?.cancel()
        uploadJobs[sysId] = scope.launch {
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
        Log.i("RoverMgr", "streamMission START: sysId=$sysId, ${items.size} items")
        try {
        pendingMissions[sysId] = items
        val ip = roverIp(sysId)
        Log.i("RoverMgr", "streamMission target IP: $ip")
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
        Log.i("RoverMgr", "streamMission DISARM sent")
        // Allow rover to process disarm before mission upload begins.
        delay(100L)
        // Send MISSION_COUNT 3× to survive WiFi packet loss — rover ignores
        // duplicates (second MISSION_COUNT resets the same upload state).
        repeat(3) { attempt ->
            sendMavlinkTo(ip, MissionCount.builder()
                .targetSystem(sysId).targetComponent(1)
                .count(items.size).build())
            if (attempt < 2) delay(50L)
        }
        // Brief pause so rover processes MISSION_COUNT before items start arriving.
        delay(100L)
        Log.i("RoverMgr", "streamMission MISSION_COUNT sent, streaming ${items.size} items")
        for (item in items) {
            sendMavlinkTo(ip, item)
            delay(50L)   // 50 ms gap: gives WiFi driver time to transmit each packet
        }
        Log.i("RoverMgr", "streamMission DONE: all ${items.size} items sent")
        } catch (e: Exception) {
            Log.e("RoverMgr", "streamMission CRASHED: ${e.message}", e)
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
                    // Auto-connect rosbridge for telemetry
                    connectRosbridge(senderId, packet.address)
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
                // Arm state from rosbridge /armed topic when connected
                if (rosbridgeConnected[senderId] != true) {
                    val armed = (payload.baseMode().value() and 128) != 0
                    scope.launch(Dispatchers.Main) { onArmState(senderId, armed) }
                }

                // Check state consistency on every heartbeat from either rover
                if (senderId == SLAVE_SYSID || senderId == MASTER_SYSID) checkLinkMismatch()
            }

            // GLOBAL_POSITION_INT (#33) — rover GPS position
            // Skipped when rosbridge provides center_pos (avoids position jumping)
            is GlobalPositionInt -> {
                if (rosbridgeConnected[senderId] == true) return
                val tMs = payload.timeBootMs().toLong() and 0xFFFFFFFFL
                val lastTms = roverLastPosTms[senderId] ?: -1L
                if (tMs <= lastTms && lastTms - tMs < 30_000L) return
                roverLastPosTms[senderId] = tMs
                val lat = payload.lat() / 1e7
                val lon = payload.lon() / 1e7
                if (lat == 0.0 && lon == 0.0) return
                val hdg = if (payload.hdg() == 65535) 0f else payload.hdg() / 100f
                scope.launch(Dispatchers.Main) { onPositionUpdate(senderId, lat, lon, hdg) }
            }

            // MISSION_REQUEST_INT — rover asks for a specific waypoint during upload
            is MissionRequestInt -> sendMissionItem(senderId, payload.seq())

            // MISSION_COUNT — rover is advertising its current mission for download
            is MissionCount -> {
                val count = payload.count()
                missionDownloadItems[senderId] = HashMap()
                missionDownloadCount[senderId] = count
                Log.i("RoverMgr", "Mission download start: rover $senderId, $count items")
                // count==0 means empty mission — complete immediately
                if (count == 0) {
                    missionDownloadItems.remove(senderId)
                    missionDownloadCount.remove(senderId)
                    scope.launch(Dispatchers.Main) { onMissionDownloaded(senderId, emptyList()) }
                }
            }

            // MISSION_ITEM_INT — rover is sending individual items during download
            is MissionItemInt -> {
                val items = missionDownloadItems[senderId] ?: return
                val expected = missionDownloadCount[senderId] ?: return
                val lat = payload.x() / 1e7
                val lon = payload.y() / 1e7
                val bypass = payload.param2() > 0.5f  // param2 = bypass flag
                items[payload.seq()] = Triple(lat, lon, bypass)
                if (items.size >= expected) {
                    val mission = (0 until expected).mapNotNull { items[it] }
                    missionDownloadItems.remove(senderId)
                    missionDownloadCount.remove(senderId)
                    sendMavlinkTo(roverIp(senderId), MissionAck.builder()
                        .targetSystem(senderId).targetComponent(1)
                        .type(EnumValue.of(MavMissionResult.MAV_MISSION_ACCEPTED))
                        .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION))
                        .build())
                    Log.i("RoverMgr", "Mission download done: rover $senderId, ${mission.size} WPs")
                    scope.launch(Dispatchers.Main) { onMissionDownloaded(senderId, mission) }
                }
            }

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
                // Discard out-of-order/duplicate packets (broadcast + unicast both arrive).
                // Per-name tracking: all names in one batch share the same timestamp, so a
                // per-rover guard would drop every name after the first in each batch.
                // Exception: a large backward jump (> 30 s) means the rover restarted.
                val tMs  = payload.timeBootMs().toLong() and 0xFFFFFFFFL
                val name = payload.name().trimEnd('\u0000')
                val perRover = roverLastNvfTms.getOrPut(senderId) { HashMap() }
                val lastTms  = perRover[name] ?: -1L
                if (tMs <= lastTms && lastTms - tMs < 30_000L) return
                perRover[name] = tMs
                val value = payload.value()
                when (name) {
                    "TANK"   -> scope.launch(Dispatchers.Main) {
                        onSensorUpdate(senderId, -1f, -1f, value, -1f)
                    }
                    "HUMID"  -> scope.launch(Dispatchers.Main) {
                        onSensorUpdate(senderId, -1f, -1f, -1f, value)
                    }
                    "STATUS"  -> {
                        // Skip MAVLink STATUS when rosbridge provides nav_status
                        if (rosbridgeConnected[senderId] == true) {}
                        else {
                            val s = when {
                                value >= 2f -> "ARM"
                                value >= 1f -> "MSL"
                                else        -> "NA"
                            }
                            scope.launch(Dispatchers.Main) { onNavStatus(senderId, s) }
                        }
                    }
                    "SBUS_OK" -> scope.launch(Dispatchers.Main) {
                        onLinkStatus(senderId, "SBUS", value >= 1f)
                    }
                    "RF_OK"   -> scope.launch(Dispatchers.Main) {
                        onLinkStatus(senderId, "RF", value >= 1f)
                    }
                    "RTK"     -> scope.launch(Dispatchers.Main) {
                        onGpsStatus(senderId, value.toInt())
                    }
                    "HACC"    -> scope.launch(Dispatchers.Main) {
                        onHaccStatus(senderId, value)
                    }
                    "WP_ACT"  -> {
                        if (rosbridgeConnected[senderId] != true)
                            scope.launch(Dispatchers.Main) { onMissionProgress(senderId, value.toInt()) }
                    }
                    "MSN_ID" -> {
                        val newId = value.toInt()
                        val oldId = roverMsnId[senderId] ?: -1
                        if (newId != oldId) {
                            roverMsnId[senderId] = newId
                            if (newId == 0) {
                                // Mission cleared on rover
                                scope.launch(Dispatchers.Main) { onMissionDownloaded(senderId, emptyList()) }
                            } else {
                                scope.launch(Dispatchers.IO) { requestMissionFromRover(senderId) }
                            }
                        }
                    }
                    "REROUTE" -> {
                        scope.launch(Dispatchers.Main) {
                            onReroutePending(senderId, value > 0.5f)
                        }
                    }
                }
            }

            // RC_CHANNELS (#65) — raw SBUS channel values from the RP2040 (MK32 order).
            // mavlink_bridge sends all 16 SBUS channels in SBUS/MK32 order.
            // We remap to logical/PPM order so channel indices match the hardware PPM map.
            is RcChannels -> {
                // Discard out-of-order/duplicate packets (sent at 10 Hz; broadcast + unicast
                // both arrive, so each real update generates two packets with the same timestamp).
                // Exception: a large backward jump (> 30 s) means the rover restarted.
                val tMs = payload.timeBootMs().toLong() and 0xFFFFFFFFL
                val lastRc = roverLastRcTms[senderId] ?: -1L
                if (tMs <= lastRc && lastRc - tMs < 30_000L) return
                roverLastRcTms[senderId] = tMs
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

            // PARAM_VALUE (#22) — response to PARAM_REQUEST_LIST or PARAM_SET
            is ParamValue -> {
                val name  = payload.paramId().trimEnd('\u0000')
                val value = payload.paramValue()
                val index = payload.paramIndex()
                val count = payload.paramCount()
                scope.launch(Dispatchers.Main) { onParamValue(senderId, name, value, index, count) }
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
                        val speeds = mutableListOf<Float>()
                        for (i in 0 until arr.length()) {
                            val pt = arr.getJSONArray(i)
                            path.add(Triple(pt.getDouble(0), pt.getDouble(1), pt.getInt(2) != 0))
                            speeds.add(if (pt.length() > 3) pt.getDouble(3).toFloat() else 0f)
                        }
                        scope.launch(Dispatchers.Main) { onReroutedPath(senderId, path, speeds) }
                    } catch (e: Exception) {
                        Log.w("RoverMgr", "TUNNEL JSON parse error: ${e.message}")
                    }
                }
            }
        }
    }

    // ─── Mismatch detection ───────────────────────────────────────────────────

    /**
     * Compares the master rover's reported mode (from HEARTBEAT flags) against
     * what the slave rover is actually applying (from slave's HEARTBEAT flags).
     *
     * Both sides use HEARTBEAT — not RC_CHANNELS — because in AUTO mode the master
     * RP2040 outputs navigator-commanded PPM values on serial (commit 37dc7e5), not
     * raw switch positions.  Reading SWA/SWB from RC_CHANNELS in that state would
     * incorrectly see 1500 (neutral) and fire a false emergency mismatch.
     *
     * mavlink_bridge sets the flags correctly from the RP2040 mode string:
     *   MAV_STATE_EMERGENCY          → mode == 'EMERGENCY'
     *   MAV_MODE_FLAG_GUIDED_ENABLED → mode == 'AUTONOMOUS'
     *
     * Conditions checked:
     *   • Master in EMERGENCY but slave NOT in emergency  [safety-critical]
     *   • Master NOT autonomous but slave still running autonomous  [safety-critical]
     *   NOTE: master→AUTO but slave→IDLE is intentionally NOT flagged — slave is
     *   stopped (safe); it will go autonomous once its navigator connects.
     */
    private fun checkLinkMismatch() {
        val slaveConnected = (roverLastHb[SLAVE_SYSID] ?: 0L) >
                              System.currentTimeMillis() - 3000L
        // Only check mismatch when slave is connected; disconnection is
        // handled separately by the watchdog → onConnectionChange callback.
        if (!slaveConnected) { mismatchStart = null; return }

        val masterBase   = roverBaseMode[MASTER_SYSID]  ?: return
        val masterStatus = roverSysStatus[MASTER_SYSID] ?: return
        val masterEmergency  = masterStatus == 6               // MAV_STATE_EMERGENCY
        val masterAutonomous = (masterBase and 0x08) != 0      // MAV_MODE_FLAG_GUIDED_ENABLED

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

    private fun requestMissionFromRover(sysId: Int) {
        sendMavlinkTo(roverIp(sysId), MissionRequestList.builder()
            .targetSystem(sysId).targetComponent(1)
            .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION))
            .build())
        Log.i("RoverMgr", "Mission sync request → rover $sysId")
    }

    /** Request all navigator parameters from a rover (PARAM_REQUEST_LIST → stream of PARAM_VALUE). */
    fun requestParams(sysId: Int) {
        scope.launch {
            sendMavlinkTo(roverIp(sysId), ParamRequestList.builder()
                .targetSystem(sysId).targetComponent(1).build())
            Log.i("RoverMgr", "PARAM_REQUEST_LIST → rover $sysId")
        }
    }

    /** Send a PARAM_SET to change a single navigator parameter on a rover. */
    fun setParam(sysId: Int, name: String, value: Float) {
        scope.launch {
            sendMavlinkTo(roverIp(sysId), ParamSet.builder()
                .targetSystem(sysId).targetComponent(1)
                .paramId(name)
                .paramValue(value)
                .paramType(EnumValue.create(MavParamType::class.java, 9)) // REAL32
                .build())
            Log.i("RoverMgr", "PARAM_SET $name=$value → rover $sysId")
        }
    }

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
