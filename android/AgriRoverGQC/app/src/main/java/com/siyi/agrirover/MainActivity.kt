package com.siyi.agrirover

import android.content.Context
import android.content.res.ColorStateList
import android.graphics.*
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.View
import android.widget.*
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.OnMapReadyCallback
import com.google.android.gms.maps.SupportMapFragment
import com.google.android.gms.maps.model.*
import com.google.android.material.button.MaterialButton
import com.google.android.material.floatingactionbutton.FloatingActionButton
import android.os.SystemClock
import java.io.File
import kotlin.math.*

class MainActivity : AppCompatActivity(), OnMapReadyCallback {

    companion object {
        /** Must match navigator_params.yaml max_speed / min_speed. */
        const val MAX_SPEED_MPS = 1.5f
        const val MIN_SPEED_MPS = 0.3f
    }

    // --- ENUMS & STATE ---
    enum class AppMode { MANUAL, PLANNER, AUTO }

    private var currentMode = AppMode.MANUAL

    // --- MULTI-ROVER STATE ---
    private var selectedRoverId = 1
    private var roverExplicitlySelected = false  // true once CH9 goes low or high
    private val roverMarkers    = HashMap<Int, Marker>()
    private val roverPositions  = HashMap<Int, LatLng>()
    private val roverModes      = HashMap<Int, AppMode>()
    private val roverArmed      = HashMap<Int, Boolean>()
    // Cached icon state — only rebuild bitmap when armed/auto/selected actually changes
    private val roverIconKeys   = HashMap<Int, Triple<Boolean, Boolean, Boolean>>()
    private val roverNavStatus  = HashMap<Int, String>()   // "NA" | "MSL" | "ARM"
    private val roverGpsFix    = HashMap<Int, Int>()       // GPS_RAW_INT fixType (0–6); 6=RTK_FIX
    private val roverHaccMm    = HashMap<Int, Float>()     // UBX NAV-PVT hAcc in mm; -1 = not received
    private val roverPpmChannels = HashMap<Int, IntArray>()   // latest RC_CHANNELS per rover

    // Per-rover uploaded missions (synced from rover via MSN_ID)
    private val roverMissions             = HashMap<Int, List<LatLng>>()
    private val roverMissionBypass        = HashMap<Int, List<Boolean>>()  // parallel: is bypass WP?
    private val roverMissionVisible       = HashMap<Int, Boolean>()
    private val roverMissionOverlays      = HashMap<Int, MutableList<Any>>()
    private val roverNextWaypointIndex    = HashMap<Int, Int>()   // last MISSION_ITEM_REACHED seq per rover

    // Per-rover rerouted path received from navigator via TUNNEL (after obstacle avoidance)
    // Triple: (lat, lon, isBypass)
    private val roverReroutedPaths   = HashMap<Int, List<Triple<Double, Double, Boolean>>>()
    private val reroutedPathOverlays = HashMap<Int, MutableList<Any>>()

    // Live parameter tuning — EditText fields keyed by MAVLink param name;
    // populated when the Nav Params dialog is open.
    private val paramFields       = HashMap<String, android.widget.EditText>()
    private var paramDialogRoverId = 1
    private val PARAM_DEFS = listOf(
        Triple("MAX_SPEED",    "Max Speed",           "m/s"),
        Triple("MIN_SPEED",    "Min Speed",           "m/s"),
        Triple("LOOKAHEAD",    "Lookahead Dist",      "m"),
        Triple("STANLEY_K",    "Stanley K",           ""),
        Triple("PIVOT_THRESH", "Pivot Threshold",     "°"),
        Triple("PIVOT_DIST",   "Pivot Approach Dist", "m"),
        Triple("ALIGN_THRESH", "Align Threshold",     "°"),
        Triple("ACCEPT_RAD",   "Acceptance Radius",   "m"),
        Triple("AFS_MIN_THR",  "AFS Min Throttle",    "µs"),
        Triple("AFS_MIN_STR",  "AFS Min Steer Δ",     "µs"),
        Triple("AFS_COAST_ANG", "AFS Coast Angle",    "°"),
    )

    // UI Elements
    private lateinit var map: GoogleMap
    private lateinit var touchOverlay: View
    private lateinit var btnModeMenu: Button

    // Per-rover HUD panels
    private lateinit var dotRv1Hb:    TextView
    private lateinit var dotRv2Hb:    TextView
    private lateinit var dotRv1Sbus:  TextView
    private lateinit var dotRv2Rf:    TextView
    private lateinit var txtRv1Bat:   TextView
    private lateinit var txtRv1Temp:  TextView
    private lateinit var txtRv1Tank:  TextView
    private lateinit var txtRv1Rtk:    TextView
    private lateinit var txtRv1Status: TextView
    private lateinit var txtRv1Wp:    TextView
    private lateinit var txtRv2Bat:   TextView
    private lateinit var txtRv2Temp:  TextView
    private lateinit var txtRv2Tank:  TextView
    private lateinit var txtRv2Rtk:    TextView
    private lateinit var txtRv2Status: TextView
    private lateinit var txtRv2Wp:    TextView
    private lateinit var txtRcChannels:  TextView   // RC PPM strip
    private lateinit var btnPlannerMenu: ImageButton
    private lateinit var btnRec:         MaterialButton
    private lateinit var btnLayers:      FloatingActionButton
    private lateinit var btnCenter:      FloatingActionButton
    private lateinit var btnEStop:       Button

    // Auto Buttons
    private lateinit var btnStart: Button
    private lateinit var btnStop:  Button

    // Data
    private val routePoints      = ArrayList<LatLng>()
    private var nextWaypointIndex = 0
    private var waterPoint:       LatLng? = null
    private var batteryPoint:     LatLng? = null
    private var isReturningHome   = false
    private val routeOverlays     = mutableListOf<Any>()
    private var isFollowingRover  = false

    // --- RECORDING ---
    // recordedMission stores the full sequence including servo commands.
    // routePoints stores only waypoints (for map display and plain uploads).
    private var isRecording = false
    private val recordedMission = mutableListOf<MissionAction>()
    // Timestamps (SystemClock.elapsedRealtime()) for each Waypoint in recordedMission.
    // Parallel to waypoint entries only — servo commands have no timestamp.
    private val waypointTimestamps = mutableListOf<Long>()
    // Last PPM µs values for aux channels (PPM CH5–CH8, logical indices 4–7).
    // Initialised to 1500 (neutral) and updated when a >100 µs change is detected.
    private val lastAuxPwm = IntArray(4) { 1500 }

    // --- CORRIDOR RECORDING ---
    // Each REC cycle adds a corridor. UPLOAD sends all as a corridor mission.
    // Each corridor point = Triple(LatLng, speed m/s). Speed from smoothRecordedSpeeds().
    private var isCorridorMode = false
    private val corridorList = mutableListOf<List<Pair<LatLng, Float>>>()  // (position, speed)
    private var corridorWidth = 1.5f   // half-width in metres (default)

    // --- OBSTACLE DRAWING ---
    private var isDrawingObstacle = false
    private val currentObstacleDraft = mutableListOf<LatLng>()
    private val obstaclePolygons     = mutableListOf<MutableList<LatLng>>()
    private val obstacleOverlays     = mutableListOf<Polygon>()
    private var draftPolyline: Polyline? = null
    // Stationary hold tracking: last position where a waypoint was saved, and
    // accumulated stationary time. When rover moves again, the hold time is
    private var lastRecordedPos: LatLng? = null
    private val MIN_RECORD_DIST = 0.3   // metres — below this = skip (GPS noise)

    private val recordHandler = Handler(Looper.getMainLooper())
    private val recordRunnable = object : Runnable {
        override fun run() {
            if (!isRecording) return
            val pos = roverPositions[selectedRoverId]
            if (pos != null) {
                val last = lastRecordedPos
                val dist = if (last != null)
                    haversineMetres(last.latitude, last.longitude, pos.latitude, pos.longitude)
                else Double.MAX_VALUE

                if (dist >= MIN_RECORD_DIST) {
                    lastRecordedPos = pos
                    // Store waypoint with speed=0; actual speed computed post-hoc
                    // in smoothRecordedSpeeds() using sliding-window averaging.
                    recordedMission.add(MissionAction.Waypoint(pos.latitude, pos.longitude, 0f))
                    waypointTimestamps.add(SystemClock.elapsedRealtime())
                    routePoints.add(pos)
                    redrawMap()
                }
            }
            recordHandler.postDelayed(this, 500)
        }
    }

    /** Flat-earth haversine distance in metres. */
    private fun haversineMetres(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val R = 6_371_000.0
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = Math.sin(dLat / 2).pow(2) +
                Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
                Math.sin(dLon / 2).pow(2)
        return 2 * R * Math.asin(Math.sqrt(a))
    }

    // --- ROVER MANAGER ---
    private val roverManager = RoverPositionManager(
        onPositionUpdate = { sysId, lat, lon, hdg ->
            runOnUiThread { updateRover(sysId, lat, lon, hdg) }
        },
        onMissionAck = { msg ->
            runOnUiThread { Toast.makeText(this, msg, Toast.LENGTH_SHORT).show() }
        },
        onSensorUpdate = { sysId, bat, temp, tank, _ ->
            runOnUiThread { updateRoverSensors(sysId, bat, temp, tank) }
        },

        onArmState = { sysId, armed ->
            runOnUiThread {
                roverArmed[sysId] = armed
                val pos = roverPositions[sysId] ?: return@runOnUiThread
                updateRover(sysId, pos.latitude, pos.longitude,
                    roverMarkers[sysId]?.rotation ?: 0f)
            }
        },

        onMissionProgress = { sysId, seq ->
            runOnUiThread {
                roverNextWaypointIndex[sysId] = seq
                val total = roverMissions[sysId]?.size ?: 0
                val wpView = if (sysId == 1) txtRv1Wp else txtRv2Wp
                wpView.text = if (total > 0) "WP: ${seq + 1}/$total" else "WP: --"
                redrawRoverMissions()
                if (sysId == selectedRoverId) {
                    nextWaypointIndex = seq
                    redrawMap()
                }
            }
        },

        onCommandAck = { sysId, commandId, result ->
            runOnUiThread {
                if (result != 0) {
                    val reason = when (result) {
                        1 -> "Temporarily Rejected"
                        2 -> "Denied"
                        3 -> "Unsupported"
                        4 -> "Failed"
                        else -> "Error $result"
                    }
                    Toast.makeText(this,
                        "Rover $sysId cmd $commandId: $reason",
                        Toast.LENGTH_SHORT).show()
                }
            }
        },

        onConnectionChange = { sysId: Int, connected: Boolean ->
            runOnUiThread { updateHbDot(sysId, connected) }
            if (connected) sendSavedStations(sysId)
        },

        // RC_CHANNELS (#65) — PPM µs values from RP2040 (physical RC sticks via HM30/SBUS)
        onRcChannels = { sysId, channels ->
            runOnUiThread {
                roverPpmChannels[sysId] = channels
                // CH9 (index 8) selects active rover: <1250 → RV1, >1750 → RV2, mid → no change
                if (channels.size > 8 && channels[8] != 65535) {
                    val newId = when {
                        channels[8] < 1250 -> { roverExplicitlySelected = true; 1 }
                        channels[8] > 1750 -> { roverExplicitlySelected = true; 2 }
                        else               -> selectedRoverId
                    }
                    if (newId != selectedRoverId) {
                        selectedRoverId = newId
                        updateRcStrip(newId)
                        updateRecButtonState()
                    }
                }
                if (sysId == selectedRoverId) {
                    updateRcStrip(sysId)
                    if (isRecording) checkAuxChannelChanges(channels)
                }
                // Link indicators updated via onLinkStatus (SBUS_OK / RF_OK named floats)
            }
        },

        // Link mismatch: RC switch state ≠ slave reported state
        onLinkMismatch = { reason ->
            runOnUiThread { handleLinkMismatch(reason) }
        },

        // Rerouted path from navigator — draw as dashed orange/cyan overlay
        onReroutedPath = { sysId, path ->
            runOnUiThread {
                roverReroutedPaths[sysId] = path
                redrawReroutedPath(sysId)
            }
        },

        onGpsStatus = { sysId, fixType ->
            runOnUiThread {
                updateRtkLabel(sysId, fixType)
                handleRtkChange(sysId, fixType)
            }
        },

        onNavStatus = { sysId, status ->
            runOnUiThread {
                roverNavStatus[sysId] = status
                updateNavStatus(sysId, status)
            }
        },

        onLinkStatus = { sysId, type, ok ->
            runOnUiThread { updateLinkIndicator(sysId, type, ok) }
        },

        onMissionDownloaded = { sysId, waypoints ->
            runOnUiThread {
                if (waypoints.isNotEmpty()) {
                    roverMissions[sysId]        = waypoints.map { (lat, lon, _) -> LatLng(lat, lon) }
                    roverMissionBypass[sysId]   = waypoints.map { (_, _, bypass) -> bypass }
                    roverMissionVisible[sysId]  = true
                    // Preserve current WP index if rover is mid-mission
                    if (roverNextWaypointIndex[sysId] == null)
                        roverNextWaypointIndex[sysId] = 0
                } else {
                    roverMissions.remove(sysId)
                    roverMissionBypass.remove(sysId)
                    roverMissionVisible.remove(sysId)
                }
                redrawRoverMissions()
            }
        },

        onParamValue = { _, name, value, _, _ ->
            runOnUiThread { updateParamField(name, value) }
        },

        onHaccStatus = { sysId, haccMm ->
            runOnUiThread { handleHaccChange(sysId, haccMm) }
        },
        onReroutePending = { sysId, pending ->
            runOnUiThread {
                if (pending && sysId == selectedRoverId) showRerouteConfirmation(sysId)
            }
        },
    )

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        touchOverlay          = findViewById(R.id.touchOverlay)
        btnModeMenu           = findViewById(R.id.btnModeMenu)
        dotRv1Hb              = findViewById(R.id.dotRv1Hb)
        dotRv2Hb              = findViewById(R.id.dotRv2Hb)
        dotRv1Sbus            = findViewById(R.id.dotRv1Sbus)
        dotRv2Rf              = findViewById(R.id.dotRv2Rf)
        txtRv1Bat             = findViewById(R.id.txtRv1Bat)
        txtRv1Temp            = findViewById(R.id.txtRv1Temp)
        txtRv1Tank            = findViewById(R.id.txtRv1Tank)
        txtRv1Rtk             = findViewById(R.id.txtRv1Rtk)
        txtRv1Status          = findViewById(R.id.txtRv1Status)
        txtRv1Wp              = findViewById(R.id.txtRv1Wp)
        txtRv2Bat             = findViewById(R.id.txtRv2Bat)
        txtRv2Temp            = findViewById(R.id.txtRv2Temp)
        txtRv2Tank            = findViewById(R.id.txtRv2Tank)
        txtRv2Rtk             = findViewById(R.id.txtRv2Rtk)
        txtRv2Status          = findViewById(R.id.txtRv2Status)
        txtRv2Wp              = findViewById(R.id.txtRv2Wp)
        txtRcChannels         = findViewById(R.id.txtRcChannels)
        btnPlannerMenu        = findViewById(R.id.btnPlannerMenu)
        btnRec                = findViewById(R.id.btnRec)
        btnLayers             = findViewById(R.id.btnLayers)
        btnCenter             = findViewById(R.id.btnCenter)
        btnEStop              = findViewById(R.id.btnEStop)
        btnStart              = findViewById(R.id.btnStart)
        btnStop               = findViewById(R.id.btnStop)

        loadStations()

        val mapFragment = supportFragmentManager
            .findFragmentById(R.id.map) as SupportMapFragment
        mapFragment.getMapAsync(this)

        setupListeners()
        setMode(AppMode.PLANNER)
        roverManager.startListening(this)
    }

    override fun onDestroy() {
        super.onDestroy()
        roverManager.stopListening()
    }

    override fun onMapReady(googleMap: GoogleMap) {
        map = googleMap
        setupMap()
    }

    // ─── RC channel strip ─────────────────────────────────────────────────────

    private fun updateRcStrip(sysId: Int) {
        val ch = roverPpmChannels[sysId]
        if (ch == null || ch.isEmpty()) {
            txtRcChannels.text = "RC: --"
            return
        }
        // Compact: "R1 | 1:1500 2:1500 3:2000 4:2000 5:1500 6:1500 7:1500 8:1500"
        val sb = StringBuilder("R$sysId |")
        ch.forEachIndexed { i, v ->
            sb.append(" ${i + 1}:${if (v == 65535) "---" else v.toString()}")
        }
        txtRcChannels.text = sb.toString()
    }

    // ─── Listeners ────────────────────────────────────────────────────────────

    private fun setupListeners() {
        btnModeMenu.setOnClickListener { showModeMenu(it) }

        // Planner menu (Upload Mission / Save / Load / Clear)
        btnPlannerMenu.setOnClickListener { showPlannerMenu(it) }

        // REC — toggles timed position recording with aux-channel servo capture
        btnRec.setOnClickListener {
            if (!hasGoodAccuracy(selectedRoverId)) {
                Toast.makeText(this, "GPS accuracy insufficient — cannot record", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            if (isRecording) stopRecording() else startRecording()
        }

        // Map Controls
        btnLayers.setOnClickListener { toggleMapLayer() }
        btnCenter.setOnClickListener {
            isFollowingRover = !isFollowingRover
            updateFollowButtonState()
            val target = roverPositions[selectedRoverId]
            if (isFollowingRover && target != null)
                map.animateCamera(CameraUpdateFactory.newLatLng(target))
        }

        // EMERGENCY STOP — disarm all rovers immediately (broadcast sysId=0)
        // Sent 3× at 100 ms intervals — one dropped UDP packet must never prevent E-STOP.
        btnEStop.setOnClickListener {
            roverManager.sendCriticalCommand(0, 400, 0f, 0f)
            Toast.makeText(this, "EMERGENCY STOP — ALL DISARMED", Toast.LENGTH_LONG).show()
        }

        // START — arm then set AUTO mode
        btnStart.setOnClickListener {
            if (!roverExplicitlySelected) {
                Toast.makeText(this, "Choose a rover (CH9 switch)", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            // Primary gate: rover telemetry must confirm mission is loaded.
            if (roverNavStatus[selectedRoverId].let { it == null || it == "NA" }) {
                Toast.makeText(this, "No mission loaded on rover", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            if (!hasGoodAccuracy(selectedRoverId)) {
                Toast.makeText(this, "GPS accuracy insufficient — cannot start autonomous", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            // Mid-mission resume (WP_ACT > 0): arm directly, no approach confirmation
            val wpReached = roverNextWaypointIndex[selectedRoverId] ?: -1
            if (wpReached > 0) {
                armAndStart()
            } else {
                showApproachPathConfirmation()
            }
        }

        // STOP/PAUSE — set manual mode
        btnStop.setOnClickListener {
            roverManager.sendCommand(selectedRoverId, 176, 1f, 0f)
            roverModes[selectedRoverId] = AppMode.MANUAL
            Toast.makeText(this, "Rover $selectedRoverId: MANUAL", Toast.LENGTH_SHORT).show()
        }

    }



    // ─── Recording ───────────────────────────────────────────────────────────

    private fun startRecording() {
        isRecording = true
        routePoints.clear()
        recordedMission.clear()
        waypointTimestamps.clear()
        nextWaypointIndex = 0
        lastRecordedPos = null
        // Snapshot current aux channel states and prepend as initial DO_SET_SERVO
        // items so the rover restores servo positions at mission start.
        val ch = roverPpmChannels[selectedRoverId]
        for (i in 0..3) {
            val pwm = if (ch != null && i + 4 < ch.size) ch[i + 4] else 1500
            lastAuxPwm[i] = pwm
            recordedMission.add(MissionAction.ServoCmd(servo = i + 5, pwm = pwm))
        }
        btnRec.iconTint = ColorStateList.valueOf(Color.parseColor("#FFD600"))
        btnRec.strokeWidth = (3 * resources.displayMetrics.density + 0.5f).toInt()
        btnRec.strokeColor = ColorStateList.valueOf(Color.WHITE)
        recordHandler.post(recordRunnable)
        Toast.makeText(this, "Recording…", Toast.LENGTH_SHORT).show()
    }

    private fun stopRecording() {
        isRecording = false
        recordHandler.removeCallbacks(recordRunnable)
        smoothRecordedSpeeds()
        btnRec.iconTint = ColorStateList.valueOf(Color.parseColor("#444444"))
        btnRec.strokeWidth = 0
        val wpCount  = recordedMission.filterIsInstance<MissionAction.Waypoint>().size
        val srvCount = recordedMission.filterIsInstance<MissionAction.ServoCmd>().size

        if (isCorridorMode && routePoints.size >= 2) {
            // Extract smoothed speeds from recordedMission (parallel to routePoints)
            val speeds = recordedMission
                .filterIsInstance<MissionAction.Waypoint>()
                .map { it.speed }
            val corridor = routePoints.mapIndexed { i, pt ->
                Pair(pt, speeds.getOrElse(i) { 0f })
            }
            corridorList.add(corridor)
            Toast.makeText(this,
                "Corridor ${corridorList.size}: $wpCount points. " +
                "REC next row, or UPLOAD (rover auto-splits turns).",
                Toast.LENGTH_LONG).show()
        } else {
            Toast.makeText(this,
                "Recorded $wpCount waypoints" + if (srvCount > 0) " + $srvCount servo cmds" else "",
                Toast.LENGTH_LONG).show()
        }
    }

    /**
     * Post-hoc speed smoothing: replaces per-tick instantaneous speed with a
     * sliding-window average over ±W waypoints.  This eliminates GPS-noise
     * spikes while preserving deliberate speed changes (acceleration/braking).
     *
     * First waypoint keeps speed=0 so the navigator uses its default max_speed.
     */
    private fun smoothRecordedSpeeds() {
        // Collect indices of Waypoint entries inside the interleaved recordedMission list.
        val wpIndices = mutableListOf<Int>()
        for (i in recordedMission.indices) {
            if (recordedMission[i] is MissionAction.Waypoint) wpIndices.add(i)
        }
        if (wpIndices.size < 2 || waypointTimestamps.size != wpIndices.size) return

        val W = 2  // half-window → 5-point average (±2 neighbours)
        for (k in 1 until wpIndices.size) {  // skip k=0 (first WP → speed 0)
            val lo = maxOf(0, k - W)
            val hi = minOf(wpIndices.lastIndex, k + W)

            var totalDist = 0.0
            for (j in lo until hi) {
                val wp1 = recordedMission[wpIndices[j]] as MissionAction.Waypoint
                val wp2 = recordedMission[wpIndices[j + 1]] as MissionAction.Waypoint
                totalDist += haversineMetres(wp1.lat, wp1.lon, wp2.lat, wp2.lon)
            }
            val totalTimeSec = (waypointTimestamps[hi] - waypointTimestamps[lo]) / 1000.0
            if (totalTimeSec < 0.01) continue

            val speed = (totalDist / totalTimeSec).toFloat().coerceIn(MIN_SPEED_MPS, MAX_SPEED_MPS)
            val orig = recordedMission[wpIndices[k]] as MissionAction.Waypoint
            recordedMission[wpIndices[k]] = orig.copy(speed = speed)
        }
    }

    private fun clearMission() {
        if (isRecording) stopRecording()
        // In AUTO mode the rover may be actively navigating — disarm it first
        // so it stops immediately before the empty mission arrives.
        if (currentMode == AppMode.AUTO) {
            roverManager.sendCriticalCommand(selectedRoverId, 400, 0f, 0f)
        }
        corridorList.clear()
        nextWaypointIndex = 0
        roverNextWaypointIndex[selectedRoverId] = 0
        routePoints.clear()
        recordedMission.clear()
        lastRecordedPos = null
        roverMissions.remove(selectedRoverId)
        roverMissionBypass.remove(selectedRoverId)
        roverMissionVisible.remove(selectedRoverId)
        // Clear rerouted path overlay
        roverReroutedPaths.remove(selectedRoverId)
        reroutedPathOverlays[selectedRoverId]?.forEach {
            when (it) { is Marker -> it.remove(); is Polyline -> it.remove() }
        }
        reroutedPathOverlays.remove(selectedRoverId)
        // Reset obstacle drawing state
        isDrawingObstacle = false
        currentObstacleDraft.clear()
        draftPolyline?.remove()
        draftPolyline = null
        obstaclePolygons.clear()
        obstacleOverlays.forEach { it.remove() }
        obstacleOverlays.clear()
        redrawMap()
        redrawRoverMissions()
        roverManager.uploadMission(selectedRoverId, emptyList())
    }

    /**
     * Called on every RC_CHANNELS update while recording is active.
     * channels[4..7] = PPM CH5–CH8 (aux outputs, logical/SBUS order, no inversion).
     * A change of >100 µs triggers a DO_SET_SERVO entry in the recorded mission.
     * Servo numbers 5–8 match the RP2040 PPM CH5–CH8 outputs.
     */
    private fun checkAuxChannelChanges(channels: IntArray) {
        for (i in 0..3) {
            val ppmIdx = i + 4
            if (ppmIdx >= channels.size) break
            val newPwm = channels[ppmIdx]
            if (Math.abs(newPwm - lastAuxPwm[i]) > 100) {
                lastAuxPwm[i] = newPwm
                recordedMission.add(MissionAction.ServoCmd(servo = i + 5, pwm = newPwm))
            }
        }
    }

    private fun sendSavedStations(sysId: Int) {
        val bp: LatLng? = batteryPoint
        if (bp != null) {
            roverManager.sendCommand(sysId, 50001, (bp.latitude * 1e5).toFloat(), (bp.longitude * 1e5).toFloat())
        }
        val wp: LatLng? = waterPoint
        if (wp != null) {
            roverManager.sendCommand(sysId, 50002, (wp.latitude * 1e5).toFloat(), (wp.longitude * 1e5).toFloat())
        }
    }

    // ─── Per-rover HUD helpers ───────────────────────────────────────────────

    private fun updateHbDot(sysId: Int, connected: Boolean) {
        val dot = if (sysId == 1) dotRv1Hb else dotRv2Hb
        dot.setTextColor(
            if (connected) Color.parseColor("#4CAF50") else Color.parseColor("#F44336"))
        if (sysId == selectedRoverId) updateRcStrip(sysId)
    }

    private fun updateRoverSensors(sysId: Int, bat: Float, temp: Float, tank: Float) {
        val batView  = if (sysId == 1) txtRv1Bat  else txtRv2Bat
        val tempView = if (sysId == 1) txtRv1Temp else txtRv2Temp
        val tankView = if (sysId == 1) txtRv1Tank else txtRv2Tank
        if (bat  != -1f) batView.text  = "BAT: ${bat.toInt()}%"
        if (temp != -1f) tempView.text = "TMP: %.1f°".format(temp)
        if (tank != -1f) tankView.text = "TNK: %.0f%%".format(tank)
    }

    private fun updateRtkLabel(sysId: Int, fixType: Int) {
        val (text, colorHex) = when (fixType) {
            6    -> "RTK FIX" to "#4CAF50"
            5    -> "RTK FLT" to "#FFC107"
            4    -> "DGPS"    to "#00BCD4"
            3    -> "3D FIX"  to "#FFFFFF"
            2    -> "2D FIX"  to "#FF9800"
            1    -> "NO FIX"  to "#F44336"
            else -> "NO GPS"  to "#888888"
        }
        val hacc = roverHaccMm[sysId] ?: -1f
        val haccStr = if (hacc >= 0f) "  ${hacc.toInt()}mm" else ""
        val view = if (sysId == 1) txtRv1Rtk else txtRv2Rtk
        view.text = text + haccStr
        view.setTextColor(Color.parseColor(colorHex))
    }

    /**
     * Called on every GPS_RAW_INT update.
     * fixType: 0=NO_GPS  1=NO_FIX  2=2D  3=3D  4=DGPS  5=RTK_FLOAT  6=RTK_FIXED
     *
     * Enforces RTK-required safety:
     *   • REC button: grayed out (alpha 0.4) when fix < 6; full opacity when fixed.
     *     Click handler will still show a Toast if pressed while grayed.
     *   • If recording is active and fix drops below 6: stop recording immediately.
     *   • If rover is armed (autonomous) and fix drops below 6: disarm + alert dialog.
     *
     * Acts only on the transition fixed→not-fixed (wasFixed && !isFixed) to avoid
     * firing the dialog on every GPS update.
     */
    /**
     * Returns true if the rover has acceptable GPS accuracy for recording/autonomous.
     * When hAcc data is available (>= 0), uses hAcc <= 200 mm as the criterion.
     * Falls back to fix type >= 5 (RTK_FLOAT or better) when hAcc is not yet received.
     */
    private fun hasGoodAccuracy(sysId: Int): Boolean {
        val hacc = roverHaccMm[sysId] ?: -1f
        return if (hacc >= 0f) hacc <= 200f else (roverGpsFix[sysId] ?: 0) >= 5
    }

    private fun handleRtkChange(sysId: Int, fixType: Int) {
        roverGpsFix[sysId] = fixType

        if (sysId == selectedRoverId) updateRecButtonState()

        // Fix-type disarm is the fallback when UBX hAcc data is not available.
        // If hAcc is being received, handleHaccChange() handles accuracy-based disarm.
        val haccActive = (roverHaccMm[sysId] ?: -1f) >= 0f
        if (!haccActive && fixType < 5) {
            if (isRecording && sysId == selectedRoverId) {
                stopRecording()
                Toast.makeText(this, "Recording stopped — RTK fix lost", Toast.LENGTH_LONG).show()
            }
            if (roverArmed[sysId] == true) {
                roverManager.sendCriticalCommand(sysId, 400, 0f, 0f)
                AlertDialog.Builder(this)
                    .setTitle("⚠ RTK Lost — Rover $sysId Stopped")
                    .setMessage("RTK fix lost while rover was active.\nRover $sysId has been disarmed.")
                    .setCancelable(false)
                    .setPositiveButton("OK", null)
                    .show()
            }
        }
    }

    /**
     * Called when hAcc (horizontal accuracy) is received from a rover.
     * Disarms and alerts when accuracy degrades beyond 200 mm.
     */
    private fun handleHaccChange(sysId: Int, haccMm: Float) {
        val wasGood = hasGoodAccuracy(sysId)
        roverHaccMm[sysId] = haccMm
        val isGood = hasGoodAccuracy(sysId)

        updateRtkLabel(sysId, roverGpsFix[sysId] ?: 0)
        if (sysId == selectedRoverId) updateRecButtonState()

        if (wasGood && !isGood) {
            if (isRecording && sysId == selectedRoverId) {
                stopRecording()
                Toast.makeText(this,
                    "Recording stopped — GPS accuracy degraded (${haccMm.toInt()} mm > 200 mm)",
                    Toast.LENGTH_LONG).show()
            }
            if (roverArmed[sysId] == true) {
                roverManager.sendCriticalCommand(sysId, 400, 0f, 0f)
                AlertDialog.Builder(this)
                    .setTitle("⚠ GPS Accuracy Degraded — Rover $sysId Stopped")
                    .setMessage("GPS accuracy ${haccMm.toInt()} mm exceeds 200 mm limit.\nRover $sysId has been disarmed.")
                    .setCancelable(false)
                    .setPositiveButton("OK", null)
                    .show()
            }
        }
    }

    /** Gray out the REC button when the selected rover has insufficient GPS accuracy. */
    private fun updateRecButtonState() {
        btnRec.alpha = if (hasGoodAccuracy(selectedRoverId)) 1.0f else 0.4f
    }

    private fun updateNavStatus(sysId: Int, status: String) {
        val (text, colorHex) = when (status) {
            "ARM" -> "ARM" to "#FF9800"
            "MSL" -> "MSL" to "#2196F3"
            else  -> "NA"  to "#888888"
        }
        val view = if (sysId == 1) txtRv1Status else txtRv2Status
        view.text = text
        view.setTextColor(Color.parseColor(colorHex))
    }

    private fun updateLinkIndicator(sysId: Int, type: String, ok: Boolean) {
        val color = if (ok) Color.parseColor("#4CAF50") else Color.parseColor("#F44336")
        when {
            sysId == 1 && type == "SBUS" -> dotRv1Sbus.setTextColor(color)
            sysId == 2 && type == "RF"   -> dotRv2Rf.setTextColor(color)
        }
    }

    // ─── Link mismatch warning ───────────────────────────────────────────────

    /**
     * Called when the RC switch state (SWA/SWB on master) disagrees with what
     * the slave rover is reporting in its HEARTBEAT for more than 2 seconds.
     *
     * Action taken:
     *  1. Broadcast DISARM to all rovers immediately (sysId=0)
     *  2. Show a blocking AlertDialog that the user must acknowledge
     *
     * The pilot must physically move the RC emergency switch (SWA) and verify
     * both rovers are stopped before dismissing the dialog.
     */
    private fun handleLinkMismatch(reason: String) {
        // Immediately disarm all rovers (3× for UDP reliability)
        roverManager.sendCriticalCommand(0, 400, 0f, 0f)

        AlertDialog.Builder(this)
            .setTitle("⚠ RC / Slave Link Mismatch — All Stopped")
            .setMessage(
                "$reason\n\n" +
                "Both rovers have been disarmed.\n\n" +
                "Before restarting:\n" +
                "  • Check WiFi relay is active (master → slave)\n" +
                "  • Verify RC emergency switch (SWA) is in the SAFE position\n" +
                "  • Confirm slave rover telemetry is visible on the map\n"
            )
            .setCancelable(false)
            .setPositiveButton("Acknowledged") { _, _ -> }
            .show()
    }

    // ─── Multi-rover visuals ─────────────────────────────────────────────────

    private fun updateRover(sysId: Int, lat: Double, lon: Double, hdg: Float) {
        val pos = LatLng(lat, lon)
        if (lat == 0.0 && lon == 0.0) return

        roverPositions[sysId] = pos

        if (isFollowingRover && sysId == selectedRoverId)
            map.animateCamera(CameraUpdateFactory.newLatLng(pos))

        val isAuto     = roverModes[sysId] == AppMode.AUTO
        val isArmed    = roverArmed[sysId] ?: false
        val isSelected = sysId == selectedRoverId
        val iconKey    = Triple(isAuto, isArmed, isSelected)

        if (!roverMarkers.containsKey(sysId)) {
            val bmp = createRoverBitmap(sysId, isAuto, isArmed, isSelected)
            val marker = map.addMarker(
                MarkerOptions()
                    .position(pos)
                    .anchor(0.5f, 0.75f)   // anchor at back of arrow
                    .flat(true)
                    .zIndex(1f)            // mission overlays sit above at zIndex 2+
                    .icon(BitmapDescriptorFactory.fromBitmap(bmp))
            )
            if (marker != null) {
                roverMarkers[sysId]  = marker
                roverIconKeys[sysId] = iconKey
            }
        }

        roverMarkers[sysId]?.apply {
            position = pos
            rotation = hdg
            // Only rebuild the bitmap when state changes — prevents flicker on every GPS tick
            if (roverIconKeys[sysId] != iconKey) {
                roverIconKeys[sysId] = iconKey
                setIcon(BitmapDescriptorFactory.fromBitmap(
                    createRoverBitmap(sysId, isAuto, isArmed, isSelected)))
            }
        }
    }

    /**
     * Navigation-arrow rover icon (Google Maps style).
     *   Body colour  — Red = RV1 / Blue = RV2
     *   Status dot   — Green = disarmed manual / Orange = armed / Yellow = AUTO
     *   White outline when this rover is selected
     * Tip points UP (North) in the bitmap; Map.flat + rotation handles heading.
     * Anchored at (0.5, 0.75) — back of the chevron body.
     */
    private fun createRoverBitmap(sysId: Int, isAuto: Boolean,
                                  isArmed: Boolean, isSelected: Boolean): Bitmap {
        val dp = resources.displayMetrics.density
        val w  = (22 * dp + 0.5f).toInt()
        val h  = (30 * dp + 0.5f).toInt()
        val bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)

        val bodyColor = if (sysId == 1) Color.parseColor("#F44336")
                        else             Color.parseColor("#2196F3")
        val dotColor  = when {
            isAuto  -> Color.YELLOW
            isArmed -> Color.parseColor("#FF8C00")
            else    -> Color.parseColor("#4CAF50")
        }

        // Chevron arrow: tip at top, concave notch at base
        val path = Path().apply {
            moveTo(w / 2f,        0f           )   // tip — heading direction
            lineTo(w.toFloat(),   h * 0.85f    )   // bottom-right wing
            lineTo(w / 2f,        h * 0.55f    )   // inner notch
            lineTo(0f,            h * 0.85f    )   // bottom-left wing
            close()
        }
        canvas.drawPath(path, Paint(Paint.ANTI_ALIAS_FLAG).apply { color = bodyColor })

        // Thin drop-shadow for depth
        canvas.drawPath(path, Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color       = Color.argb(60, 0, 0, 0)
            style       = Paint.Style.STROKE
            strokeWidth = (1.5f * dp).coerceAtLeast(1.5f)
        })

        // White outline when selected
        if (isSelected) {
            canvas.drawPath(path, Paint(Paint.ANTI_ALIAS_FLAG).apply {
                color       = Color.WHITE
                style       = Paint.Style.STROKE
                strokeWidth = (2.5f * dp).coerceAtLeast(2f)
            })
        }

        // Status dot at the notch
        canvas.drawCircle(w / 2f, h * 0.7f, w * 0.13f,
            Paint(Paint.ANTI_ALIAS_FLAG).apply { color = dotColor })

        return bitmap
    }

    // ─── Map setup ────────────────────────────────────────────────────────────

    private fun setupMap() {
        map.mapType = GoogleMap.MAP_TYPE_SATELLITE
        // Default view: user's field location
        map.moveCamera(CameraUpdateFactory.newLatLngZoom(
            LatLng(20.727715034053315, -103.5667815504699), 18.0f))
        map.uiSettings.isZoomControlsEnabled = true
        map.setPadding(0, 0, 0, 200)

        map.setOnCameraMoveStartedListener { reason ->
            if (reason == GoogleMap.OnCameraMoveStartedListener.REASON_GESTURE
                && isFollowingRover) {
                isFollowingRover = false
                updateFollowButtonState()
                Toast.makeText(this, "Follow Mode: OFF", Toast.LENGTH_SHORT).show()
            }
        }

        // Obstacle drawing: tap map to add vertices when OBS mode is active
        map.setOnMapClickListener { latLng ->
            if (currentMode == AppMode.PLANNER && isDrawingObstacle) {
                currentObstacleDraft.add(latLng)
                redrawObstacleDraft()
            }
        }
    }

    // ─── Map drawing ─────────────────────────────────────────────────────────

    private fun redrawMap() {
        if (!::map.isInitialized) return
        routeOverlays.forEach {
            when (it) {
                is Marker   -> it.remove()
                is Polyline -> it.remove()
            }
        }
        routeOverlays.clear()

        waterPoint?.let {
            map.addMarker(MarkerOptions().position(it).title("Water")
                .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_AZURE)))
                ?.also { m -> routeOverlays.add(m) }
        }
        batteryPoint?.let {
            map.addMarker(MarkerOptions().position(it).title("Battery")
                .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN)))
                ?.also { m -> routeOverlays.add(m) }
        }

        if (routePoints.isNotEmpty()) {
            // Planner route — white (not yet uploaded to rover)
            routeOverlays.add(map.addPolyline(
                PolylineOptions().addAll(routePoints).width(8f).color(Color.WHITE).zIndex(2f)))
            routePoints.forEachIndexed { _, pt ->
                map.addMarker(MarkerOptions().position(pt).anchor(0.5f, 0.5f)
                    .icon(createDotBitmap(Color.WHITE, 7)).flat(true).zIndex(2f))
                    ?.also { routeOverlays.add(it) }
            }
        }
    }


    private fun redrawRoverMissions() {
        if (!::map.isInitialized) return
        roverMissionOverlays.values.forEach { overlays ->
            overlays.forEach {
                when (it) {
                    is Marker   -> it.remove()
                    is Polyline -> it.remove()
                }
            }
            overlays.clear()
        }
        for ((roverId, points) in roverMissions) {
            if (roverMissionVisible[roverId] == false || points.isEmpty()) continue
            val overlays     = roverMissionOverlays.getOrPut(roverId) { mutableListOf() }
            val bypass       = roverMissionBypass[roverId] ?: emptyList()
            val pendingColor = if (roverId == 1) Color.parseColor("#F44336")
                               else              Color.parseColor("#2196F3")
            val walkedColor  = if (roverId == 1) Color.parseColor("#FFCDD2")
                               else              Color.parseColor("#BBDEFB")
            val bypassColor  = if (roverId == 1) Color.parseColor("#FF9800")
                               else              Color.parseColor("#00BCD4")
            val walkedIdx    = roverNextWaypointIndex[roverId] ?: 0
            val dashPattern  = listOf(Dot(), Gap(10f))

            // Walked portion — light rover color
            if (walkedIdx > 0) {
                val walked = (0..min(walkedIdx, points.size - 1)).map { points[it] }
                overlays.add(map.addPolyline(
                    PolylineOptions().addAll(walked).width(8f)
                        .color(walkedColor).zIndex(2f)))
            }
            // Pending portion — split by bypass flag for distinct rendering
            if (walkedIdx < points.size) {
                val start = max(0, walkedIdx - 1)
                var segStart = start
                var segIsBypass = bypass.getOrElse(start) { false }
                for (i in (start + 1)..points.size) {
                    val curBypass = if (i < points.size) bypass.getOrElse(i) { false } else !segIsBypass
                    if (curBypass != segIsBypass || i == points.size) {
                        val segPts = (segStart..min(i, points.size - 1)).map { points[it] }
                        if (segPts.size >= 2) {
                            val opts = PolylineOptions().addAll(segPts).width(8f).zIndex(2f)
                            if (segIsBypass) opts.color(bypassColor).pattern(dashPattern)
                            else             opts.color(pendingColor)
                            overlays.add(map.addPolyline(opts))
                        }
                        segStart = min(i, points.size - 1)
                        segIsBypass = curBypass
                    }
                }
            }
            // Waypoint dots — bypass dots in bypass color
            points.forEachIndexed { i, pt ->
                val isBypass = bypass.getOrElse(i) { false }
                val dotColor = if (i < walkedIdx) walkedColor
                               else if (isBypass) bypassColor
                               else pendingColor
                map.addMarker(MarkerOptions().position(pt).anchor(0.5f, 0.5f)
                    .icon(createDotBitmap(dotColor, 7)).flat(true).zIndex(2f))
                    ?.also { overlays.add(it) }
            }
        }
    }

    private fun distanceMeters(a: LatLng, b: LatLng): Double {
        val R    = 6371000.0
        val lat1 = Math.toRadians(a.latitude)
        val lat2 = Math.toRadians(b.latitude)
        val dLat = Math.toRadians(b.latitude - a.latitude)
        val dLon = Math.toRadians(b.longitude - a.longitude)
        val h    = sin(dLat / 2).pow(2) + cos(lat1) * cos(lat2) * sin(dLon / 2).pow(2)
        return 2 * R * asin(sqrt(h))
    }

    private fun createDotBitmap(color: Int, sizeDp: Int): BitmapDescriptor {
        val px = (sizeDp * resources.displayMetrics.density + 0.5f).toInt()
        val bmp = Bitmap.createBitmap(px, px, Bitmap.Config.ARGB_8888)
        Canvas(bmp).drawCircle(px / 2f, px / 2f, px / 2f,
            Paint(Paint.ANTI_ALIAS_FLAG).apply { this.color = color })
        return BitmapDescriptorFactory.fromBitmap(bmp)
    }

    // ─── Rerouted path drawing ────────────────────────────────────────────────

    /**
     * Draw the rerouted path for [roverId] as a dashed overlay.
     * Original path: green (#4CAF50) dots. Bypass detour segments: rover color
     * (orange #FF9800 for RV1, cyan #00BCD4 for RV2).
     * Called whenever a new rerouted path arrives via TUNNEL.
     */
    private fun redrawReroutedPath(roverId: Int) {
        if (!::map.isInitialized) return

        // Clear previous overlays for this rover
        reroutedPathOverlays[roverId]?.forEach {
            when (it) {
                is Marker   -> it.remove()
                is Polyline -> it.remove()
            }
        }
        reroutedPathOverlays[roverId] = mutableListOf()

        val path = roverReroutedPaths[roverId] ?: return
        if (path.isEmpty()) return

        val overlays = reroutedPathOverlays[roverId]!!
        // Only draw bypass detour segments in rover color (orange RV1, cyan RV2).
        // Original (non-bypass) path segments are already shown by redrawRoverMissions().
        val roverColor = if (roverId == 1) Color.parseColor("#FF9800")
                         else              Color.parseColor("#00BCD4")
        val pattern = listOf<com.google.android.gms.maps.model.PatternItem>(
            com.google.android.gms.maps.model.Dot(),
            com.google.android.gms.maps.model.Gap(10f)
        )

        // Walk the path and emit polyline segments only for bypass (detour) portions
        var segStart = 0
        var segBypass = path[0].third
        for (i in 1..path.size) {
            val flip = i == path.size || path[i].third != segBypass
            if (flip) {
                if (segBypass) {
                    val pts = path.slice(segStart until i)
                        .map { LatLng(it.first, it.second) }
                    overlays.add(map.addPolyline(
                        PolylineOptions().addAll(pts).width(6f).color(roverColor)
                            .pattern(pattern).zIndex(2f)
                    ))
                }
                if (i < path.size) {
                    segStart  = i - 1   // overlap by 1 pt so segments connect
                    segBypass = path[i].third
                }
            }
        }
    }

    /** Redraw the in-progress obstacle draft as a closed polyline. */
    private fun redrawObstacleDraft() {
        if (!::map.isInitialized) return
        draftPolyline?.remove()
        if (currentObstacleDraft.size >= 2) {
            val pts = ArrayList(currentObstacleDraft).also { it.add(currentObstacleDraft[0]) }
            draftPolyline = map.addPolyline(
                PolylineOptions().addAll(pts).width(5f)
                    .color(Color.parseColor("#FF5722")))
        }
    }

    /** Redraw all finalised obstacle polygons as red semi-transparent Google Maps Polygons. */
    private fun redrawObstacleOverlays() {
        if (!::map.isInitialized) return
        obstacleOverlays.forEach { it.remove() }
        obstacleOverlays.clear()
        for (poly in obstaclePolygons) {
            if (poly.size >= 3) {
                obstacleOverlays.add(map.addPolygon(
                    PolygonOptions().addAll(poly)
                        .strokeColor(Color.parseColor("#F44336"))
                        .strokeWidth(4f)
                        .fillColor(Color.argb(64, 244, 67, 54))))
            }
        }
    }

    // ─── Helpers ──────────────────────────────────────────────────────────────

    private fun updateFollowButtonState() {
        btnCenter.backgroundTintList = ColorStateList.valueOf(
            if (isFollowingRover) Color.GREEN else Color.WHITE)
    }

    private fun toggleMapLayer() {
        if (!::map.isInitialized) return
        if (map.mapType == GoogleMap.MAP_TYPE_SATELLITE) {
            map.mapType = GoogleMap.MAP_TYPE_NORMAL
            Toast.makeText(this, "Standard Map", Toast.LENGTH_SHORT).show()
        } else {
            map.mapType = GoogleMap.MAP_TYPE_SATELLITE
            Toast.makeText(this, "Satellite", Toast.LENGTH_SHORT).show()
        }
    }

    private fun showModeMenu(view: View) {
        val popup = PopupMenu(this, view)
        popup.menu.add("Mission Planner")
        popup.menu.add("Auto Mode")
        popup.menu.add("Nav Params")
        popup.setOnMenuItemClickListener { item ->
            when (item.title) {
                "Mission Planner" -> setMode(AppMode.PLANNER)
                "Auto Mode"       -> setMode(AppMode.AUTO)
                "Nav Params"      -> showNavParamsDialog()
            }
            true
        }
        popup.show()
    }

    private fun showNavParamsDialog() {
        paramDialogRoverId = selectedRoverId
        paramFields.clear()

        val ctx   = this
        val dp4   = (4 * resources.displayMetrics.density + 0.5f).toInt()
        val dp48  = (48 * resources.displayMetrics.density + 0.5f).toInt()
        val layout = android.widget.LinearLayout(ctx).apply {
            orientation = android.widget.LinearLayout.VERTICAL
            setPadding(dp48, dp4 * 6, dp48, dp4 * 6)
        }

        for ((name, label, unit) in PARAM_DEFS) {
            val row = android.widget.LinearLayout(ctx).apply {
                orientation = android.widget.LinearLayout.HORIZONTAL
                setPadding(0, dp4 * 2, 0, dp4 * 2)
            }
            val tv = android.widget.TextView(ctx).apply {
                text = if (unit.isNotEmpty()) "$label ($unit)" else label
                layoutParams = android.widget.LinearLayout.LayoutParams(
                    0, android.widget.LinearLayout.LayoutParams.WRAP_CONTENT, 1.5f)
                textSize = 13f
            }
            val et = android.widget.EditText(ctx).apply {
                inputType = android.text.InputType.TYPE_CLASS_NUMBER or
                            android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL
                layoutParams = android.widget.LinearLayout.LayoutParams(
                    0, android.widget.LinearLayout.LayoutParams.WRAP_CONTENT, 1f)
                textSize = 13f
                hint = "?"
                setText("")
            }
            paramFields[name] = et
            row.addView(tv)
            row.addView(et)
            layout.addView(row)
        }

        val scroll = android.widget.ScrollView(ctx).also { it.addView(layout) }

        AlertDialog.Builder(ctx)
            .setTitle("Nav Params — Rover $paramDialogRoverId")
            .setView(scroll)
            .setPositiveButton("Apply") { _, _ -> applyNavParams() }
            .setNegativeButton("Cancel", null)
            .setNeutralButton("Refresh") { _, _ ->
                roverManager.requestParams(paramDialogRoverId)
            }
            .show()

        // Request current values immediately so fields are pre-populated
        roverManager.requestParams(paramDialogRoverId)
    }

    private fun updateParamField(name: String, value: Float) {
        val et = paramFields[name] ?: return
        val text = if (value == kotlin.math.floor(value.toDouble()).toFloat() && value < 10_000f)
            value.toInt().toString()
        else
            "%.3f".format(value)
        et.setText(text)
    }

    private fun applyNavParams() {
        var sent = 0
        for ((name, et) in paramFields) {
            val text  = et.text.toString().trim()
            if (text.isEmpty()) continue
            val value = text.toFloatOrNull() ?: continue
            roverManager.setParam(paramDialogRoverId, name, value)
            sent++
        }
        if (sent > 0)
            Toast.makeText(this, "$sent param(s) sent to Rover $paramDialogRoverId",
                Toast.LENGTH_SHORT).show()
    }

    private fun showPlannerMenu(view: View) {
        val popup = PopupMenu(this, view)
        popup.menu.add("Upload Mission")
        popup.menu.add("Add Wait Point")
        popup.menu.add(if (isCorridorMode) "Switch to Waypoint Mode" else "Switch to Corridor Mode")
        popup.menu.add("Save")
        popup.menu.add("Load")
        popup.menu.add("Clear")
        popup.menu.add("Set Battery Station")
        popup.menu.add("Set Water Station")
        popup.setOnMenuItemClickListener { item ->
            when (item.title) {
                "Upload Mission"             -> doUploadMission()
                "Add Wait Point"             -> addWaitPoint()
                "Switch to Corridor Mode"    -> {
                    isCorridorMode = true
                    corridorList.clear()
                    Toast.makeText(this, "Corridor mode: REC each row, then UPLOAD", Toast.LENGTH_LONG).show()
                }
                "Switch to Waypoint Mode"    -> {
                    isCorridorMode = false
                    corridorList.clear()
                    Toast.makeText(this, "Waypoint mode", Toast.LENGTH_SHORT).show()
                }
                "Save"                -> showSaveDialog()
                "Load"                -> showLoadDialog()
                "Clear"               -> clearMission()
                "Set Battery Station" -> showSetStationDialog("Battery")
                "Set Water Station"   -> showSetStationDialog("Water")
            }
            true
        }
        popup.show()
    }

    private fun showSetStationDialog(type: String) {
        val roverPos = roverPositions[selectedRoverId]
        val options  = if (roverPos != null)
            arrayOf("Use Rover $selectedRoverId position", "Enter coordinates")
        else
            arrayOf("Enter coordinates")
        AlertDialog.Builder(this)
            .setTitle("Set $type Station")
            .setItems(options) { _, which ->
                val realWhich = if (roverPos != null) which else which + 1
                when (realWhich) {
                    0 -> applyStation(type, roverPos!!)
                    1 -> showStationCoordInput(type)
                }
            }
            .show()
    }

    private fun showStationCoordInput(type: String) {
        val dp = resources.displayMetrics.density.toInt()
        val layout = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp * 24, dp * 8, dp * 24, 0)
        }
        val latInput = EditText(this).apply {
            hint = "Latitude  (e.g. 20.7277)"
            inputType = android.text.InputType.TYPE_CLASS_NUMBER or
                        android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL or
                        android.text.InputType.TYPE_NUMBER_FLAG_SIGNED
        }
        val lonInput = EditText(this).apply {
            hint = "Longitude  (e.g. -103.5668)"
            inputType = android.text.InputType.TYPE_CLASS_NUMBER or
                        android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL or
                        android.text.InputType.TYPE_NUMBER_FLAG_SIGNED
        }
        layout.addView(latInput)
        layout.addView(lonInput)
        AlertDialog.Builder(this)
            .setTitle("Set $type Station")
            .setView(layout)
            .setPositiveButton("Set") { _, _ ->
                val lat = latInput.text.toString().toDoubleOrNull()
                val lon = lonInput.text.toString().toDoubleOrNull()
                if (lat != null && lon != null) applyStation(type, LatLng(lat, lon))
                else Toast.makeText(this, "Invalid coordinates", Toast.LENGTH_SHORT).show()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun applyStation(type: String, pos: LatLng) {
        val cmdId = if (type == "Battery") 50001 else 50002
        if (type == "Battery") batteryPoint = pos else waterPoint = pos
        saveStations()
        redrawMap()
        // Send to both rovers — stations are field infrastructure, both need to know
        val p1 = (pos.latitude  * 1e5).toFloat()
        val p2 = (pos.longitude * 1e5).toFloat()
        roverManager.sendCommand(1, cmdId, p1, p2)
        roverManager.sendCommand(2, cmdId, p1, p2)
        Toast.makeText(this,
            "$type station set: ${"%.5f".format(pos.latitude)}, ${"%.5f".format(pos.longitude)}",
            Toast.LENGTH_SHORT).show()
    }

    private fun doUploadMission() {
        if (isRecording) stopRecording()
        if (!roverExplicitlySelected) {
            Toast.makeText(this, "Choose a rover (CH9 switch)", Toast.LENGTH_SHORT).show()
            return
        }
        // Warn if rover already has a mission loaded — confirm overwrite
        val navSt = roverNavStatus[selectedRoverId]
        if (navSt == "MSL" || navSt == "ARM") {
            AlertDialog.Builder(this)
                .setTitle("Mission Already Loaded")
                .setMessage("Rover $selectedRoverId has a mission. Replace with new one?")
                .setPositiveButton("Replace") { _, _ ->
                    // streamMission already sends DISARM×3 before uploading,
                    // which clears the rover. Just proceed with the upload.
                    roverNavStatus[selectedRoverId] = "NA"
                    doUploadMission()
                }
                .setNegativeButton("Cancel", null)
                .show()
            return
        }
        // --- Corridor upload path ---
        if (isCorridorMode) {
            // Include current recording if in progress
            if (isRecording) stopRecording()
            if (corridorList.isEmpty()) {
                Toast.makeText(this, "No corridors recorded. REC each row first.", Toast.LENGTH_SHORT).show()
                return
            }
            roverManager.uploadCorridorMission(selectedRoverId, corridorList, corridorWidth)
            Toast.makeText(this,
                "Uploading ${corridorList.size} corridor(s) to Rover $selectedRoverId",
                Toast.LENGTH_SHORT).show()
            // Show corridors on map
            val allPts = corridorList.flatten().map { it.first }
            roverMissions[selectedRoverId]          = allPts
            roverMissionBypass[selectedRoverId]     = List(allPts.size) { false }
            roverMissionVisible[selectedRoverId]    = true
            roverNextWaypointIndex[selectedRoverId] = 0
            redrawRoverMissions()
            return
        }

        // --- Legacy waypoint upload path ---
        if (routePoints.isEmpty()) {
            Toast.makeText(this, "No waypoints to upload", Toast.LENGTH_SHORT).show()
            return
        }
        roverMissions[selectedRoverId]          = ArrayList(routePoints)
        roverMissionBypass[selectedRoverId]     = List(routePoints.size) { false }
        roverMissionVisible[selectedRoverId]    = true
        roverNextWaypointIndex[selectedRoverId] = 0

        // Build upload payload BEFORE clearing planner state.
        // Always prefer recordedMission (has per-waypoint speed + holdSecs from REC)
        // over bare routePoints (lat/lon only — speed defaults to max_speed).
        val actions: List<MissionAction> = if (recordedMission.isNotEmpty()) ArrayList(recordedMission)
            else routePoints.map { MissionAction.Waypoint(it.latitude, it.longitude) }
        val wpCount = routePoints.size

        if (obstaclePolygons.isNotEmpty()) {
            val obsLatLon = obstaclePolygons.map { poly ->
                poly.map { Pair(it.latitude, it.longitude) }
            }
            roverManager.uploadMissionWithObstacles(selectedRoverId, actions, obsLatLon)
            Toast.makeText(this,
                "Uploading $wpCount WPs + ${obstaclePolygons.size} obstacle(s) to Rover $selectedRoverId…",
                Toast.LENGTH_SHORT).show()
        } else if (recordedMission.isNotEmpty()) {
            roverManager.uploadRecordedMission(selectedRoverId, ArrayList(recordedMission))
            Toast.makeText(this,
                "Uploading $wpCount WPs to Rover $selectedRoverId…",
                Toast.LENGTH_SHORT).show()
        } else {
            roverManager.uploadMission(selectedRoverId,
                routePoints.map { Pair(it.latitude, it.longitude) })
            Toast.makeText(this,
                "Uploading $wpCount WPs to Rover $selectedRoverId…",
                Toast.LENGTH_SHORT).show()
        }

        // Clear planner route AFTER upload is dispatched — white overlay disappears,
        // mission is now shown by redrawRoverMissions() in rover color.
        routePoints.clear()
        recordedMission.clear()
        redrawMap()
        redrawRoverMissions()
    }

    private fun showRerouteConfirmation(sysId: Int) {
        AlertDialog.Builder(this)
            .setTitle("Route Changed")
            .setMessage("Rover $sysId has recalculated its route around an obstacle.\nConfirm to proceed with new route.")
            .setPositiveButton("Confirm") { _, _ ->
                roverManager.sendCommand(sysId, 50003, 1f, 0f)
            }
            .setNegativeButton("Reject") { _, _ ->
                roverManager.sendCommand(sysId, 50003, 0f, 0f)
            }
            .setCancelable(false)
            .show()
    }

    private fun armAndStart() {
        roverManager.sendCommand(selectedRoverId, 400, 1f, 0f)  // ARM
        Handler(Looper.getMainLooper()).postDelayed({
            roverManager.sendCommand(selectedRoverId, 176, 1f, 3f)  // AUTO
            roverModes[selectedRoverId] = AppMode.AUTO
            Toast.makeText(this,
                "Rover $selectedRoverId: Armed & AUTO", Toast.LENGTH_SHORT).show()
        }, 500)
    }

    private fun showApproachPathConfirmation() {
        val roverPos = roverPositions[selectedRoverId]
        val missionStart = roverMissions[selectedRoverId]?.firstOrNull()
        if (roverPos == null || missionStart == null) {
            armAndStart()
            return
        }
        val dist = distanceMeters(roverPos, missionStart)
        // If very close already, skip the dialog
        if (dist < 1.0) {
            armAndStart()
            return
        }
        // Draw dashed approach line on map
        val approachLine = map.addPolyline(
            PolylineOptions().add(roverPos, missionStart).width(6f)
                .color(Color.parseColor("#4CAF50"))
                .pattern(listOf(Dot(), Gap(10f))).zIndex(3f))
        AlertDialog.Builder(this)
            .setTitle("Confirm Approach Route")
            .setMessage("Rover will travel ${"%.1f".format(dist)} m to the first waypoint.\nConfirm to arm and start.")
            .setPositiveButton("Confirm") { _, _ ->
                approachLine.remove()
                armAndStart()
            }
            .setNegativeButton("Cancel") { _, _ ->
                approachLine.remove()
            }
            .setCancelable(false)
            .show()
    }

    private fun addWaitPoint() {
        val pos = roverPositions[selectedRoverId]
        if (pos != null) {
            recordedMission.add(MissionAction.Waypoint(pos.latitude, pos.longitude, 0f, -1f))
            routePoints.add(pos)
            redrawMap()
            Toast.makeText(this, "Wait point added", Toast.LENGTH_SHORT).show()
        } else {
            Toast.makeText(this, "No rover position available", Toast.LENGTH_SHORT).show()
        }
    }

    private fun setMode(mode: AppMode) {
        // Discard any in-progress obstacle draft when leaving PLANNER mode
        if (mode != AppMode.PLANNER && isDrawingObstacle) {
            isDrawingObstacle = false
            currentObstacleDraft.clear()
            draftPolyline?.remove()
            draftPolyline = null
        }
        currentMode = mode
        touchOverlay.visibility   = View.GONE
        btnPlannerMenu.visibility = View.GONE
        btnRec.visibility         = View.GONE
        btnStart.visibility       = View.GONE
        btnStop.visibility        = View.GONE
        when (mode) {
            AppMode.MANUAL  -> { btnModeMenu.text = "MODE: MANUAL" }
            AppMode.PLANNER -> {
                btnModeMenu.text          = "MODE: PLANNER"
                btnPlannerMenu.visibility = View.VISIBLE
                btnRec.visibility         = View.VISIBLE
            }
            AppMode.AUTO    -> {
                btnModeMenu.text    = "MODE: AUTO"
                btnStart.visibility = View.VISIBLE
                btnStop.visibility  = View.VISIBLE
                redrawRoverMissions()
            }
        }
    }

    private fun saveStations() {
        val prefs = getPreferences(Context.MODE_PRIVATE).edit()
        waterPoint?.let   { prefs.putString("w_lat", it.latitude.toString()); prefs.putString("w_lon", it.longitude.toString()) }
        batteryPoint?.let { prefs.putString("b_lat", it.latitude.toString()); prefs.putString("b_lon", it.longitude.toString()) }
        prefs.apply()
    }

    private fun loadStations() {
        val prefs = getPreferences(Context.MODE_PRIVATE)
        val wLat  = prefs.getString("w_lat", null)
        val wLon  = prefs.getString("w_lon", null)
        if (wLat != null) waterPoint = LatLng(wLat.toDouble(), wLon!!.toDouble())
        val bLat  = prefs.getString("b_lat", null)
        val bLon  = prefs.getString("b_lon", null)
        if (bLat != null) batteryPoint = LatLng(bLat.toDouble(), bLon!!.toDouble())
    }

    private fun showSaveDialog() {
        val input = EditText(this).apply { hint = "Mission name" }
        AlertDialog.Builder(this).setTitle("Save Mission").setView(input)
            .setPositiveButton("Save") { _, _ -> saveMissionFile(input.text.toString()) }
            .show()
    }

    private fun saveMissionFile(name: String) {
        val sb = StringBuilder("Latitude,Longitude\n")
        routePoints.forEach { sb.append("${it.latitude},${it.longitude}\n") }
        try {
            val fname = if (name.endsWith(".csv")) name else "$name.csv"
            File(getExternalFilesDir(null), fname).writeText(sb.toString())
            Toast.makeText(this, "Saved $fname", Toast.LENGTH_SHORT).show()
        } catch (e: Exception) {
            Toast.makeText(this, "Save failed: ${e.message}", Toast.LENGTH_SHORT).show()
        }
    }

    private fun showLoadDialog() {
        val files = getExternalFilesDir(null)
            ?.listFiles { _, n -> n.endsWith(".csv") } ?: return
        if (files.isEmpty()) {
            Toast.makeText(this, "No saved missions", Toast.LENGTH_SHORT).show()
            return
        }
        AlertDialog.Builder(this).setTitle("Load Mission")
            .setItems(files.map { it.name }.toTypedArray()) { _, i ->
                parseCSV(files[i].readText())
            }.show()
    }

    private fun parseCSV(content: String) {
        nextWaypointIndex = 0
        routePoints.clear()
        content.split("\n").drop(1).forEach { line ->
            val p = line.split(",")
            if (p.size >= 2) {
                try { routePoints.add(LatLng(p[0].toDouble(), p[1].toDouble())) }
                catch (_: NumberFormatException) {}
            }
        }
        redrawMap()
        Toast.makeText(this, "Loaded ${routePoints.size} waypoints", Toast.LENGTH_SHORT).show()
    }
}
