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
    private val roverMarkers    = HashMap<Int, Marker>()
    private val roverPositions  = HashMap<Int, LatLng>()
    private val roverModes      = HashMap<Int, AppMode>()
    private val roverArmed      = HashMap<Int, Boolean>()
    // Cached icon state — only rebuild bitmap when armed/auto/selected actually changes
    private val roverIconKeys   = HashMap<Int, Triple<Boolean, Boolean, Boolean>>()
    private val roverNavStatus  = HashMap<Int, String>()   // "NA" | "MSL" | "ARM"
    private val roverGpsFix    = HashMap<Int, Int>()       // GPS_RAW_INT fixType (0–6); 6=RTK_FIX
    private val roverPpmChannels = HashMap<Int, IntArray>()   // latest RC_CHANNELS per rover

    // Per-rover uploaded missions
    private val roverMissions             = HashMap<Int, List<LatLng>>()
    private val roverMissionVisible       = HashMap<Int, Boolean>()
    private val roverMissionOverlays      = HashMap<Int, MutableList<Any>>()
    private val roverNextWaypointIndex    = HashMap<Int, Int>()   // last MISSION_ITEM_REACHED seq per rover

    // Per-rover rerouted path received from navigator via TUNNEL (after obstacle avoidance)
    // Triple: (lat, lon, isBypass)
    private val roverReroutedPaths   = HashMap<Int, List<Triple<Double, Double, Boolean>>>()
    private val reroutedPathOverlays = HashMap<Int, MutableList<Any>>()

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
    // Last PPM µs values for aux channels (PPM CH5–CH8, logical indices 4–7).
    // Initialised to 1500 (neutral) and updated when a >100 µs change is detected.
    private val lastAuxPwm = IntArray(4) { 1500 }

    // --- OBSTACLE DRAWING ---
    private var isDrawingObstacle = false
    private val currentObstacleDraft = mutableListOf<LatLng>()
    private val obstaclePolygons     = mutableListOf<MutableList<LatLng>>()
    private val obstacleOverlays     = mutableListOf<Polygon>()
    private var draftPolyline: Polyline? = null
    // Stationary hold tracking: last position where a waypoint was saved, and
    // accumulated stationary time. When rover moves again, the hold time is
    // written into the last recorded waypoint so the navigator waits there.
    private var lastRecordedPos: LatLng? = null
    private var pendingHoldSecs = 0f
    private val MIN_RECORD_DIST = 0.3   // metres — below this = considered stationary

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
                    // Rover moved — flush accumulated hold into the last waypoint
                    if (last != null && pendingHoldSecs > 0f) {
                        val idx = recordedMission.indexOfLast { it is MissionAction.Waypoint }
                        if (idx >= 0) {
                            val old = recordedMission[idx] as MissionAction.Waypoint
                            recordedMission[idx] = old.copy(holdSecs = pendingHoldSecs)
                        }
                        pendingHoldSecs = 0f
                    }
                    lastRecordedPos = pos
                    // Speed = avg distance covered in this 500 ms tick, clamped to [min, max].
                    // Use 0f for the first waypoint (no prior position) → navigator default.
                    val speed = if (last != null)
                        (dist.toFloat() / 0.5f).coerceIn(MIN_SPEED_MPS, MAX_SPEED_MPS)
                    else 0f
                    recordedMission.add(MissionAction.Waypoint(pos.latitude, pos.longitude, speed))
                    routePoints.add(pos)
                    redrawMap()
                } else {
                    // Rover stationary — accumulate hold time (500 ms per tick)
                    pendingHoldSecs += 0.5f
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

        onConnectionChange = { sysId, connected ->
            runOnUiThread { updateHbDot(sysId, connected) }
        },

        // RC_CHANNELS (#65) — PPM µs values from RP2040 (physical RC sticks via HM30/SBUS)
        onRcChannels = { sysId, channels ->
            runOnUiThread {
                roverPpmChannels[sysId] = channels
                // CH9 (index 8) selects active rover: <1250 → RV1, >1750 → RV2, mid → no change
                if (channels.size > 8 && channels[8] != 65535) {
                    val newId = when {
                        channels[8] < 1250 -> 1
                        channels[8] > 1750 -> 2
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
            if (roverGpsFix[selectedRoverId] != 6) {
                Toast.makeText(this, "RTK not fixed — cannot record", Toast.LENGTH_SHORT).show()
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
            // Primary gate: rover telemetry must confirm mission is loaded.
            // This allows missions uploaded from mission_planner.py (not just GQC).
            if (roverNavStatus[selectedRoverId].let { it == null || it == "NA" }) {
                Toast.makeText(this, "No mission loaded on rover", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            // Distance check only when we have the start position from a local upload.
            val missionStart = roverMissions[selectedRoverId]?.firstOrNull()
            val roverPos     = roverPositions[selectedRoverId]
            if (missionStart != null && (roverPos == null || distanceMeters(roverPos, missionStart) > 0.5)) {
                AlertDialog.Builder(this)
                    .setTitle("Too far from start")
                    .setMessage("Move the rover manually to the starting position")
                    .setPositiveButton("OK", null)
                    .show()
                return@setOnClickListener
            }
            if (roverGpsFix[selectedRoverId] != 6) {
                Toast.makeText(this, "RTK not fixed — cannot start autonomous", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            roverManager.sendCommand(selectedRoverId, 400, 1f, 0f)  // ARM
            Handler(Looper.getMainLooper()).postDelayed({
                roverManager.sendCommand(selectedRoverId, 176, 1f, 3f)  // AUTO
                roverModes[selectedRoverId] = AppMode.AUTO
                Toast.makeText(this,
                    "Rover $selectedRoverId: Armed & AUTO", Toast.LENGTH_SHORT).show()
            }, 500)
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
        nextWaypointIndex = 0
        lastRecordedPos = null
        pendingHoldSecs = 0f
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
        btnRec.iconTint = ColorStateList.valueOf(Color.parseColor("#444444"))
        btnRec.strokeWidth = 0
        val wpCount  = recordedMission.filterIsInstance<MissionAction.Waypoint>().size
        val srvCount = recordedMission.filterIsInstance<MissionAction.ServoCmd>().size
        Toast.makeText(this,
            "Recorded $wpCount waypoints" + if (srvCount > 0) " + $srvCount servo cmds" else "",
            Toast.LENGTH_LONG).show()
    }

    private fun clearMission() {
        if (isRecording) stopRecording()
        // In AUTO mode the rover may be actively navigating — disarm it first
        // so it stops immediately before the empty mission arrives.
        if (currentMode == AppMode.AUTO) {
            roverManager.sendCriticalCommand(selectedRoverId, 400, 0f, 0f)
        }
        nextWaypointIndex = 0
        roverNextWaypointIndex[selectedRoverId] = 0
        routePoints.clear()
        recordedMission.clear()
        lastRecordedPos = null
        pendingHoldSecs = 0f
        roverMissions.remove(selectedRoverId)
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
        val view = if (sysId == 1) txtRv1Rtk else txtRv2Rtk
        view.text = text
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
    private fun handleRtkChange(sysId: Int, fixType: Int) {
        val wasFixed = roverGpsFix[sysId] == 6
        roverGpsFix[sysId] = fixType
        val isFixed = fixType == 6

        if (sysId == selectedRoverId) updateRecButtonState()

        if (wasFixed && !isFixed) {
            // Recording: stop immediately when RTK is lost
            if (isRecording && sysId == selectedRoverId) {
                stopRecording()
                Toast.makeText(this, "Recording stopped — RTK fix lost", Toast.LENGTH_LONG).show()
            }
            // Autonomous: disarm if rover was armed
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

    /** Gray out the REC button when the selected rover has no RTK fix. */
    private fun updateRecButtonState() {
        btnRec.alpha = if (roverGpsFix[selectedRoverId] == 6) 1.0f else 0.4f
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
            if (nextWaypointIndex > 0) {
                val passed = (0..min(nextWaypointIndex, routePoints.size - 1))
                    .map { routePoints[it] }
                routeOverlays.add(map.addPolyline(
                    PolylineOptions().addAll(passed).width(8f).color(Color.parseColor("#4CAF50")).zIndex(2f)))
            }
            if (nextWaypointIndex < routePoints.size) {
                val pending = (max(0, nextWaypointIndex - 1) until routePoints.size)
                    .map { routePoints[it] }
                routeOverlays.add(map.addPolyline(
                    PolylineOptions().addAll(pending).width(8f).color(Color.parseColor("#F44336")).zIndex(2f)))
            }
            routePoints.forEachIndexed { i, pt ->
                val dotColor = when {
                    i == 0              -> Color.parseColor("#FFD600")
                    i < nextWaypointIndex -> Color.parseColor("#4CAF50")
                    else                -> Color.parseColor("#F44336")
                }
                val sizeDp   = if (i == 0) 12 else 7
                map.addMarker(MarkerOptions().position(pt).anchor(0.5f, 0.5f)
                    .icon(createDotBitmap(dotColor, sizeDp)).flat(true).zIndex(2f))
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
            val pendingColor = if (roverId == 1) Color.parseColor("#F44336")
                               else              Color.parseColor("#2196F3")
            val walkedIdx    = roverNextWaypointIndex[roverId] ?: 0

            // Walked portion — gray
            if (walkedIdx > 0) {
                val walked = (0..min(walkedIdx, points.size - 1)).map { points[it] }
                overlays.add(map.addPolyline(
                    PolylineOptions().addAll(walked).width(8f)
                        .color(Color.argb(200, 160, 160, 160)).zIndex(2f)))
            }
            // Pending portion — rover color; starts one point before walkedIdx to avoid a gap
            if (walkedIdx < points.size) {
                val pending = (max(0, walkedIdx - 1) until points.size).map { points[it] }
                overlays.add(map.addPolyline(
                    PolylineOptions().addAll(pending).width(8f).color(pendingColor).zIndex(2f)))
            }
            // Waypoint dots — skip already-passed waypoints (rover decides via WP_ACT)
            points.forEachIndexed { i, pt ->
                if (i > 0 && i < walkedIdx) return@forEachIndexed  // erased by rover
                val dotColor = if (i == 0) Color.parseColor("#FFD600") else pendingColor
                val sizeDp   = if (i == 0) 12 else 7
                map.addMarker(MarkerOptions().position(pt).anchor(0.5f, 0.5f)
                    .icon(createDotBitmap(dotColor, sizeDp)).flat(true).zIndex(2f))
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
        // Bypass detour segments shown in rover color; original path in green
        val roverColor = if (roverId == 1) Color.parseColor("#FF9800")
                         else              Color.parseColor("#00BCD4")
        val plannedColor = Color.parseColor("#4CAF50")
        val pattern = listOf<com.google.android.gms.maps.model.PatternItem>(
            com.google.android.gms.maps.model.Dot(),
            com.google.android.gms.maps.model.Gap(10f)
        )

        // Walk the path and emit a new polyline segment each time isBypass toggles
        var segStart = 0
        var segBypass = path[0].third
        for (i in 1..path.size) {
            val flip = i == path.size || path[i].third != segBypass
            if (flip) {
                val pts = path.slice(segStart until i)
                    .map { LatLng(it.first, it.second) }
                val color = if (segBypass) roverColor else plannedColor
                overlays.add(map.addPolyline(
                    PolylineOptions().addAll(pts).width(6f).color(color).pattern(pattern).zIndex(2f)
                ))
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
        popup.setOnMenuItemClickListener { item ->
            when (item.title) {
                "Mission Planner" -> setMode(AppMode.PLANNER)
                "Auto Mode"       -> setMode(AppMode.AUTO)
            }
            true
        }
        popup.show()
    }

    private fun showPlannerMenu(view: View) {
        val popup = PopupMenu(this, view)
        popup.menu.add("Upload Mission")
        popup.menu.add("Save")
        popup.menu.add("Load")
        popup.menu.add("Clear")
        popup.setOnMenuItemClickListener { item ->
            when (item.title) {
                "Upload Mission" -> doUploadMission()
                "Save"           -> showSaveDialog()
                "Load"           -> showLoadDialog()
                "Clear"          -> clearMission()
            }
            true
        }
        popup.show()
    }

    private fun doUploadMission() {
        if (isRecording) stopRecording()
        if (routePoints.isEmpty()) {
            Toast.makeText(this, "No waypoints to upload", Toast.LENGTH_SHORT).show()
            return
        }
        roverMissions[selectedRoverId]          = ArrayList(routePoints)
        roverMissionVisible[selectedRoverId]    = true
        roverNextWaypointIndex[selectedRoverId] = 0
        redrawMap()
        redrawRoverMissions()

        val servoCount = recordedMission.filterIsInstance<MissionAction.ServoCmd>().size
        val actions: List<MissionAction> = if (servoCount > 0) recordedMission
            else routePoints.map { MissionAction.Waypoint(it.latitude, it.longitude) }

        if (obstaclePolygons.isNotEmpty()) {
            val obsLatLon = obstaclePolygons.map { poly ->
                poly.map { Pair(it.latitude, it.longitude) }
            }
            roverManager.uploadMissionWithObstacles(selectedRoverId, actions, obsLatLon)
            Toast.makeText(this,
                "Uploading ${routePoints.size} WPs + ${obstaclePolygons.size} obstacle(s) to Rover $selectedRoverId…",
                Toast.LENGTH_SHORT).show()
        } else if (servoCount > 0) {
            roverManager.uploadRecordedMission(selectedRoverId, recordedMission)
            Toast.makeText(this,
                "Uploading ${routePoints.size} WPs + $servoCount servo cmds to Rover $selectedRoverId…",
                Toast.LENGTH_SHORT).show()
        } else {
            roverManager.uploadMission(selectedRoverId,
                routePoints.map { Pair(it.latitude, it.longitude) })
            Toast.makeText(this,
                "Uploading ${routePoints.size} WPs to Rover $selectedRoverId…",
                Toast.LENGTH_SHORT).show()
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
