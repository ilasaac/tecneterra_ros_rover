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
import com.google.android.material.floatingactionbutton.FloatingActionButton
import java.io.File
import kotlin.math.*

class MainActivity : AppCompatActivity(), OnMapReadyCallback {

    // --- ENUMS & STATE ---
    enum class AppMode { MANUAL, PLANNER, AUTO }

    private var currentMode = AppMode.MANUAL

    // --- MULTI-ROVER STATE ---
    private var selectedRoverId = 1
    private val roverMarkers    = HashMap<Int, Marker>()
    private val roverPositions  = HashMap<Int, LatLng>()
    private val roverModes      = HashMap<Int, AppMode>()
    private val roverArmed      = HashMap<Int, Boolean>()
    private val roverPpmChannels = HashMap<Int, IntArray>()   // latest RC_CHANNELS per rover

    // Per-rover uploaded missions
    private val roverMissions        = HashMap<Int, List<LatLng>>()
    private val roverMissionVisible  = HashMap<Int, Boolean>()
    private val roverMissionOverlays = HashMap<Int, MutableList<Any>>()

    // UI Elements
    private lateinit var map: GoogleMap
    private lateinit var touchOverlay: View
    private lateinit var btnModeMenu: Button
    private lateinit var plannerToolbar: LinearLayout
    private lateinit var autoToolbar: LinearLayout

    // Status & Buttons
    private lateinit var txtBattery:     TextView
    private lateinit var txtTank:        TextView
    private lateinit var txtTemp:        TextView
    private lateinit var txtConnection:  TextView   // ● connection dot
    private lateinit var txtRcChannels:  TextView   // RC PPM strip
    private lateinit var btnSave:        FloatingActionButton
    private lateinit var btnLoad:        FloatingActionButton
    private lateinit var btnLayers:      FloatingActionButton
    private lateinit var btnCenter:      FloatingActionButton
    private lateinit var btnEStop:       Button
    private lateinit var btnRec:         Button

    // Auto Buttons
    private lateinit var btnUpload:   Button
    private lateinit var btnStart:    Button
    private lateinit var btnStop:     Button
    private lateinit var btnClear:    Button
    private lateinit var btnToggleR1: Button
    private lateinit var btnToggleR2: Button

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

    private val recordHandler = Handler(Looper.getMainLooper())
    private val recordRunnable = object : Runnable {
        override fun run() {
            if (!isRecording) return
            roverPositions[selectedRoverId]?.let { pos ->
                recordedMission.add(MissionAction.Waypoint(pos.latitude, pos.longitude))
                routePoints.add(pos)
                redrawMap()
            }
            recordHandler.postDelayed(this, 500)
        }
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
            runOnUiThread {
                if (sysId == selectedRoverId) {
                    if (bat  != -1f) txtBattery.text = "B: ${bat.toInt()}%"
                    if (temp != -1f) txtTemp.text    = "C: %.1f°".format(temp)
                    if (tank != -1f) txtTank.text    = "T: %.0f%%".format(tank)
                }
            }
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
            runOnUiThread { updateConnectionDot(sysId, connected) }
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
                    }
                }
                if (sysId == selectedRoverId) {
                    updateRcStrip(sysId)
                    if (isRecording) checkAuxChannelChanges(channels)
                }
            }
        },

        // Link mismatch: RC switch state ≠ slave reported state
        onLinkMismatch = { reason ->
            runOnUiThread { handleLinkMismatch(reason) }
        },
    )

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        touchOverlay          = findViewById(R.id.touchOverlay)
        btnModeMenu           = findViewById(R.id.btnModeMenu)
        plannerToolbar        = findViewById(R.id.plannerToolbar)
        autoToolbar           = findViewById(R.id.autoToolbar)
        txtBattery            = findViewById(R.id.txtBattery)
        txtTank               = findViewById(R.id.txtTank)
        txtTemp               = findViewById(R.id.txtTemp)
        txtConnection         = findViewById(R.id.txtConnection)
        txtRcChannels         = findViewById(R.id.txtRcChannels)
        btnSave               = findViewById(R.id.btnSave)
        btnLoad               = findViewById(R.id.btnLoad)
        btnLayers             = findViewById(R.id.btnLayers)
        btnCenter             = findViewById(R.id.btnCenter)
        btnEStop              = findViewById(R.id.btnEStop)
        btnRec                = findViewById(R.id.btnRec)
        btnUpload             = findViewById(R.id.btnUpload)
        btnStart              = findViewById(R.id.btnStart)
        btnStop               = findViewById(R.id.btnStop)
        btnClear              = findViewById(R.id.btnClear)
        btnToggleR1           = findViewById(R.id.btnToggleR1)
        btnToggleR2           = findViewById(R.id.btnToggleR2)

        loadStations()

        // Hotspot mode: app auto-discovers rovers via broadcast — no target IP needed.
        // Just start listening; rovers will reply to us once they receive our heartbeat.
        txtConnection.text      = "● Waiting…"
        txtConnection.setTextColor(Color.parseColor("#FF9800"))

        val mapFragment = supportFragmentManager
            .findFragmentById(R.id.map) as SupportMapFragment
        mapFragment.getMapAsync(this)

        setupListeners()
        setMode(AppMode.MANUAL)
        roverManager.startListening()
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

        // Planner Actions
        btnSave.setOnClickListener      { showSaveDialog() }
        btnLoad.setOnClickListener      { showLoadDialog() }

        // REC — toggles timed position recording with aux-channel servo capture
        btnRec.setOnClickListener {
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

        // PLANNER: UPLOAD
        btnUpload.setOnClickListener {
            if (isRecording) stopRecording()
            if (routePoints.isEmpty()) {
                Toast.makeText(this, "No waypoints to upload", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            roverMissions[selectedRoverId]       = ArrayList(routePoints)
            roverMissionVisible[selectedRoverId] = true
            redrawMap()
            redrawRoverMissions()

            val servoCount = recordedMission.filterIsInstance<MissionAction.ServoCmd>().size
            if (servoCount > 0) {
                // Recorded mission: mixed waypoints + DO_SET_SERVO commands
                roverManager.uploadRecordedMission(selectedRoverId, recordedMission)
                Toast.makeText(this,
                    "Uploading ${routePoints.size} WPs + $servoCount servo cmds to Rover $selectedRoverId…",
                    Toast.LENGTH_SHORT).show()
            } else {
                // Waypoints-only path (tap-draw or ADD button, no servo changes)
                roverManager.uploadMission(selectedRoverId,
                    routePoints.map { Pair(it.latitude, it.longitude) })
                Toast.makeText(this,
                    "Uploading ${routePoints.size} WPs to Rover $selectedRoverId…",
                    Toast.LENGTH_SHORT).show()
            }
        }

        // START — arm then set AUTO mode
        btnStart.setOnClickListener {
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

        btnClear.setOnClickListener {
            nextWaypointIndex = 0; routePoints.clear(); recordedMission.clear(); redrawMap()
            roverManager.uploadMission(selectedRoverId, emptyList())
        }

        btnToggleR1.setOnClickListener { toggleRoverMission(1, btnToggleR1) }
        btnToggleR2.setOnClickListener { toggleRoverMission(2, btnToggleR2) }
    }



    // ─── Recording ───────────────────────────────────────────────────────────

    private fun startRecording() {
        isRecording = true
        routePoints.clear()
        recordedMission.clear()
        nextWaypointIndex = 0
        // Snapshot current aux channel states so only future *changes* are recorded
        roverPpmChannels[selectedRoverId]?.let { ch ->
            for (i in 0..3) lastAuxPwm[i] = if (i + 4 < ch.size) ch[i + 4] else 1500
        }
        btnRec.text = "⏹ STOP"
        btnRec.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#607D8B"))
        recordHandler.post(recordRunnable)
        Toast.makeText(this, "Recording…", Toast.LENGTH_SHORT).show()
    }

    private fun stopRecording() {
        isRecording = false
        recordHandler.removeCallbacks(recordRunnable)
        btnRec.text = "⏺ REC"
        btnRec.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#F44336"))
        val wpCount  = recordedMission.filterIsInstance<MissionAction.Waypoint>().size
        val srvCount = recordedMission.filterIsInstance<MissionAction.ServoCmd>().size
        Toast.makeText(this,
            "Recorded $wpCount waypoints" + if (srvCount > 0) " + $srvCount servo cmds" else "",
            Toast.LENGTH_LONG).show()
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

    // ─── Connection indicator ────────────────────────────────────────────────

    private fun updateConnectionDot(sysId: Int, connected: Boolean) {
        if (connected) {
            txtConnection.setTextColor(Color.GREEN)
            txtConnection.text = "● R$sysId"
        } else {
            txtConnection.setTextColor(Color.RED)
            txtConnection.text = "● Lost R$sysId"
        }
        // Refresh RC strip when selected rover connection changes
        if (sysId == selectedRoverId) updateRcStrip(sysId)
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

        if (!roverMarkers.containsKey(sysId)) {
            val marker = map.addMarker(
                MarkerOptions().position(pos).anchor(0.5f, 0.5f).flat(true)
            )
            if (marker != null) roverMarkers[sysId] = marker
        }

        val isAuto     = roverModes[sysId] == AppMode.AUTO
        val isArmed    = roverArmed[sysId] ?: false
        val isSelected = sysId == selectedRoverId
        roverMarkers[sysId]?.apply {
            position  = pos
            rotation  = hdg
            setIcon(BitmapDescriptorFactory.fromBitmap(
                createRoverBitmap(sysId, isAuto, isArmed, isSelected)))
        }
    }

    /**
     * Rover icon:
     *   Body colour  — Red = Rover 1 / Blue = Rover 2 (white ring = selected)
     *   Centre dot   — Green = MANUAL & disarmed / Orange = armed manual / Yellow = AUTO
     */
    private fun createRoverBitmap(sysId: Int, isAuto: Boolean,
                                  isArmed: Boolean, isSelected: Boolean): Bitmap {
        val drawable = ContextCompat.getDrawable(this, R.drawable.ic_rover)!!
        drawable.setTint(if (sysId == 1) Color.RED else Color.BLUE)

        val w = (drawable.intrinsicWidth  * 2.5).toInt()
        val h = (drawable.intrinsicHeight * 2.5).toInt()
        val bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        val canvas = Canvas(bitmap)
        drawable.setBounds(0, 0, w, h)
        drawable.draw(canvas)

        val dotColor = when {
            isAuto  -> Color.YELLOW
            isArmed -> Color.parseColor("#FF8C00")
            else    -> Color.GREEN
        }
        canvas.drawCircle(w / 2f, h / 2f, w / 6f,
            Paint().apply { color = dotColor; isAntiAlias = true })

        if (isSelected) {
            canvas.drawCircle(w / 2f, h / 2f, w / 2.2f, Paint().apply {
                color       = Color.WHITE
                style       = Paint.Style.STROKE
                strokeWidth = 5f
                isAntiAlias = true
            })
        }
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
                    PolylineOptions().addAll(passed).width(8f).color(Color.GREEN)))
            }
            if (nextWaypointIndex < routePoints.size) {
                val pending = (max(0, nextWaypointIndex - 1) until routePoints.size)
                    .map { routePoints[it] }
                routeOverlays.add(map.addPolyline(
                    PolylineOptions().addAll(pending).width(5f).color(Color.RED)))
            }
            routePoints.forEachIndexed { index, pt ->
                val hue = if (index == nextWaypointIndex)
                    BitmapDescriptorFactory.HUE_CYAN
                else
                    BitmapDescriptorFactory.HUE_RED
                map.addMarker(MarkerOptions().position(pt)
                    .title("WP ${index + 1}")
                    .icon(BitmapDescriptorFactory.defaultMarker(hue))
                    .anchor(0.5f, 0.5f))
                    ?.also { m -> routeOverlays.add(m) }
            }
        }
    }

    private fun toggleRoverMission(roverId: Int, button: Button) {
        // Select rover for ARM / SET_MODE / UPLOAD commands
        selectedRoverId = roverId
        updateRcStrip(roverId)
        // Visual: selected button bright, other dimmed
        btnToggleR1.alpha = if (roverId == 1) 1.0f else 0.4f
        btnToggleR2.alpha = if (roverId == 2) 1.0f else 0.4f
        roverMissionVisible[roverId] = true
        redrawRoverMissions()
        Toast.makeText(this, "Rover $roverId selected", Toast.LENGTH_SHORT).show()
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
            val overlays = roverMissionOverlays.getOrPut(roverId) { mutableListOf() }
            val color    = if (roverId == 1) Color.RED else Color.BLUE
            val hue      = if (roverId == 1) BitmapDescriptorFactory.HUE_RED
                           else              BitmapDescriptorFactory.HUE_BLUE
            overlays.add(map.addPolyline(PolylineOptions().addAll(points).width(5f).color(color)))
            points.forEachIndexed { index, pt ->
                map.addMarker(MarkerOptions().position(pt)
                    .title("R$roverId WP ${index + 1}")
                    .icon(BitmapDescriptorFactory.defaultMarker(hue))
                    .anchor(0.5f, 0.5f))
                    ?.also { overlays.add(it) }
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
        popup.menu.add("Manual Mode")
        popup.menu.add("Mission Planner")
        popup.menu.add("Auto Mode")
        popup.setOnMenuItemClickListener { item ->
            when (item.title) {
                "Manual Mode"     -> setMode(AppMode.MANUAL)
                "Mission Planner" -> setMode(AppMode.PLANNER)
                "Auto Mode"       -> setMode(AppMode.AUTO)
            }
            true
        }
        popup.show()
    }

    private fun setMode(mode: AppMode) {
        currentMode = mode
        touchOverlay.visibility           = View.GONE
        plannerToolbar.visibility         = View.GONE
        autoToolbar.visibility            = View.GONE
        when (mode) {
            AppMode.MANUAL  -> { btnModeMenu.text = "MODE: MANUAL" }
            AppMode.PLANNER -> { btnModeMenu.text = "MODE: PLANNER"; plannerToolbar.visibility = View.VISIBLE }
            AppMode.AUTO    -> { btnModeMenu.text = "MODE: AUTO";    autoToolbar.visibility = View.VISIBLE; redrawRoverMissions() }
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
