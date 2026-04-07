# Rosetta CAN Protocol Guide - TTR Agro Rover

Reverse-engineered from OEM VCU source code + live bus captures (2026-04-06).
Verified against `data_vcu_can.h`, `Vcu_CanProtocol.cpp`, `MedicineCtrl.cpp`, `Robot.cpp`.

---

## Bus Architecture

| Port | Bitrate | Frame Type | Connects To |
|------|---------|------------|-------------|
| **CAN A** | 500 kbps | Standard 11-bit | Motor controllers, BMS, PDU |
| **CAN B** | 500 kbps | Standard 11-bit | Mirror of CAN A (TBC) |
| **CAN C** | 500 kbps | Extended 29-bit | NAV computer (IPC/RPi) <-> VCU |
| **CAN T** | Unknown | Unknown | Unknown |

---

# CAN A - Motor Bus (Standard 11-bit IDs)

## Motor Controller Status (100 Hz)

| CAN ID | Controller | Physical Motor |
|--------|-----------|----------------|
| 0x0119 | #1 | Sprayer rotation (dual motor) |
| 0x0120 | #2 | Drive Left |
| 0x0121 | #3 | Drive Right |
| 0x0122 | #4 | Medicine Pump |

**Frame (8 bytes):**

| Byte | Field | Notes |
|------|-------|-------|
| 0 | **State** | See table below |
| 1 | Reserved | 0x00 |
| 2-3 | Hardware ID | 0x7530 constant |
| 4-5 | Hardware ID | 0x4E20 constant |
| 6-7 | Reserved | 0x0000 |

**State byte (byte 0):**

| Value | Binary | Meaning |
|-------|--------|---------|
| 0x01 | 0000 0001 | Standby (present, no HV) |
| 0x09 | 0000 1001 | Active (present + HV ready) |
| 0x21 | 0010 0001 | Fault (present + fault) |

Bit decode: bit0=present, bit3=HV ready, bit5=fault.

## Motor Electrical Feedback (100 Hz)

3 of 4 controllers send this. 4th (pump) only sends when HV is applied.

| CAN ID | Controller |
|--------|-----------|
| 0x0130 | #1 Sprayer rotation |
| 0x0131 | #2 Drive Left |
| 0x0132 | #3 Drive Right |
| 0x0133 | #4 Pump (**absent when pump has no HV**) |

| Byte | Field | Notes |
|------|-------|-------|
| 0-1 | Current (raw) | 0x7D00 = zero center |
| 2-3 | Current (raw) | 0x7D00 = zero center |
| 4-5 | Status/mode | 0x0080=init, 0x00B0=precharge, 0x70C0=running |
| 6-7 | Checksum/counter | |

## Motor Temperature (100 Hz)

| CAN ID | Controller |
|--------|-----------|
| 0x0140 | #1 Sprayer rotation |
| 0x0141 | #2 Drive Left |
| 0x0142 | #3 Drive Right |
| 0x0143 | #4 Pump (**absent when pump has no HV**) |

| Byte | Field | Notes |
|------|-------|-------|
| 0 | Driver temp | ~0x40 at room temp |
| 1 | Motor temp | ~0x42 at room temp |
| 2-3 | Voltage ref | 0x3A98 constant |
| 4-5 | Load current | 0x000C idle |
| 6 | Reserved | |
| 7 | Checksum | |

## Motor Speed Feedback (10 Hz)

| CAN ID | Controller | Notes |
|--------|-----------|-------|
| 0x0181 | #1 | All zeros when stationary |
| 0x0182 | #2 | |
| 0x0183 | #3 | |

## Cell Voltages (1 Hz)

**0x0111:** 4x uint16 LE. Baseline 0x2710 (10000).

## VCU -> Motor Commands (10 Hz)

| CAN ID | Purpose | Key Fields |
|--------|---------|------------|
| 0x01E1 | Speed/torque setpoint | Bytes 0-1: cmd value (0x03E4 typical). Bytes 2-3: 0x2710 baseline. Bytes 4-5: 0x03E8 |
| 0x01E4 | Secondary command | Byte 1: mode byte |
| 0x0211 | Tertiary command | |
| 0x01F3-0x01F5 | Config/parameter writes | |

## Motor Config Messages (10 Hz)

| CAN ID | Purpose |
|--------|---------|
| 0x0190 | Config, byte 2 changes during boot |
| 0x0191 | Rolling counter |
| 0x0192 | 4x uint16 config constants |
| 0x0193 | Serial/firmware string |
| 0x0194 | Rare config (~0.5 Hz) |
| 0x019A | Status flags |
| 0x019C | Channel config (0.5 Hz): `01 00 00 01 00 00 00 00` |

## 0x019B - BMS Pack Status (10 Hz)

| Byte | Field | Scale/Values |
|------|-------|-------------|
| 0 | Pack voltage | ~0.5 V/count. 0x8D=141 -> ~70.5V |
| 1 | Reserved | 0x00 |
| 2 | Pack current | Raw. Spikes at boot, settles to ~0 at idle |
| 3 | **HV relay** | **0x00=OPEN, 0x01=CLOSED** |
| 4 | Pre-charge | 0x04=charging, 0x00=done |
| 5 | Reserved | |
| 6 | Channel enable | Bitmask. 0x08=bit3=main HV |
| 7 | Temperature | ~0x42=66 |

## 0x0351 - System State (10 Hz)

Byte 0 is a bitmask that builds up during boot:

| Value | Meaning |
|-------|---------|
| 0x04 | LV power on |
| 0x0C | Comms established |
| 0x2C | Pre-charge started |
| 0x2E | Motor controllers on bus |
| 0xAE | HV enabled |
| 0xAD | **Steady state (normal)** |
| 0x2C | HV shutdown |

## PDU / Power Messages

| CAN ID | Rate | Purpose |
|--------|------|---------|
| 0x0501 | 5 Hz | PDU status (usually zeros) |
| 0x0502 | 100 Hz | PDU live data |
| 0x0503 | 5 Hz | PDU secondary |

## BMS Extended (5 Hz, 29-bit IDs on Motor Bus)

| CAN ID | Purpose |
|--------|---------|
| 0x2001000 | BMS init (once at boot) |
| 0x2002000 | BMS voltage + current |
| 0x2003000 | BMS SOC + temperature |

**0x2002000:**

| Byte | Field | Notes |
|------|-------|-------|
| 0 | HV state | 0=off, 1=on |
| 1 | Current | Spikes at startup, settles to idle |
| 3 | Voltage | ~0.5V/count. 0x8D -> ~70.5V |
| 4 | Mode | 0x10=precharge, 0x11=running |
| 7 | Checksum | |

**0x2003000:**

| Byte | Field | Notes |
|------|-------|-------|
| 3 | Voltage | Mirrors 0x2002000 byte 3 |
| 4 | SOC % | 0x64=100% |
| 5 | SOH % | 0x64=100% |
| 6 | Temp 1 | Slowly increments |
| 7 | Temp 2 | ~0x42 |

## J1939 (boot only)

| CAN ID | Purpose |
|--------|---------|
| 0x18EEFFEB | Address Claimed (SA=0xEB) |
| 0x18EEFFEC | Address Claimed (SA=0xEC) |

---

# CAN C - NAV Bus (Extended 29-bit IDs)

All bitfield positions are from LSB of the 8-byte little-endian uint64 union.

**Scaling constants used throughout:**
- **RPM:** raw - 32000
- **Voltage:** raw * 0.1 (V)
- **Current:** raw * 0.1 - 1500 (A) for motors, raw * 0.1 - 1000 (A) for battery
- **Torque:** raw - 32000
- **Temperature:** raw - 40 (C) for motors, raw - 50 (C) for battery

## NAV -> VCU Commands

### 0x1001 - Motor Control (100 Hz from VCU driver, 10 Hz from NAV)

| Bits | Width | Field | Scale / Values |
|------|-------|-------|----------------|
| 0-7 | 8 | NAV_VCU_CheckSum | ~(B1 ^ B2 ^ B3 ^ B4 ^ B5 ^ B6 ^ B7) |
| 8-14 | 7 | NAV_VCU_TickVal | Rolling counter 0-127 |
| 15 | 1 | NAV_Ctrl_EN | 0=disabled, 1=enabled |
| 16-31 | 16 | NAV_WheelL_Veloc | int16 mm/s |
| 32-47 | 16 | NAV_WheelR_Or_LineVeloc | int16 mm/s |
| 48-59 | 12 | NAV_AngularVeloc | raw * 0.001 - 0.873 rad/s |
| 60 | 1 | NAV_Speed_Ctr_Mode | 0=differential, 1=line+angular |
| 61 | 1 | NAV_Brake | 1=brake |
| 62-63 | 2 | NAV_Lock | Lock command |

**Watchdog:** VCU zeros CMD1 payload if no NAV update for >1 second.

### 0x1002 - Pump / Accessory Control (10 Hz)

| Bits | Width | Field | Values |
|------|-------|-------|--------|
| 0-3 | 4 | NAV_PUMP_Speed_Lvl | 0-15 |
| 4-7 | 4 | NAV_SWAY_Speed_Lvl | 0-15 |
| 8-11 | 4 | NAV_GSCUT_Speed_Lvl | 0-15 |
| 12-13 | 2 | NAV_Ctrl_GrassCut_Module | 0-3 |
| 14-15 | 2 | **NAV_Ctrl_MedicSpray_Module** | **0=OFF, 1=IDLE, 2=SPRAY, 3=AUTO** |
| 16 | 1 | NAV_Switch_SolenoidValveL | 0/1 |
| 17 | 1 | NAV_Switch_SolenoidValveR | 0/1 |
| 18 | 1 | NAV_Switch_LightTurnLeft | 0/1 |
| 19 | 1 | NAV_Switch_LightTurnRight | 0/1 |
| 20 | 1 | NAV_Switch_LightHead | 0/1 |
| 21 | 1 | NAV_Switch_SoundLampAlarm | 0/1 |
| 22 | 1 | VCU_Fan01Ctrl | 0/1 |
| 23 | 1 | VCU_Fan02Ctrl | 0/1 |
| 24 | 1 | VCU_Fan03Ctrl | 0/1 |
| 25 | 1 | VCU_Fan04Ctrl | 0/1 |
| 26 | 1 | NAV_EmergPowerOff | 0/1 |
| 27-63 | 37 | Reserved | |

## VCU -> NAV Feedback

### 0x1003 - VCU Version (0.3 Hz)

Bytes 0-2: HW version A.B.C. Bytes 3-5: SW version A.B.C.

### 0x1004 / 0x1005 - VIN Code (0.3 Hz)

Vehicle identification. See `data_vcu_can.h` for full fields.

### 0x1006 - Wheel Speed Feedback (100 Hz)

| Bits | Width | Field | Scale |
|------|-------|-------|-------|
| 0-15 | 16 | VCU_WheelL_Veloc | int16 mm/s |
| 16-31 | 16 | VCU_WheelR_Or_LineVeloc | int16 mm/s |
| 32-43 | 12 | VCU_AngularVeloc | *0.001 - 0.873 rad/s |
| 44 | 1 | VCU_Speed_Ctr_Mode | |
| 45 | 1 | VCU_Brake | 1=braking |
| 46-47 | 2 | VCU_Lock | |
| 48-51 | 4 | VCU_PUMP_Speed_Lvl | 0-15 |
| 52-55 | 4 | VCU_SWAY_Speed_Lvl | 0-15 |
| 56-59 | 4 | VCU_GrassCut_Speed_Lvl | 0-15 |
| 60-63 | 4 | VCU_TickVal | Counter 0-15 |

### 0x1007 - Module Status / Safety Flags (5 Hz)

| Bits | Width | Field | Values |
|------|-------|-------|--------|
| 0-1 | 2 | VCU_GrassCutModule_CtrlSta | 0-3 |
| 2-3 | 2 | **VCU_MedicSprayModule_CtrlSta** | **0=OFF, 1=IDLE, 2=SPRAY, 3=AUTO** |
| 4 | 1 | VCU_Switch_SolenoidValveL | |
| 5 | 1 | VCU_Switch_SolenoidValveR | |
| 6 | 1 | VCU_Switch_LightTurnLeft | |
| 7 | 1 | VCU_Switch_LightTurnRight | |
| 8 | 1 | VCU_Switch_LightHead | |
| 9 | 1 | VCU_Switch_SoundLampAlarm | |
| 10 | 1 | **VCU_CollisionSta** | **1 = blocks pump** |
| 11 | 1 | **VCU_RC_SwitchStopSta** | **1 = blocks pump** |
| 12 | 1 | VCU_WalkMotSta | |
| 13 | 1 | **VCU_ForceStopSta** | **1 = blocks pump** |
| 14 | 1 | **VCU_HwEmergStopSwitchSta** | **1 = blocks pump** |
| 15 | 1 | VCU_WalkMotHVInterlockSta | |
| 16 | 1 | VCU_RearMotHVInterlockSta | |
| 17-18 | 2 | VCU_HVonSta | ==1 blocks pump |
| 19-23 | 5 | VCU_WorkType | 1=spray, 3=other |
| 24-31 | 8 | VCU_MedicLeft | 0-255 medicine level % |
| 36-47 | 12 | VCU_LV_Batt_Volt | *0.1 - 60 = V |
| 48-49 | 2 | VCU_HV_SelfTestSta | |
| 50-51 | 2 | VCU_HV_PreChargeSta | 3=done |
| 52-53 | 2 | VCU_HV_ChargeDischageSta | >0 blocks pump |
| 54 | 1 | VCU_HV_HighVoltOnSta | |
| 55-56 | 2 | DCDC_VCU_Sta | |
| 57-59 | 3 | **VCU_RobotCtrlMode** | **1=REMOTE, 2=AUTO_NAV** |
| 59-66 | 8 | DCDC_VCU_Temper | raw - 40 = C |

### 0x1008 - HV Battery (5 Hz)

| Bits | Width | Field | Scale |
|------|-------|-------|-------|
| 0-7 | 8 | VCU_HV_Model | |
| 8-15 | 8 | VCU_HV_SOC | 0-100% |
| 16-23 | 8 | VCU_HV_SOH | 0-100% |
| 24-39 | 16 | VCU_HV_Volt | *0.1 = V |
| 40-55 | 16 | VCU_HV_Curr | *0.1 - 1000 = A |
| 56-63 | 8 | VCU_HV_Temper | raw - 50 = C |

### 0x1009 - Communication Status Flags (5 Hz)

1 = fault/offline for comm bits.

| Bit | Field |
|-----|-------|
| 0 | VCU_ComWalkMotDrvL |
| 1 | VCU_ComWalkMotDrvR |
| 2 | **VCU_ComMedicPumpMotDrv** |
| 3 | VCU_ComSwayMotDrv |
| 4 | VCU_ComGrassCutMotDrv |
| 5 | VCU_ComPushMotDrv |
| 6 | **VCU_ComNav** |
| 7 | VCU_ComRC |
| 8 | VCU_ComPDU |
| 9 | VCU_ComDCDC |
| 10 | VCU_ComHighVoltBMS |
| 11-14 | Fan01-04 CtrlSta |
| 15-18 | UltrasonicSens01-04 Sta |

### Motor Feedback ID Map

| Motor | Tgt/Fbk RPM | Bus V / Bus I | Torque / Temps |
|-------|-------------|---------------|----------------|
| Left Drive | 0x100A bytes 4-7 | 0x100B bytes 0-3 | 0x100B bytes 4-7 |
| Right Drive | 0x100C bytes 0-3 | 0x100C bytes 4-7 | 0x100D bytes 0-3 |
| **Medicine Pump** | **0x100D bytes 4-7** | **0x100E bytes 0-3** | **0x100E bytes 4-7** |
| Grass Cutter | 0x100F bytes 0-3 | 0x100F bytes 4-7 | 0x1010 bytes 0-3 |
| Sway Motor | 0x1010 bytes 4-7 | 0x1011 bytes 0-3 | 0x1011 bytes 4-7 |

All RPM: raw - 32000. Voltage: raw * 0.1 V. Current: raw * 0.1 - 1500 A. Temp: raw - 40 C.

### 0x1012 - Error Codes 1 (10 Hz)

Each: 4-bit error level + 8-bit error code. **Level > 2 = critical.**

| Bits | Field |
|------|-------|
| 0-3 / 4-11 | WalkMotL Level / Code |
| 12-15 / 16-23 | WalkMotR Level / Code |
| 24-27 / 28-35 | **MedicPumpMot Level / Code** |
| 36-39 / 40-47 | GrassCutMot Level / Code |
| 48-51 / 52-59 | PushMot Level / Code |

### 0x1013 - Error Codes 2 (10 Hz)

| Bits | Field |
|------|-------|
| 0-3 / 4-11 | SwayMot Level / Code |
| 12-15 / 16-23 | PDU Level / Code |
| 24-27 / 28-35 | HVBatt Level / Code |
| 36-39 / 40-47 | DCDC Level / Code |
| 48-51 / 52-59 | VCU_Self Level / Code (level 4 = fatal) |

### 0x3100 - PDU Relay Status (10 Hz)

| Bits | Width | Field | Notes |
|------|-------|-------|-------|
| 0-1 | 2 | PDU_SelfCheck | 1=OK |
| 2-3 | 2 | PDU_K1_Sta | 1=closed |
| 4-5 | 2 | PDU_K2_Sta | **0=OPEN when PushMot ErrLvl=3** |
| 6-7 | 2 | PDU_K3_Sta | 1=closed |
| 8-9 | 2 | PDU_K4_Sta | 1=closed |
| 10-11 | 2 | PDU_EmergStopSig_Sta | |
| 12-13 | 2 | PDU_PreChargeFinish_Sta | 1=done |
| 14 | 1 | PDU_HighVoltInterLock_Sta | |
| 15 | 1 | PDU_ExeDischarge_Sta | |

Normal steady state: K1=1, K2=0 (push motor fault), K3=1, K4=1.

### 0x3101 - PDU Voltages (10 Hz)

4x uint16 LE: K1_Front_Volt, K1_Back_Volt, K3_Back_Volt, K4_Back_Volt.

### 0x3102 - PDU Currents (10 Hz)

4x uint16 LE: LeftMotor, RightMotor, RearMotor, ExtSupply.

### 0x3103 - BMS Times (0.3 Hz)

2x uint32 LE: CumuTimeDischarge, CumuTimeCharge (seconds).

### 0x3104 - BMS Power (5 Hz)

uint32 CumuTimeAll + uint16 BatteryPower + uint16 CycleTimes.

---

# Medicine Pump Control Logic

From `MedicineCtrl.cpp`, `Robot.cpp`.

## CanUpMedicine() return values (CAN mode, vcuInterfaceType==2)

| Return | MedicSpray State | Condition |
|--------|-----------------|-----------|
| 3 | AUTO | `autoSpraying == 1` |
| 2 | SPRAY (active) | Task active + between waypoints + long route + no errors + pump ErrLvl <= 2 |
| 1 | IDLE (disabled) | Any error, wrong state, or wrong driver type |

## checkerror() - What Blocks the Pump

Returns true (ALL actuators disabled) if ANY flag is nonzero:

**From 0x1007 (emergency flags):**
| Index | Source Field | Meaning |
|-------|------------|---------|
| [0] | VCU_Brake | Braking |
| [1] | VCU_CollisionSta | Collision detected |
| [2] | VCU_RC_SwitchStopSta | RC stop pressed |
| [3] | VCU_ComNav && VCU_ComRC | Both NAV and RC offline |
| [5] | VCU_HwEmergStopSwitchSta | Hardware e-stop |
| [6] | VCU_HV_ChargeDischageSta | Charging/discharging |
| [7] | VCU_HVonSta == 1 | HV fault |

**From 0x1012/serial (device faults):**
| Index | Fault |
|-------|-------|
| [0] | Left walk motor |
| [1] | Right walk motor |
| [2] | Medicine pump motor |
| [3] | Swing motor |
| [4] | HV battery |
| [5] | IPC offline |
| [7] | Grass cutter |

## Watchdog Timeouts

- **NAV -> VCU:** CMD1 must arrive every <1s or VCU zeros payload
- **VCU -> NAV:** Feedback must arrive every <1s or NAV sets Get_Botton_Info_Error
- Both timeouts trigger checkerror() -> pump disabled

## Control Mode Gate

- `VCU_RobotCtrlMode == 1` (REMOTE): VCU accepts RC commands, NAV auto-pauses
- `VCU_RobotCtrlMode == 2` (AUTO_NAV): VCU accepts NAV commands, ignores RC pump input

---

# Debug Checklist - Pump Not Working

Check in this order:

| # | Bus | CAN ID | Field | Problem if |
|---|-----|--------|-------|-----------|
| 1 | C | 0x1007 | VCU_RobotCtrlMode | != expected mode (1=RC, 2=NAV) |
| 2 | C | 0x1007 | CollisionSta / RC_Stop / ForceStop / HwEStop | Any = 1 |
| 3 | C | 0x1012 | VCU_ErrLevel_MedicPumpMot | > 2 |
| 4 | C | 0x1009 | VCU_ComMedicPumpMotDrv | = 1 (offline) |
| 5 | C | 0x100D | VCU_MeicPumpMot_TgtRPM | = 32000 (0 RPM, not commanded) |
| 6 | C | 0x100E | VCU_MeicPumpMot_BusVolt | = 0 (no HV at pump) |
| 7 | C | 0x3100 | PDU K1-K4 | Expected relay open |
| 8 | A | 0x0122 | byte[0] | 0x01=no HV, 0x09=OK, 0x21=fault |
| 9 | A | 0x0133 | presence | Absent = pump controller not reporting electrical |
| 10 | C | 0x1007 | VCU_HV_ChargeDischageSta | > 0 (charging blocks pump) |

---

# Known Issues (2026-04-06)

## Root Cause: Pump Not Spraying

**RC firmware had an inverted switch** causing `VCU_RobotCtrlMode = 2` (AUTO_NAV) when operator expected REMOTE mode. In AUTO_NAV, VCU ignores RC pump commands and waits for NAV CMD2. Since NAV wasn't sending spray commands, pump target RPM stayed at 0.

Additionally `VCU_CollisionSta = 1` was active, which blocks the pump via `checkerror()` regardless of control mode.

**Fix:** Re-flash RC firmware with corrected switch polarity. Investigate collision sensor.

## Other: PushMot ErrLvl = 3

Push motor has a persistent critical error (level 3), causing PDU K2 relay to stay open. Separate issue from pump. Needs investigation.

---

# Power On / Power Off Sequence

## Physical Controls

- **HV power button** - Enables high-voltage system
- **2x Emergency stop buttons** - Each has 2 pairs of cables (NC contacts). Cuts power when pressed.
- **Start button** - Activates the system after HV and e-stops are cleared

## Power ON Procedure

1. HV power button ON
2. Release emergency stops (if set)
3. Press Start button

## Power OFF Procedure

Reverse of power on:
1. Press emergency stop (or stop button)
2. HV power button OFF

## CAN Signals During Power Sequence

### CAN A (Motor Bus)

**0x0160 byte[0]** - Power/ignition state:
| Value | Meaning |
|-------|---------|
| 0x00 | Power OFF / standby |
| 0x01 | Power ON (system alive) |

Transitions observed:
- Power ON: 0x0160 goes 0x00 -> 0x01, then 0x0351 climbs to 0xAD (normal)
- Power OFF: 0x0160 goes 0x01 -> 0x00, all motors go 0x21 (fault) -> 0x01 (standby), 0x0351 drops to 0x2C, bus dies ~0.5s later

### CAN C (NAV Bus)

| CAN ID | Field | Meaning |
|--------|-------|---------|
| 0x1002 bit 26 | NAV_EmergPowerOff | NAV commanding VCU to shut down |
| 0x1007 bit 13 | VCU_ForceStopSta | Force stop active |
| 0x1007 bit 14 | VCU_HwEmergStopSwitchSta | Hardware e-stop pressed |
| 0x3100 bits 10-11 | PDU_EmergStopSig_Sta | PDU emergency stop signal |

### TODO (2026-04-07)

- Map which CAN signals correspond to each physical button
- Identify which e-stop button maps to which signal
- Document the full relay sequence (K1-K4) during power on/off
- Check CAN T port purpose
