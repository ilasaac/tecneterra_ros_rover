# Rosetta CAN Protocol Guide - Rover

This document provides the decoding and control logic for the rover's CAN bus system.

## 1. Control Commands (Navigation -> VCU)
All commands use a **8-byte payload**.

### **0x1001 - Motion Control (NAV_VCU_CMD1)**
| Byte | Bits | Field | Description / Scaling |
| :--- | :--- | :--- | :--- |
| 0 | 0-7 | Checksum | \~(Byte1 ^ Byte2 ^ ... ^ Byte7)\ |
| 1 | 0-6 | Tick Value | Rolling counter (0-127) |
| 1 | 7 | Ctrl Enable | 1 = Enable Control, 0 = Disable |
| 2-3 | 0-15 | Left Wheel Velocity | Raw Value (Offset: 1600) |
| 4-5 | 0-15 | Right Wheel Velocity | Raw Value (Offset: 1600) |
| 6 | 0-3 | Angular Velocity | (Lower 4 bits of 12-bit field) |
| 7 | 4-7 | Angular Velocity | (Upper 8 bits of 12-bit field) |
| 7 | 4 | Speed Ctrl Mode | 0 = Torque?, 1 = Speed? |
| 7 | 5 | Brake | 1 = Active |
| 7 | 6-7 | Lock | Differential lock status |

### **0x1002 - Module Control (NAV_VCU_CMD2)**
| Byte | Bits | Field | Description |
| :--- | :--- | :--- | :--- |
| 0 | 0-3 | Pump Speed Level | 0-15 |
| 0 | 4-7 | Sway Speed Level | 0-15 |
| 1 | 0-3 | Grass Cut Speed Level | 0-15 |
| 1 | 4-5 | Grass Cut Module | 0=Off, 1=On, 2=Auto |
| 1 | 6-7 | Medic Spray Module | 0=Off, 1=On, 2=Auto |
| 2 | 0 | Solenoid Valve Left | 1 = Open |
| 2 | 1 | Solenoid Valve Right | 1 = Open |
| 2 | 2 | Light Turn Left | 1 = On |
| 2 | 3 | Light Turn Right | 1 = On |
| 2 | 4 | Headlight | 1 = On |
| 2 | 5 | Sound/Lamp Alarm | 1 = On |
| 2 | 6 | Fan 1-4 Ctrl | 4 bits for individual fans |
| 3 | 2 | Emergency Power Off | 1 = Kill Power |

---

## 2. Motor Feedback (VCU -> Navigation)

**Scaling Legend:** 
- **RPM:** \Raw - 32000\ (Center point is 32000)
- **Voltage:** \Raw * 0.1\ (V)
- **Current:** \Raw * 0.1 - 1500\ (A)
- **Torque:** \Raw - 32000\ (Nm)
- **Temperature:** \Raw - 40\ (°C)

### **Motor Feedback ID Mapping:**
| Motor | Tgt/Fbk RPM | Bus V / Bus I | Torque / Temp |
| :--- | :--- | :--- | :--- |
| **Left Drive** | 0x100A (Bytes 4-7) | 0x100B (Bytes 0-3) | 0x100B (Bytes 4-7) |
| **Right Drive**| 0x100C (Bytes 0-3) | 0x100C (Bytes 4-7) | 0x100D (Bytes 0-3) |
| **Medic Pump** | 0x100D (Bytes 4-7) | 0x100E (Bytes 0-3) | 0x100E (Bytes 4-7) |
| **Grass Cutter**| 0x100F (Bytes 0-3) | 0x100F (Bytes 4-7) | 0x1010 (Bytes 0-3) |
| **Sway Motor** | 0x1010 (Bytes 4-7) | 0x1011 (Bytes 0-3) | 0x1011 (Bytes 4-7) |

---

## 3. System Status & Battery

### **0x1012 / 0x1013 - Error Codes**
- **0x1012:** \Level\ (4 bits) + \Code\ (8 bits) for Left Motor, Right Motor, Pump, Grass Cutter, and Push Motor.
- **0x1013:** \Level\ + \Code\ for Sway Motor, PDU, HV Battery, DCDC, and VCU Self-check.

### **0x3100 - 0x3102 - PDU (Power Distribution Unit)**
- **0x3100:** Contactors (K1-K4), Emergency Stop, Pre-charge status.
- **0x3101:** Voltages (K1 Front/Back, K3/K4 Back).
- **0x3102:** Currents (Left/Right Motor, Rear Motor, External Supply).

### **0x3103 - 0x3104 - BMS (Battery Management System)**
- **0x3103:** Cumulative Discharge/Charge Time (32-bit fields).
- **0x3104:** Total Time (32-bit), Battery Power (16-bit), Cycles (16-bit).

---

## 4. Aggregated Status

### **0x1006 (FBK_INFO1)**
- Actual Wheel Velocities, Angular Velocity, Brake, and Lock status.

### **0x1007 (FBK_INFO2)**
- Control Mode, Collision status, LV Battery Voltage (\*0.1 - 60\), Medic Level.

### **0x1008 (FBK_INFO3)**
- HV Battery: SOC, SOH, Volt (\*0.1\), Current (\*0.1 - 1000\), Temp (\-50\).
