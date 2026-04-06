#!/usr/bin/env python3
"""
TTR Agro Rover - Live CAN Dashboard (RPi + MCP2515 via SocketCAN)
==================================================================
Setup on RPi:
  sudo ip link set can0 up type can bitrate 250000
  pip3 install python-can curses-is-builtin
  python3 can_dashboard.py          # defaults to can0
  python3 can_dashboard.py can1     # or specify interface
  python3 can_dashboard.py --file dump.log   # replay candump log

Keys: q=quit, p=pause, r=reset counters, 1-6=switch tab, UP/DOWN=scroll
"""

import sys
import os
import time
import curses
import threading
import struct
from collections import OrderedDict, defaultdict
from datetime import datetime

# ---------------------------------------------------------------------------
# CAN interface - python-can or fallback to candump pipe
# ---------------------------------------------------------------------------
try:
    import can as python_can
    HAS_PYTHON_CAN = True
except ImportError:
    HAS_PYTHON_CAN = False

# ---------------------------------------------------------------------------
# Bitfield helpers (little-endian uint64 unions, matching C structs)
# ---------------------------------------------------------------------------
def _u64(data: bytes) -> int:
    return int.from_bytes(data[:8], byteorder='little')

def _bits(val: int, start: int, width: int) -> int:
    return (val >> start) & ((1 << width) - 1)

def _s16(raw: int) -> int:
    return raw - 0x10000 if raw >= 0x8000 else raw

# ---------------------------------------------------------------------------
# Alert levels
# ---------------------------------------------------------------------------
ALERT_NONE = 0
ALERT_WARN = 1
ALERT_CRIT = 2

def _a(val, thresh_warn=1, thresh_crit=None):
    """Return (value, alert_level)."""
    if thresh_crit is not None and val >= thresh_crit:
        return (val, ALERT_CRIT)
    if val >= thresh_warn:
        return (val, ALERT_WARN)
    return (val, ALERT_NONE)

# ---------------------------------------------------------------------------
# Per-ID decoders: return list of (label, value, alert_level)
# ---------------------------------------------------------------------------

def dec_1001(d):
    v = _u64(d)
    en = _bits(v, 15, 1)
    brk = _bits(v, 61, 1)
    raw_ang = _bits(v, 48, 12)
    return [
        ("Checksum",        _bits(v, 0, 8),                   ALERT_NONE),
        ("Tick",            _bits(v, 8, 7),                    ALERT_NONE),
        ("NAV_Ctrl_EN",     en,                                ALERT_WARN if en == 0 else ALERT_NONE),
        ("WheelL mm/s",     _s16(_bits(v, 16, 16)),            ALERT_NONE),
        ("WheelR mm/s",     _s16(_bits(v, 32, 16)),            ALERT_NONE),
        ("AngVel rad/s",    round(raw_ang * 0.001 - 0.873, 3), ALERT_NONE),
        ("SpeedCtrlMode",   _bits(v, 60, 1),                   ALERT_NONE),
        ("Brake",           brk,                                ALERT_CRIT if brk else ALERT_NONE),
        ("Lock",            _bits(v, 62, 2),                    ALERT_NONE),
    ]

def dec_1002(d):
    v = _u64(d)
    spray = _bits(v, 14, 2)
    spray_s = {0: "OFF", 1: "IDLE", 2: "SPRAY", 3: "AUTO"}
    pump_lvl = _bits(v, 0, 4)
    epoff = _bits(v, 26, 1)
    return [
        ("PUMP Speed Lvl",   pump_lvl,                         ALERT_NONE),
        ("SWAY Speed Lvl",   _bits(v, 4, 4),                   ALERT_NONE),
        ("GSCUT Speed Lvl",  _bits(v, 8, 4),                   ALERT_NONE),
        ("GrassCut Module",  _bits(v, 12, 2),                   ALERT_NONE),
        ("MedicSpray Mode",  spray_s.get(spray, "?"),           ALERT_CRIT if spray <= 1 else ALERT_NONE),
        ("SolenoidValve L",  _bits(v, 16, 1),                   ALERT_NONE),
        ("SolenoidValve R",  _bits(v, 17, 1),                   ALERT_NONE),
        ("Light L/R/Head",   f"{_bits(v,18,1)}/{_bits(v,19,1)}/{_bits(v,20,1)}", ALERT_NONE),
        ("Alarm",            _bits(v, 21, 1),                   ALERT_NONE),
        ("EmergPowerOff",    epoff,                              ALERT_CRIT if epoff else ALERT_NONE),
    ]

def dec_1003(d):
    return [
        ("HW Version", f"{d[0]}.{d[1]}.{d[2]}", ALERT_NONE),
        ("SW Version", f"{d[3]}.{d[4]}.{d[5]}", ALERT_NONE),
    ]

def dec_1006(d):
    v = _u64(d)
    raw_ang = _bits(v, 32, 12)
    brk = _bits(v, 45, 1)
    return [
        ("WheelL mm/s",      _s16(_bits(v, 0, 16)),             ALERT_NONE),
        ("WheelR mm/s",      _s16(_bits(v, 16, 16)),            ALERT_NONE),
        ("AngVel rad/s",     round(raw_ang * 0.001 - 0.873, 3), ALERT_NONE),
        ("SpeedCtrlMode",    _bits(v, 44, 1),                    ALERT_NONE),
        ("Brake",            brk,                                 ALERT_CRIT if brk else ALERT_NONE),
        ("Lock",             _bits(v, 46, 2),                     ALERT_NONE),
        ("PUMP Spd Lvl",    _bits(v, 48, 4),                     ALERT_NONE),
        ("SWAY Spd Lvl",    _bits(v, 52, 4),                     ALERT_NONE),
        ("GSCUT Spd Lvl",   _bits(v, 56, 4),                     ALERT_NONE),
        ("Tick",             _bits(v, 60, 4),                     ALERT_NONE),
    ]

def dec_1007(d):
    v = _u64(d)
    ctrl = _bits(v, 56, 3)
    ctrl_s = {0: "?", 1: "REMOTE", 2: "AUTO_NAV"}
    collision  = _bits(v, 10, 1)
    rc_stop    = _bits(v, 11, 1)
    force_stop = _bits(v, 13, 1)
    hw_estop   = _bits(v, 14, 1)
    hv_charge  = _bits(v, 52, 2)
    hv_on      = _bits(v, 54, 1)
    raw_batt   = _bits(v, 36, 12)
    medic_left = _bits(v, 24, 8)
    spray_sta  = _bits(v, 2, 2)
    spray_s    = {0: "OFF", 1: "IDLE", 2: "SPRAY", 3: "AUTO"}
    return [
        ("CtrlMode",          ctrl_s.get(ctrl, f"?{ctrl}"),      ALERT_WARN if ctrl == 1 else ALERT_NONE),
        ("MedicSpray Sta",    spray_s.get(spray_sta, "?"),        ALERT_CRIT if spray_sta <= 1 else ALERT_NONE),
        ("GrassCut Sta",      _bits(v, 0, 2),                     ALERT_NONE),
        ("SolValve L/R",      f"{_bits(v,4,1)}/{_bits(v,5,1)}",   ALERT_NONE),
        ("Collision!",        collision,                           ALERT_CRIT if collision else ALERT_NONE),
        ("RC Stop!",          rc_stop,                             ALERT_CRIT if rc_stop else ALERT_NONE),
        ("WalkMotSta",        _bits(v, 12, 1),                     ALERT_NONE),
        ("ForceStop!",        force_stop,                          ALERT_CRIT if force_stop else ALERT_NONE),
        ("HW E-Stop!",        hw_estop,                            ALERT_CRIT if hw_estop else ALERT_NONE),
        ("WalkMot HV Intlk",  _bits(v, 15, 1),                    ALERT_NONE),
        ("RearMot HV Intlk",  _bits(v, 16, 1),                    ALERT_NONE),
        ("HV On Sta",         _bits(v, 17, 2),                     ALERT_WARN if _bits(v,17,2) == 1 else ALERT_NONE),
        ("WorkType",          _bits(v, 19, 5),                     ALERT_NONE),
        ("MedicLeft",         medic_left,                          ALERT_WARN if medic_left < 20 else ALERT_NONE),
        ("LV Batt V",         round(raw_batt * 0.1 - 60, 1),      ALERT_WARN if raw_batt * 0.1 - 60 < 22 else ALERT_NONE),
        ("HV SelfTest",       _bits(v, 48, 2),                     ALERT_NONE),
        ("HV PreCharge",      _bits(v, 50, 2),                     ALERT_NONE),
        ("HV Charge/Disch",   hv_charge,                           ALERT_WARN if hv_charge else ALERT_NONE),
        ("HV HighVoltOn",     hv_on,                               ALERT_NONE),
        ("DCDC Sta",          _bits(v, 55, 2),                     ALERT_NONE),
        ("DCDC Temp C",       _bits(v, 59, 8) - 40,                ALERT_WARN if _bits(v,59,8)-40 > 70 else ALERT_NONE),
    ]

def dec_1008(d):
    v = _u64(d)
    soc = _bits(v, 8, 8)
    volt = round(_bits(v, 24, 16) * 0.1, 1)
    curr = round(_bits(v, 40, 16) * 0.1 - 1000, 1)
    temp = _bits(v, 56, 8) - 50
    return [
        ("HV Model",    _bits(v, 0, 8),  ALERT_NONE),
        ("SOC %",        soc,             ALERT_CRIT if soc < 10 else ALERT_WARN if soc < 20 else ALERT_NONE),
        ("SOH %",        _bits(v, 16, 8), ALERT_WARN if _bits(v,16,8) < 50 else ALERT_NONE),
        ("HV Volt V",    volt,            ALERT_NONE),
        ("HV Curr A",    curr,            ALERT_NONE),
        ("HV Temp C",    temp,            ALERT_CRIT if temp > 55 else ALERT_WARN if temp > 45 else ALERT_NONE),
    ]

def dec_1009(d):
    v = _u64(d)
    labels = [
        "ComWalkMotDrvL", "ComWalkMotDrvR", "ComMedicPumpDrv",
        "ComSwayMotDrv", "ComGrassCutDrv", "ComPushMotDrv",
        "ComNav", "ComRC", "ComPDU", "ComDCDC", "ComHVBMS",
    ]
    result = []
    for i, lbl in enumerate(labels):
        val = _bits(v, i, 1)
        # For comm flags: 1 = fault/lost
        alert = ALERT_CRIT if val else ALERT_NONE
        result.append((lbl, val, alert))
    for i in range(4):
        result.append((f"Fan0{i+1} Sta", _bits(v, 11+i, 1), ALERT_NONE))
    for i in range(4):
        result.append((f"Ultrasonic0{i+1}", _bits(v, 15+i, 1), ALERT_NONE))
    return result

def dec_100A(d):
    v = _u64(d)
    return [
        ("DCDC Volt V",       round(_bits(v, 0, 16) * 0.1, 1),  ALERT_NONE),
        ("DCDC Curr A",       round(_bits(v, 16, 16) * 0.1, 1), ALERT_NONE),
        ("WalkMotL TgtRPM",   _s16(_bits(v, 32, 16)),            ALERT_NONE),
        ("WalkMotL FbkRPM",   _s16(_bits(v, 48, 16)),            ALERT_NONE),
    ]

def dec_100B(d):
    v = _u64(d)
    drv_t = _bits(v, 48, 8) - 40
    mot_t = _bits(v, 56, 8) - 40
    return [
        ("MotL BusVolt V",   round(_bits(v, 0, 16) * 0.1, 1),     ALERT_NONE),
        ("MotL BusCurr A",   round(_bits(v, 16, 16) * 0.1 - 1500, 1), ALERT_NONE),
        ("MotL Torque",       _s16(_bits(v, 32, 16)),                ALERT_NONE),
        ("MotL Drv Temp C",   drv_t,  ALERT_CRIT if drv_t > 80 else ALERT_WARN if drv_t > 65 else ALERT_NONE),
        ("MotL Temp C",       mot_t,  ALERT_CRIT if mot_t > 100 else ALERT_WARN if mot_t > 80 else ALERT_NONE),
    ]

def dec_100C(d):
    v = _u64(d)
    return [
        ("WalkMotR TgtRPM",   _s16(_bits(v, 0, 16)),               ALERT_NONE),
        ("WalkMotR FbkRPM",   _s16(_bits(v, 16, 16)),              ALERT_NONE),
        ("MotR BusVolt V",    round(_bits(v, 32, 16) * 0.1, 1),    ALERT_NONE),
        ("MotR BusCurr A",    round(_bits(v, 48, 16) * 0.1 - 1500, 1), ALERT_NONE),
    ]

def dec_100D(d):
    v = _u64(d)
    tgt = _s16(_bits(v, 32, 16))
    fbk = _s16(_bits(v, 48, 16))
    drv_t = _bits(v, 16, 8) - 40
    mot_t = _bits(v, 24, 8) - 40
    return [
        ("MotR Torque",        _s16(_bits(v, 0, 16)),  ALERT_NONE),
        ("MotR Drv Temp C",    drv_t,                   ALERT_CRIT if drv_t > 80 else ALERT_WARN if drv_t > 65 else ALERT_NONE),
        ("MotR Temp C",        mot_t,                   ALERT_CRIT if mot_t > 100 else ALERT_WARN if mot_t > 80 else ALERT_NONE),
        ("PUMP TgtRPM",        tgt,                     ALERT_NONE),
        ("PUMP FbkRPM",        fbk,                     ALERT_WARN if tgt != 0 and fbk == 0 else ALERT_NONE),
    ]

def dec_100E(d):
    v = _u64(d)
    bus_v = round(_bits(v, 0, 16) * 0.1, 1)
    bus_c = round(_bits(v, 16, 16) * 0.1 - 1500, 1)
    drv_t = _bits(v, 48, 8) - 40
    mot_t = _bits(v, 56, 8) - 40
    return [
        ("PUMP BusVolt V",    bus_v,                    ALERT_NONE),
        ("PUMP BusCurr A",    bus_c,                    ALERT_NONE),
        ("PUMP Torque",        _s16(_bits(v, 32, 16)),  ALERT_NONE),
        ("PUMP Drv Temp C",    drv_t,  ALERT_CRIT if drv_t > 80 else ALERT_WARN if drv_t > 65 else ALERT_NONE),
        ("PUMP Mot Temp C",    mot_t,  ALERT_CRIT if mot_t > 100 else ALERT_WARN if mot_t > 80 else ALERT_NONE),
    ]

def dec_100F(d):
    v = _u64(d)
    return [
        ("GCut TgtRPM",     _s16(_bits(v, 0, 16)),               ALERT_NONE),
        ("GCut FbkRPM",     _s16(_bits(v, 16, 16)),              ALERT_NONE),
        ("GCut BusVolt V",  round(_bits(v, 32, 16) * 0.1, 1),   ALERT_NONE),
        ("GCut BusCurr A",  round(_bits(v, 48, 16) * 0.1 - 1500, 1), ALERT_NONE),
    ]

def dec_1010(d):
    v = _u64(d)
    drv_t = _bits(v, 16, 8) - 40
    mot_t = _bits(v, 24, 8) - 40
    return [
        ("GCut Torque",       _s16(_bits(v, 0, 16)),   ALERT_NONE),
        ("GCut Drv Temp C",   drv_t,                    ALERT_WARN if drv_t > 65 else ALERT_NONE),
        ("GCut Mot Temp C",   mot_t,                    ALERT_WARN if mot_t > 80 else ALERT_NONE),
        ("Sway TgtRPM",       _s16(_bits(v, 32, 16)),  ALERT_NONE),
        ("Sway FbkRPM",       _s16(_bits(v, 48, 16)),  ALERT_NONE),
    ]

def dec_1011(d):
    v = _u64(d)
    drv_t = _bits(v, 48, 8) - 40
    mot_t = _bits(v, 56, 8) - 40
    return [
        ("Sway BusVolt V",   round(_bits(v, 0, 16) * 0.1, 1),         ALERT_NONE),
        ("Sway BusCurr A",   round(_bits(v, 16, 16) * 0.1 - 1500, 1), ALERT_NONE),
        ("Sway Torque",       _s16(_bits(v, 32, 16)),                   ALERT_NONE),
        ("Sway Drv Temp C",   drv_t,  ALERT_WARN if drv_t > 65 else ALERT_NONE),
        ("Sway Mot Temp C",   mot_t,  ALERT_WARN if mot_t > 80 else ALERT_NONE),
    ]

def dec_1012(d):
    v = _u64(d)
    def _err(name, off):
        lvl = _bits(v, off, 4)
        code = _bits(v, off + 4, 8)
        alert = ALERT_CRIT if lvl > 2 else ALERT_WARN if lvl > 0 else ALERT_NONE
        return [(f"{name} ErrLvl", lvl, alert), (f"{name} ErrCode", code, alert)]
    return (
        _err("WalkMotL", 0) + _err("WalkMotR", 12) +
        _err("MedicPump", 24) + _err("GrassCut", 36) + _err("PushMot", 48)
    )

def dec_1013(d):
    v = _u64(d)
    def _err(name, off):
        lvl = _bits(v, off, 4)
        code = _bits(v, off + 4, 8)
        alert = ALERT_CRIT if lvl > 2 else ALERT_WARN if lvl > 0 else ALERT_NONE
        return [(f"{name} ErrLvl", lvl, alert), (f"{name} ErrCode", code, alert)]
    return (
        _err("SwayMot", 0) + _err("PDU", 12) +
        _err("HVBatt", 24) + _err("DCDC", 36) + _err("VCU_Self", 48)
    )

def dec_3100(d):
    v = _u64(d)
    estop = _bits(v, 10, 2)
    return [
        ("PDU SelfCheck",     _bits(v, 0, 2),   ALERT_NONE),
        ("K1 Relay",          _bits(v, 2, 2),    ALERT_NONE),
        ("K2 Relay",          _bits(v, 4, 2),    ALERT_NONE),
        ("K3 Relay",          _bits(v, 6, 2),    ALERT_NONE),
        ("K4 Relay",          _bits(v, 8, 2),    ALERT_NONE),
        ("EmergStop Sig",     estop,              ALERT_CRIT if estop else ALERT_NONE),
        ("PreCharge Done",    _bits(v, 12, 2),    ALERT_NONE),
        ("HV Interlock",      _bits(v, 14, 1),    ALERT_NONE),
        ("Discharge Sta",     _bits(v, 15, 1),    ALERT_NONE),
    ]

def dec_3101(d):
    v = _u64(d)
    return [
        ("K1 Front Volt", _bits(v, 0, 16),  ALERT_NONE),
        ("K1 Back Volt",  _bits(v, 16, 16), ALERT_NONE),
        ("K3 Back Volt",  _bits(v, 32, 16), ALERT_NONE),
        ("K4 Back Volt",  _bits(v, 48, 16), ALERT_NONE),
    ]

def dec_3102(d):
    v = _u64(d)
    return [
        ("LeftMotor Curr",  _bits(v, 0, 16),  ALERT_NONE),
        ("RightMotor Curr", _bits(v, 16, 16), ALERT_NONE),
        ("RearMotor Curr",  _bits(v, 32, 16), ALERT_NONE),
        ("ExtSupply Curr",  _bits(v, 48, 16), ALERT_NONE),
    ]

def dec_3103(d):
    v = _u64(d)
    return [
        ("Discharge Time s", _bits(v, 0, 32),  ALERT_NONE),
        ("Charge Time s",    _bits(v, 32, 32), ALERT_NONE),
    ]

def dec_3104(d):
    v = _u64(d)
    return [
        ("Total Time s",    _bits(v, 0, 32),  ALERT_NONE),
        ("Battery Power W", _bits(v, 32, 16), ALERT_NONE),
        ("Cycle Count",     _bits(v, 48, 16), ALERT_NONE),
    ]

# ---------------------------------------------------------------------------
# Registry: CAN ID -> (short_name, tab_group, decoder_func)
# Tab groups: 0=NAV_CMD, 1=DRIVE, 2=PUMP, 3=STATUS, 4=ERRORS, 5=PDU/BMS
# ---------------------------------------------------------------------------
CAN_DB = {
    0x1001: ("CMD1 Motor",       0, dec_1001),
    0x1002: ("CMD2 Pump/Acc",    0, dec_1002),
    0x1003: ("Version",          3, dec_1003),
    0x1006: ("Fbk Wheels",       1, dec_1006),
    0x1007: ("Fbk Status",       3, dec_1007),
    0x1008: ("Fbk HV Batt",     5, dec_1008),
    0x1009: ("Fbk CommFlags",    3, dec_1009),
    0x100A: ("DCDC+MotL RPM",    1, dec_100A),
    0x100B: ("MotL Electrical",  1, dec_100B),
    0x100C: ("MotR RPM",         1, dec_100C),
    0x100D: ("MotR+PumpRPM",     2, dec_100D),
    0x100E: ("Pump Electrical",  2, dec_100E),
    0x100F: ("GCut RPM",         1, dec_100F),
    0x1010: ("GCut+Sway",        1, dec_1010),
    0x1011: ("Sway Electrical",  1, dec_1011),
    0x1012: ("Errors 1",         4, dec_1012),
    0x1013: ("Errors 2",         4, dec_1013),
    0x3100: ("PDU Status",       5, dec_3100),
    0x3101: ("PDU Voltages",     5, dec_3101),
    0x3102: ("PDU Currents",     5, dec_3102),
    0x3103: ("BMS Times",        5, dec_3103),
    0x3104: ("BMS Power",        5, dec_3104),
}

TAB_NAMES = ["NAV CMD", "DRIVE", "PUMP", "STATUS", "ERRORS", "PDU/BMS"]

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.msg_count = defaultdict(int)        # CAN_ID -> count
        self.msg_rate = defaultdict(float)        # CAN_ID -> msgs/sec
        self.last_time = defaultdict(float)       # CAN_ID -> last timestamp
        self.last_data = {}                       # CAN_ID -> raw bytes
        self.decoded = {}                         # CAN_ID -> [(label, val, alert)]
        self.alerts = []                          # [(timestamp, text, level)]
        self.total_msgs = 0
        self.start_time = time.time()
        self.paused = False
        self.current_tab = 0
        self.scroll_offset = 0
        self.unknown_ids = defaultdict(int)

    def reset_counters(self):
        with self.lock:
            self.msg_count.clear()
            self.msg_rate.clear()
            self.total_msgs = 0
            self.start_time = time.time()
            self.alerts.clear()
            self.unknown_ids.clear()

    def feed(self, can_id, data, timestamp=None):
        if self.paused:
            return
        ts = timestamp or time.time()
        with self.lock:
            self.total_msgs += 1
            self.msg_count[can_id] += 1
            prev = self.last_time.get(can_id, ts)
            dt = ts - prev
            if dt > 0 and dt < 5:
                alpha = 0.3
                self.msg_rate[can_id] = alpha * (1.0 / dt) + (1 - alpha) * self.msg_rate.get(can_id, 0)
            self.last_time[can_id] = ts
            self.last_data[can_id] = bytes(data[:8])

            if can_id in CAN_DB:
                _, _, decoder = CAN_DB[can_id]
                try:
                    fields = decoder(data)
                    self.decoded[can_id] = fields
                    # Collect alerts
                    for label, val, alert in fields:
                        if alert >= ALERT_CRIT:
                            entry = (datetime.fromtimestamp(ts).strftime("%H:%M:%S"),
                                     f"0x{can_id:04X} {label}={val}", alert)
                            self.alerts.append(entry)
                            if len(self.alerts) > 200:
                                self.alerts = self.alerts[-100:]
                except Exception:
                    pass
            else:
                self.unknown_ids[can_id] += 1


state = State()

# ---------------------------------------------------------------------------
# CAN reader thread
# ---------------------------------------------------------------------------
def can_reader_socketcan(interface):
    bus = python_can.Bus(interface=interface, channel=interface if '/' not in interface else interface,
                         bustype='socketcan')
    while True:
        msg = bus.recv(timeout=1.0)
        if msg is not None:
            d = bytes(msg.data).ljust(8, b'\x00')
            state.feed(msg.arbitration_id, d, msg.timestamp)

def can_reader_file(path):
    """Read candump-style log: (timestamp) interface CANID#DATA"""
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            try:
                parts = line.split()
                ts = None
                id_data = None
                for p in parts:
                    if p.startswith('(') and p.endswith(')'):
                        ts = float(p.strip('()'))
                    if '#' in p:
                        id_data = p
                if id_data:
                    cid_str, data_str = id_data.split('#', 1)
                    cid = int(cid_str, 16)
                    d = bytes.fromhex(data_str).ljust(8, b'\x00')
                    state.feed(cid, d, ts)
                    time.sleep(0.001)
            except Exception:
                continue


def can_reader_saleae(path, replay_speed=1.0):
    """
    Read Saleae Logic 2 CAN analyzer CSV export.
    Expected columns (tab or comma separated):
      name, type, start_time, duration, data

    Rows per frame:
      CAN  identifier_field  <time>  <dur>  0x1007      <- CAN ID (hex)
      CAN  control_field     <time>  <dur>  [optional]
      CAN  data_field        <time>  <dur>  0xFE        <- byte 0
      CAN  data_field        <time>  <dur>  0x7C        <- byte 1
      ...up to 8 data_field rows

    If identifier_field has NO data value, the CAN ID can't be decoded.
    This usually means the analyzer is set to Standard CAN but the bus
    uses Extended CAN (29-bit).  Fix: enable Extended in Saleae settings.
    """
    import csv

    # Detect separator
    with open(path, 'r') as f:
        first_lines = [f.readline() for _ in range(3)]
    sep = '\t' if '\t' in first_lines[0] else ','

    current_id = None
    current_ts = None
    current_bytes = []
    frames_ok = 0
    frames_no_id = 0
    first_ts = None
    last_wall = None

    with open(path, 'r', newline='') as f:
        reader = csv.reader(f, delimiter=sep)
        header = next(reader, None)

        # Find column indices by name (flexible)
        col_map = {}
        if header:
            for i, h in enumerate(header):
                h = h.strip().lower()
                if 'type' in h:
                    col_map['type'] = i
                elif 'start' in h or 'time' in h:
                    col_map.setdefault('time', i)
                elif 'data' in h:
                    col_map['data'] = i
            # If no match, assume: 0=name, 1=type, 2=start_time, 3=duration, 4=data
            col_map.setdefault('type', 1)
            col_map.setdefault('time', 2)
            col_map.setdefault('data', 4)

        for row in reader:
            if len(row) <= col_map.get('type', 1):
                continue
            rtype = row[col_map['type']].strip().lower()
            rtime = float(row[col_map['time']].strip()) if col_map['time'] < len(row) and row[col_map['time']].strip() else 0
            rdata = row[col_map['data']].strip() if col_map['data'] < len(row) else ""

            if rtype == 'identifier_field':
                # Flush previous frame
                if current_id is not None and current_bytes:
                    d = bytes(current_bytes).ljust(8, b'\x00')[:8]
                    state.feed(current_id, d, current_ts)
                    frames_ok += 1
                    # Pace replay to roughly real-time
                    if replay_speed > 0 and first_ts is not None:
                        elapsed_capture = rtime - first_ts
                        elapsed_wall = time.time() - last_wall
                        wait = (elapsed_capture / replay_speed) - elapsed_wall
                        if 0 < wait < 2.0:
                            time.sleep(wait)
                elif current_id is None and current_bytes:
                    frames_no_id += 1

                # Start new frame
                current_bytes = []
                current_ts = rtime
                if first_ts is None:
                    first_ts = rtime
                    last_wall = time.time()

                if rdata:
                    try:
                        current_id = int(rdata, 0)  # handles 0x prefix
                    except ValueError:
                        current_id = None
                else:
                    current_id = None

            elif rtype == 'data_field' and rdata:
                try:
                    current_bytes.append(int(rdata, 0))
                except ValueError:
                    pass

        # Flush last frame
        if current_id is not None and current_bytes:
            d = bytes(current_bytes).ljust(8, b'\x00')[:8]
            state.feed(current_id, d, current_ts)
            frames_ok += 1
        elif current_id is None and current_bytes:
            frames_no_id += 1

    # Report to state for the footer
    if frames_no_id > 0:
        # Use a special "alert" to warn the user
        state.alerts.append((
            datetime.now().strftime("%H:%M:%S"),
            f"Saleae: {frames_no_id} frames with MISSING CAN ID! Enable Extended CAN in analyzer",
            ALERT_CRIT
        ))
    if frames_ok == 0 and frames_no_id > 0:
        state.alerts.append((
            datetime.now().strftime("%H:%M:%S"),
            "ALL frames have no ID - analyzer must be set to Extended CAN (29-bit)",
            ALERT_CRIT
        ))

# ---------------------------------------------------------------------------
# Curses Dashboard
# ---------------------------------------------------------------------------
def draw_dashboard(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    # Colors
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_WHITE, -1)       # normal
    curses.init_pair(2, curses.COLOR_YELLOW, -1)      # warn
    curses.init_pair(3, curses.COLOR_RED, -1)          # critical
    curses.init_pair(4, curses.COLOR_GREEN, -1)        # good
    curses.init_pair(5, curses.COLOR_CYAN, -1)         # header
    curses.init_pair(6, curses.COLOR_BLACK, curses.COLOR_WHITE)  # tab active
    curses.init_pair(7, curses.COLOR_BLACK, curses.COLOR_RED)    # alert banner
    curses.init_pair(8, curses.COLOR_BLACK, curses.COLOR_YELLOW) # warn banner

    C_NORM   = curses.color_pair(1)
    C_WARN   = curses.color_pair(2) | curses.A_BOLD
    C_CRIT   = curses.color_pair(3) | curses.A_BOLD
    C_GOOD   = curses.color_pair(4)
    C_HEAD   = curses.color_pair(5) | curses.A_BOLD
    C_TAB    = curses.color_pair(6)
    C_ABANN  = curses.color_pair(7) | curses.A_BOLD
    C_WBANN  = curses.color_pair(8)
    ALERT_COLORS = {ALERT_NONE: C_NORM, ALERT_WARN: C_WARN, ALERT_CRIT: C_CRIT}

    while True:
        k = stdscr.getch()
        if k == ord('q') or k == ord('Q'):
            return
        elif k == ord('p') or k == ord('P'):
            state.paused = not state.paused
        elif k == ord('r') or k == ord('R'):
            state.reset_counters()
        elif k in (ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6')):
            state.current_tab = k - ord('1')
            state.scroll_offset = 0
        elif k == curses.KEY_UP:
            state.scroll_offset = max(0, state.scroll_offset - 1)
        elif k == curses.KEY_DOWN:
            state.scroll_offset += 1
        elif k == curses.KEY_LEFT:
            state.current_tab = max(0, state.current_tab - 1)
            state.scroll_offset = 0
        elif k == curses.KEY_RIGHT:
            state.current_tab = min(5, state.current_tab + 1)
            state.scroll_offset = 0

        h, w = stdscr.getmaxyx()
        stdscr.erase()

        # --- Title bar ---
        uptime = time.time() - state.start_time
        title = f" TTR AGRO ROVER - CAN DASHBOARD "
        status = f" Msgs:{state.total_msgs}  Up:{int(uptime)}s "
        if state.paused:
            status += " [PAUSED] "
        pad = w - len(title) - len(status)
        stdscr.addnstr(0, 0, title + " " * max(pad, 1) + status, w, C_HEAD | curses.A_REVERSE)

        # --- Active alerts banner ---
        active_crits = []
        with state.lock:
            seen = set()
            for ts, txt, lvl in reversed(state.alerts):
                if lvl >= ALERT_CRIT and txt not in seen:
                    active_crits.append(txt)
                    seen.add(txt)
                if len(active_crits) >= 3:
                    break
        if active_crits:
            banner = " ALERT: " + " | ".join(active_crits)
            stdscr.addnstr(1, 0, banner.ljust(w), w, C_ABANN)
            top = 2
        else:
            top = 1

        # --- Tab bar ---
        tx = 0
        for i, tn in enumerate(TAB_NAMES):
            label = f" {i+1}:{tn} "
            if i == state.current_tab:
                stdscr.addnstr(top, tx, label, min(len(label), w - tx), C_TAB)
            else:
                stdscr.addnstr(top, tx, label, min(len(label), w - tx), C_NORM)
            tx += len(label) + 1
            if tx >= w:
                break
        top += 1

        # --- Content area: messages for current tab ---
        tab = state.current_tab
        with state.lock:
            tab_ids = sorted([cid for cid, (_, grp, _) in CAN_DB.items() if grp == tab])

        row = top
        content_lines = []  # pre-build so we can scroll

        for cid in tab_ids:
            name = CAN_DB[cid][0]
            cnt = state.msg_count.get(cid, 0)
            rate = state.msg_rate.get(cid, 0)
            raw = state.last_data.get(cid)
            fields = state.decoded.get(cid)

            # Has any alert?
            max_alert = ALERT_NONE
            if fields:
                for _, _, a in fields:
                    if a > max_alert:
                        max_alert = a

            raw_hex = " ".join(f"{b:02X}" for b in raw) if raw else "-- -- -- -- -- -- -- --"

            # Header line for this message
            hdr_color = ALERT_COLORS[max_alert]
            content_lines.append((
                f"0x{cid:04X} {name:<18s} cnt:{cnt:<8d} {rate:5.1f}/s  [{raw_hex}]",
                hdr_color | curses.A_BOLD
            ))

            # Field lines
            if fields:
                cols = min(3, max(1, (w - 2) // 30))
                for i in range(0, len(fields), cols):
                    parts = []
                    colors = []
                    for j in range(cols):
                        if i + j < len(fields):
                            label, val, alert = fields[i + j]
                            txt = f"  {label}: {val}"
                            parts.append((txt.ljust(28), ALERT_COLORS[alert]))
                    content_lines.append(parts)

            content_lines.append(("", C_NORM))  # spacer

        # Render with scroll
        visible = h - top - 2  # leave room for footer
        offset = min(state.scroll_offset, max(0, len(content_lines) - visible))

        for idx in range(offset, min(offset + visible, len(content_lines))):
            line = content_lines[idx]
            if row >= h - 2:
                break
            try:
                if isinstance(line, tuple) and len(line) == 2 and isinstance(line[0], str):
                    stdscr.addnstr(row, 0, line[0], w - 1, line[1])
                elif isinstance(line, list):
                    cx = 0
                    for txt, col in line:
                        if cx < w - 1:
                            stdscr.addnstr(row, cx, txt, min(len(txt), w - cx - 1), col)
                        cx += len(txt)
                else:
                    pass
            except curses.error:
                pass
            row += 1

        # --- Footer ---
        unk_str = ""
        if state.unknown_ids:
            unk = ", ".join(f"0x{k:X}:{v}" for k, v in sorted(state.unknown_ids.items())[:5])
            unk_str = f"  Unknown: {unk}"
        footer = f" q:quit p:pause r:reset 1-6:tab arrows:scroll{unk_str}"
        try:
            stdscr.addnstr(h - 1, 0, footer.ljust(w), w, C_HEAD | curses.A_REVERSE)
        except curses.error:
            pass

        stdscr.refresh()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    interface = "can0"
    log_file = None

    args = sys.argv[1:]
    if "--help" in args or "-h" in args:
        print(__doc__)
        sys.exit(0)
    if "--file" in args:
        idx = args.index("--file")
        log_file = args[idx + 1]
    elif args and not args[0].startswith("-"):
        interface = args[0]

    # Start reader thread
    if log_file:
        reader = threading.Thread(target=can_reader_file, args=(log_file,), daemon=True)
    elif HAS_PYTHON_CAN:
        reader = threading.Thread(target=can_reader_socketcan, args=(interface,), daemon=True)
    else:
        print("ERROR: python-can not installed. Install with: pip3 install python-can")
        print("Or use --file <candump.log> to replay a log file.")
        sys.exit(1)

    reader.start()
    curses.wrapper(draw_dashboard)


if __name__ == "__main__":
    main()
