#!/usr/bin/env python3
"""
Analyze a can_dashboard.py CSV log and report alarms.

Reuses the alert thresholds from can_dashboard.py (the decoded column already
contains every field as 'label=value', so we don't need to re-parse the bits).

Usage:
  python tools/analyze_can_log.py tools/can_log_YYYYMMDD_HHMMSS.csv
"""
from __future__ import annotations

import csv
import re
import sys
from collections import defaultdict
from datetime import datetime

# (regex against the decoded column, predicate(value_str)->level, severity_label)
WARN, CRIT = 1, 2
RULES: list[tuple[str, callable, str]] = [
    # CMD1 / Wheels feedback
    (r"NAV_Ctrl_EN=(\d+)",       lambda v: WARN if int(v) == 0 else 0, "NAV_Ctrl_EN"),
    (r"\bBrake=(\d+)",           lambda v: CRIT if int(v) else 0,      "Brake"),
    # CMD2
    (r"MedicSpray Mode=(\w+)",   lambda v: CRIT if v in ("OFF", "IDLE") else 0, "MedicSpray Mode"),
    (r"EmergPowerOff=(\d+)",     lambda v: CRIT if int(v) else 0,      "EmergPowerOff"),
    # Status (1007)
    (r"\bCtrlMode=(\w+)",        lambda v: WARN if v == "REMOTE" else 0, "CtrlMode"),
    (r"MedicSpray Sta=(\w+)",    lambda v: CRIT if v in ("OFF", "IDLE") else 0, "MedicSpray Sta"),
    (r"Collision!=(\d+)",        lambda v: CRIT if int(v) else 0,      "Collision"),
    (r"RC Stop!=(\d+)",          lambda v: CRIT if int(v) else 0,      "RC Stop"),
    (r"ForceStop!=(\d+)",        lambda v: CRIT if int(v) else 0,      "ForceStop"),
    (r"HW E-Stop!=(\d+)",        lambda v: CRIT if int(v) else 0,      "HW E-Stop"),
    (r"HV On Sta=(\d+)",         lambda v: WARN if int(v) == 1 else 0, "HV On Sta"),
    (r"MedicLeft=(\d+)",         lambda v: WARN if int(v) < 20 else 0, "MedicLeft"),
    (r"LV Batt V=([\d.]+)",      lambda v: WARN if float(v) < 22 else 0, "LV Batt V"),
    (r"HV Charge/Disch=(\d+)",   lambda v: WARN if int(v) else 0,      "HV Charge/Disch"),
    (r"DCDC Temp C=(-?\d+)",     lambda v: WARN if int(v) > 70 else 0, "DCDC Temp"),
    # HV Battery (1008)
    (r"\bSOC %=(\d+)",           lambda v: CRIT if int(v) < 10 else (WARN if int(v) < 20 else 0), "SOC %"),
    (r"\bSOH %=(\d+)",           lambda v: WARN if int(v) < 50 else 0, "SOH %"),
    (r"\bHV Temp C=(-?\d+)",     lambda v: CRIT if int(v) > 55 else (WARN if int(v) > 45 else 0), "HV Temp"),
    # Comm flags (1009) — 1 means LOST
    (r"ComWalkMotDrvL=(\d+)",    lambda v: CRIT if int(v) else 0, "ComWalkMotDrvL"),
    (r"ComWalkMotDrvR=(\d+)",    lambda v: CRIT if int(v) else 0, "ComWalkMotDrvR"),
    (r"ComMedicPumpDrv=(\d+)",   lambda v: CRIT if int(v) else 0, "ComMedicPumpDrv"),
    (r"ComSwayMotDrv=(\d+)",     lambda v: CRIT if int(v) else 0, "ComSwayMotDrv"),
    (r"ComGrassCutDrv=(\d+)",    lambda v: CRIT if int(v) else 0, "ComGrassCutDrv"),
    (r"ComPushMotDrv=(\d+)",     lambda v: CRIT if int(v) else 0, "ComPushMotDrv"),
    (r"ComNav=(\d+)",            lambda v: CRIT if int(v) else 0, "ComNav"),
    (r"ComRC=(\d+)",             lambda v: CRIT if int(v) else 0, "ComRC"),
    (r"ComPDU=(\d+)",            lambda v: CRIT if int(v) else 0, "ComPDU"),
    (r"ComDCDC=(\d+)",           lambda v: CRIT if int(v) else 0, "ComDCDC"),
    (r"ComHVBMS=(\d+)",          lambda v: CRIT if int(v) else 0, "ComHVBMS"),
    # Motor temps (100B / 100D / 100E)
    (r"MotL Drv Temp C=(-?\d+)", lambda v: CRIT if int(v) > 80 else (WARN if int(v) > 65 else 0), "MotL Drv Temp"),
    (r"MotL Temp C=(-?\d+)",     lambda v: CRIT if int(v) > 100 else (WARN if int(v) > 80 else 0), "MotL Temp"),
    (r"MotR Drv Temp C=(-?\d+)", lambda v: CRIT if int(v) > 80 else (WARN if int(v) > 65 else 0), "MotR Drv Temp"),
    (r"MotR Temp C=(-?\d+)",     lambda v: CRIT if int(v) > 100 else (WARN if int(v) > 80 else 0), "MotR Temp"),
    (r"PUMP Drv Temp C=(-?\d+)", lambda v: CRIT if int(v) > 80 else (WARN if int(v) > 65 else 0), "PUMP Drv Temp"),
    (r"PUMP Mot Temp C=(-?\d+)", lambda v: CRIT if int(v) > 100 else (WARN if int(v) > 80 else 0), "PUMP Mot Temp"),
    (r"GCut Drv Temp C=(-?\d+)", lambda v: WARN if int(v) > 65 else 0, "GCut Drv Temp"),
    (r"GCut Mot Temp C=(-?\d+)", lambda v: WARN if int(v) > 80 else 0, "GCut Mot Temp"),
    (r"Sway Drv Temp C=(-?\d+)", lambda v: WARN if int(v) > 65 else 0, "Sway Drv Temp"),
    (r"Sway Mot Temp C=(-?\d+)", lambda v: WARN if int(v) > 80 else 0, "Sway Mot Temp"),
    # PUMP feedback mismatch (100D)
    # We need both TgtRPM and FbkRPM on the same row — handled separately below.
    # PDU emergency stop (3100)
    (r"EmergStop Sig=(\d+)",     lambda v: CRIT if int(v) else 0, "EmergStop Sig"),
    # Error levels (1012 / 1013)
    (r"WalkMotL ErrLvl=(\d+)",   lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "WalkMotL ErrLvl"),
    (r"WalkMotR ErrLvl=(\d+)",   lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "WalkMotR ErrLvl"),
    (r"MedicPump ErrLvl=(\d+)",  lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "MedicPump ErrLvl"),
    (r"GrassCut ErrLvl=(\d+)",   lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "GrassCut ErrLvl"),
    (r"PushMot ErrLvl=(\d+)",    lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "PushMot ErrLvl"),
    (r"SwayMot ErrLvl=(\d+)",    lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "SwayMot ErrLvl"),
    (r"\bPDU ErrLvl=(\d+)",      lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "PDU ErrLvl"),
    (r"HVBatt ErrLvl=(\d+)",     lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "HVBatt ErrLvl"),
    (r"\bDCDC ErrLvl=(\d+)",     lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "DCDC ErrLvl"),
    (r"VCU_Self ErrLvl=(\d+)",   lambda v: CRIT if int(v) > 2 else (WARN if int(v) > 0 else 0), "VCU_Self ErrLvl"),
]

LEVEL_NAME = {WARN: "WARN", CRIT: "CRIT"}


def fmt_ts(epoch: float) -> str:
    return datetime.fromtimestamp(epoch).strftime("%H:%M:%S.%f")[:-3]


def main(path: str) -> int:
    # alarm key -> dict with first/last/peak/count/level
    alarms: dict[str, dict] = defaultdict(lambda: {
        "first": None, "last": None, "count": 0,
        "min_val": None, "max_val": None, "level": 0,
    })
    pump_mismatch_count = 0
    pump_mismatch_first = pump_mismatch_last = None

    t_first = t_last = None
    n_rows = 0

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            n_rows += 1
            try:
                ts = float(row["timestamp"])
            except (ValueError, KeyError):
                continue
            decoded = row.get("decoded", "") or ""
            if t_first is None:
                t_first = ts
            t_last = ts

            # PUMP TgtRPM/FbkRPM mismatch (100D) — special-case, both fields on same row
            m_tgt = re.search(r"PUMP TgtRPM=(-?\d+)", decoded)
            m_fbk = re.search(r"PUMP FbkRPM=(-?\d+)", decoded)
            if m_tgt and m_fbk:
                tgt = int(m_tgt.group(1)); fbk = int(m_fbk.group(1))
                if tgt != 0 and fbk == 0:
                    pump_mismatch_count += 1
                    if pump_mismatch_first is None:
                        pump_mismatch_first = ts
                    pump_mismatch_last = ts

            for pattern, predicate, label in RULES:
                m = re.search(pattern, decoded)
                if not m:
                    continue
                val_str = m.group(1)
                try:
                    level = predicate(val_str)
                except Exception:
                    continue
                if not level:
                    continue
                key = f"{label} ({LEVEL_NAME[level]})"
                a = alarms[key]
                if a["first"] is None:
                    a["first"] = ts
                a["last"] = ts
                a["count"] += 1
                a["level"] = max(a["level"], level)
                # Track numeric range when possible
                try:
                    num = float(val_str)
                    a["min_val"] = num if a["min_val"] is None else min(a["min_val"], num)
                    a["max_val"] = num if a["max_val"] is None else max(a["max_val"], num)
                except ValueError:
                    a["min_val"] = a["max_val"] = val_str

    # Report
    duration = (t_last - t_first) if (t_first and t_last) else 0.0
    print(f"\nFile      : {path}")
    print(f"Rows      : {n_rows}")
    if t_first:
        print(f"Span      : {fmt_ts(t_first)} -> {fmt_ts(t_last)}  ({duration:.1f} s)")
    print()

    if not alarms and not pump_mismatch_count:
        print("No alarms triggered. Log is clean.")
        return 0

    # Sort: CRIT first, then by count desc
    items = sorted(alarms.items(), key=lambda kv: (-kv[1]["level"], -kv[1]["count"]))
    print(f"{'Alarm':<28}  {'Count':>6}  {'First':>13}  {'Last':>13}  {'Range':>20}")
    print("-" * 90)
    for key, a in items:
        rng = ""
        if a["min_val"] is not None:
            if a["min_val"] == a["max_val"]:
                rng = str(a["min_val"])
            else:
                rng = f"{a['min_val']} .. {a['max_val']}"
        print(f"{key:<28}  {a['count']:>6}  {fmt_ts(a['first']):>13}  {fmt_ts(a['last']):>13}  {rng:>20}")

    if pump_mismatch_count:
        print(f"{'PUMP Tgt!=0 Fbk=0 (WARN)':<28}  {pump_mismatch_count:>6}  "
              f"{fmt_ts(pump_mismatch_first):>13}  {fmt_ts(pump_mismatch_last):>13}")

    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python tools/analyze_can_log.py <can_log_*.csv>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
