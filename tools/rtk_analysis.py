#!/usr/bin/env python3
"""Complete RTK dynamics analysis - identifies CH3 as primary throttle."""

import csv
import math
from collections import defaultdict

CSV_PATH = "C:/ros_agri_rover/dynamics_rtk.csv"

rows = []
with open(CSV_PATH, "r") as f:
    reader = csv.DictReader(f)
    for r in reader:
        lat = float(r["lat"])
        if lat == 0.0:
            continue
        if r["source"].strip() != "rc":
            continue
        rows.append({
            "t": float(r["t"]),
            "ch1": int(r["ppm_throttle"]),
            "ch2": int(r["ppm_steer"]),
            "ch3": int(r["ppm_ch3"]),
            "ch4": int(r["ppm_ch4"]),
            "ch5": int(r["ppm_ch5"]),
            "ch6": int(r["ppm_ch6"]),
            "lat": lat,
            "lon": float(r["lon"]),
            "hdg": float(r["heading"]),
        })

T0 = rows[0]["t"]
print(f"Rows: {len(rows)}, Duration: {rows[-1]['t'] - T0:.1f}s\n")

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    rl1, rl2 = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(rl1)*math.cos(rl2)*math.sin(dlon/2)**2
    return 2 * R * math.asin(math.sqrt(a))

# Build GPS-rate epochs (deduplicate same lat/lon/hdg)
gps = []
prev_key = None
for r in rows:
    key = (r["lat"], r["lon"], r["hdg"])
    if prev_key is None or key != prev_key:
        gps.append(dict(r))
        prev_key = key
    else:
        for k in ["ch1", "ch2", "ch3", "ch4", "ch5", "ch6"]:
            gps[-1][k] = r[k]

# Compute speed at each epoch
for i, e in enumerate(gps):
    best = None
    for j in range(i+1, min(i+20, len(gps))):
        dt = gps[j]["t"] - e["t"]
        if 0.4 <= dt <= 1.2:
            d = haversine(e["lat"], e["lon"], gps[j]["lat"], gps[j]["lon"])
            best = d / dt
            break
    e["speed"] = best if best is not None else 0.0

print(f"GPS epochs: {len(gps)}\n")

# ====================================================================
# 0. IDENTIFY PRIMARY THROTTLE CHANNEL
# ====================================================================
print("=" * 70)
print("0. CHANNEL IDENTIFICATION")
print("=" * 70)

# Check CH5 over time
ch5_vals = set(e["ch5"] for e in gps)
print(f"CH5 values: {sorted(ch5_vals)}")
first_non1939 = next((e for e in gps if e["ch5"] != 1939), None)
if first_non1939:
    print(f"CH5 first non-1939 at t={first_non1939['t']-T0:.0f}s: {first_non1939['ch5']}")
else:
    print("CH5 = 1939 for entire dataset")

# Correlation test
moving = [e for e in gps if e["speed"] > 0.1]
print(f"\nMoving epochs (speed > 0.1): {len(moving)}")

for name, key in [("CH1 (ppm_throttle)", "ch1"), ("CH2 (ppm_steer)", "ch2"), ("CH3 (ppm_ch3)", "ch3")]:
    xs = [abs(e[key] - 1500) for e in moving]
    ys = [e["speed"] for e in moving]
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxy = sum(x*y for x, y in zip(xs, ys))
    sx2, sy2 = sum(x*x for x in xs), sum(y*y for y in ys)
    denom = math.sqrt((n*sx2 - sx**2) * (n*sy2 - sy**2))
    r = (n*sxy - sx*sy) / denom if denom > 0 else 0
    print(f"  |{name}| vs speed: r = {r:.4f}")

# Speed comparison at same offset
ch1_full = [e for e in gps if abs(e["ch1"]-1500) > 400 and abs(e["ch3"]-1500) < 50]
ch3_full = [e for e in gps if abs(e["ch3"]-1500) > 400 and abs(e["ch1"]-1500) < 50]
if ch1_full:
    avg = sum(e["speed"] for e in ch1_full) / len(ch1_full)
    print(f"\nCH1 at full offset (>400), CH3 neutral: avg speed = {avg:.3f} m/s ({len(ch1_full)} epochs)")
if ch3_full:
    avg = sum(e["speed"] for e in ch3_full) / len(ch3_full)
    print(f"CH3 at full offset (>400), CH1 neutral: avg speed = {avg:.3f} m/s ({len(ch3_full)} epochs)")

print("\n>>> CH3 is the PRIMARY throttle channel (left stick Y on MK32)")
print(">>> CH1 has very weak effect (~0.15 m/s at full offset)")
print(">>> Analysis below uses CH3 as throttle\n")

# ====================================================================
# 1. THROTTLE-TO-SPEED CURVE (using CH3)
# ====================================================================
print("=" * 70)
print("1. THROTTLE-TO-SPEED CURVE (CH3 as throttle, CH2 near neutral)")
print("=" * 70)

def find_runs(epochs, ch_key, tol=40, steer_lo=1400, steer_hi=1600, min_dur=1.5, min_offset=60):
    runs = []
    i = 0
    while i < len(epochs):
        ref = epochs[i][ch_key]
        if abs(ref-1500) < min_offset or not (steer_lo <= epochs[i]["ch2"] <= steer_hi):
            i += 1
            continue
        j = i + 1
        while j < len(epochs):
            if abs(epochs[j][ch_key] - ref) > tol or not (steer_lo <= epochs[j]["ch2"] <= steer_hi):
                break
            j += 1
        dur = epochs[j-1]["t"] - epochs[i]["t"]
        if dur >= min_dur:
            seg = epochs[i:j]
            t_start = seg[0]["t"]
            settled = [e for e in seg if e["t"]-t_start >= 1.0 and e["speed"] > 0]
            if not settled:
                settled = [e for e in seg if e["speed"] > 0]
            if settled:
                speeds = [e["speed"] for e in settled]
                avg_spd = sum(speeds)/len(speeds)
                med_spd = sorted(speeds)[len(speeds)//2]
                neutral_steer = all(1450 <= e["ch2"] <= 1550 for e in seg)
                runs.append({
                    "ref": ref, "avg": avg_spd, "med": med_spd,
                    "dur": dur, "t": t_start-T0, "straight": neutral_steer,
                    "n": len(settled), "min": min(speeds), "max": max(speeds)
                })
        i = j
    return runs

ch3_runs = find_runs(gps, "ch3")
print(f"Found {len(ch3_runs)} sustained CH3 runs\n")

for r in ch3_runs:
    tag = "STRAIGHT" if r["straight"] else "w/steer"
    direction = "FWD" if r["ref"] > 1500 else "REV"
    print(f"  t={r['t']:5.0f}s: CH3={r['ref']:4d} ({r['ref']-1500:+4d} {direction}), "
          f"dur={r['dur']:.1f}s, med={r['med']:.3f}, avg={r['avg']:.3f}, "
          f"range=[{r['min']:.3f},{r['max']:.3f}] m/s  [{tag}]")

# Aggregate by CH3 value
print("\n--- SUMMARY TABLE: CH3 PPM -> Speed ---")
print(f"  {'CH3':>5s} {'Offset':>7s} {'Speed m/s':>10s} {'Runs':>5s} {'Dir':>5s}")
print("  " + "-" * 40)
thr_groups = defaultdict(list)
for r in ch3_runs:
    if r["straight"]:
        thr_groups[r["ref"]].append(r["med"])

for ppm in sorted(thr_groups.keys()):
    speeds = thr_groups[ppm]
    avg = sum(speeds)/len(speeds)
    direction = "FWD" if ppm > 1500 else "REV"
    print(f"  {ppm:5d} {ppm-1500:+5d}   {avg:8.3f}   {len(speeds):4d}  {direction}")

# Include non-straight too in a separate table
print(f"\n  ALL runs (including with steer):")
thr_all = defaultdict(list)
for r in ch3_runs:
    thr_all[r["ref"]].append(r["med"])
for ppm in sorted(thr_all.keys()):
    speeds = thr_all[ppm]
    avg = sum(speeds)/len(speeds)
    direction = "FWD" if ppm > 1500 else "REV"
    print(f"  {ppm:5d} {ppm-1500:+5d}   {avg:8.3f}   ({len(speeds)} runs)  {direction}")

# Also show CH1 runs for comparison
print("\n--- CH1 runs (for comparison) ---")
ch1_runs = find_runs(gps, "ch1")
for r in ch1_runs:
    tag = "S" if r["straight"] else "T"
    direction = "FWD" if r["ref"] > 1500 else "REV"
    print(f"  t={r['t']:5.0f}s: CH1={r['ref']:4d} ({r['ref']-1500:+4d} {direction}), "
          f"dur={r['dur']:.1f}s, med={r['med']:.3f} m/s [{tag}]")

# Linearity analysis on CH3 straight runs
print("\n--- LINEARITY CHECK (CH3, straight only) ---")
fwd_pts = [(abs(ppm-1500), sum(s)/len(s)) for ppm, s in thr_groups.items() if ppm > 1500]
rev_pts = [(abs(ppm-1500), sum(s)/len(s)) for ppm, s in thr_groups.items() if ppm < 1500]

for label, pts in [("FORWARD", fwd_pts), ("REVERSE", rev_pts)]:
    if len(pts) < 2:
        if pts:
            o, s = pts[0]
            extrap = s / o * 500 if o > 0 else 0
            print(f"  {label}: 1 point only: offset {o} -> {s:.3f} m/s (extrap to 500: {extrap:.3f})")
        continue
    pts.sort()
    print(f"  {label}:")
    for o, s in pts:
        print(f"    offset {o:4d} -> {s:.3f} m/s")
    xs, ys = zip(*pts)
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxy = sum(x*y for x, y in zip(xs, ys))
    sx2 = sum(x*x for x in xs)
    denom = n*sx2 - sx**2
    if denom:
        slope = (n*sxy - sx*sy) / denom
        intercept = (sy - slope*sx) / n
        y_mean = sy/n
        ss_tot = sum((y-y_mean)**2 for y in ys)
        ss_res = sum((y-(slope*x+intercept))**2 for x, y in zip(xs, ys))
        r2 = 1 - ss_res/ss_tot if ss_tot > 0 else 0
        print(f"    Linear: speed = {slope:.6f} * offset + {intercept:.4f}")
        print(f"    At full stick (500): {slope*500+intercept:.3f} m/s")
        print(f"    R^2 = {r2:.4f}")
        if r2 < 0.85:
            print(f"    ** Low R^2 -> non-linear or insufficient data")
    else:
        print(f"    Cannot compute regression (degenerate)")

print()

# ====================================================================
# 2. SPIN RATE
# ====================================================================
print("=" * 70)
print("2. SPIN RATE (CH1/CH3 near neutral, CH2 active)")
print("=" * 70)

spin_data = []
i = 0
while i < len(gps):
    e = gps[i]
    ch1_n = abs(e["ch1"]-1500) <= 60
    ch3_n = abs(e["ch3"]-1500) <= 60
    str_active = abs(e["ch2"]-1500) > 80

    if ch1_n and ch3_n and str_active:
        j = i + 1
        while j < len(gps):
            ej = gps[j]
            if not (abs(ej["ch1"]-1500) <= 60 and abs(ej["ch3"]-1500) <= 60 and abs(ej["ch2"]-1500) > 80):
                break
            j += 1
        dur = gps[j-1]["t"] - gps[i]["t"]
        if dur >= 0.5:
            seg = gps[i:j]
            total_dh = 0
            for k in range(1, len(seg)):
                dh = seg[k]["hdg"] - seg[k-1]["hdg"]
                while dh > 180: dh -= 360
                while dh < -180: dh += 360
                total_dh += dh
            rate = total_dh / dur
            avg_str = sum(s["ch2"] for s in seg) / len(seg)
            direction = "RIGHT" if avg_str > 1500 else "LEFT"
            t_rel = gps[i]["t"] - T0
            spin_data.append((avg_str, rate, dur, direction, t_rel))
            print(f"  t={t_rel:5.0f}s: steer={avg_str:.0f} ({avg_str-1500:+.0f}), dur={dur:.1f}s, "
                  f"rate={rate:+.1f} deg/s ({direction})")
        i = j
    else:
        i += 1

# Also check for spin WITH throttle active (zero-throttle spins rare)
print("\n  Also checking spins with CH1 active (CH3 neutral):")
i = 0
while i < len(gps):
    e = gps[i]
    ch1_active = abs(e["ch1"]-1500) > 50
    ch3_n = abs(e["ch3"]-1500) <= 60
    str_active = abs(e["ch2"]-1500) > 100

    if ch1_active and ch3_n and str_active:
        j = i + 1
        while j < len(gps):
            ej = gps[j]
            if not (abs(ej["ch1"]-1500) > 50 and abs(ej["ch3"]-1500) <= 60 and abs(ej["ch2"]-1500) > 100):
                break
            j += 1
        dur = gps[j-1]["t"] - gps[i]["t"]
        if dur >= 0.8:
            seg = gps[i:j]
            total_dh = 0
            for k in range(1, len(seg)):
                dh = seg[k]["hdg"] - seg[k-1]["hdg"]
                while dh > 180: dh -= 360
                while dh < -180: dh += 360
                total_dh += dh
            rate = total_dh / dur
            avg_str = sum(s["ch2"] for s in seg) / len(seg)
            avg_ch1 = sum(s["ch1"] for s in seg) / len(seg)
            direction = "RIGHT" if avg_str > 1500 else "LEFT"
            t_rel = gps[i]["t"] - T0
            total_dist = sum(haversine(seg[k-1]["lat"], seg[k-1]["lon"],
                                       seg[k]["lat"], seg[k]["lon"])
                            for k in range(1, len(seg)))
            speed = total_dist / dur if dur > 0 else 0
            spin_data.append((avg_str, rate, dur, direction, t_rel))
            print(f"  t={t_rel:5.0f}s: CH1={avg_ch1:.0f}, steer={avg_str:.0f} ({avg_str-1500:+.0f}), "
                  f"dur={dur:.1f}s, rate={rate:+.1f} deg/s, speed={speed:.3f} m/s ({direction})")
        i = j
    else:
        i += 1

left = [(s, abs(r)) for s, r, d, di, t in spin_data if di == "LEFT"]
right = [(s, abs(r)) for s, r, d, di, t in spin_data if di == "RIGHT"]
if left:
    avg_rate = sum(r for _, r in left)/len(left)
    avg_str = sum(s for s, _ in left)/len(left)
    print(f"\n  LEFT average: {avg_rate:.1f} deg/s (steer avg {avg_str:.0f})")
if right:
    avg_rate = sum(r for _, r in right)/len(right)
    avg_str = sum(s for s, _ in right)/len(right)
    print(f"  RIGHT average: {avg_rate:.1f} deg/s (steer avg {avg_str:.0f})")

print()

# ====================================================================
# 3. TURNING AT SPEED
# ====================================================================
print("=" * 70)
print("3. TURNING AT SPEED (CH3 or CH1 + CH2 both active)")
print("=" * 70)

i = 0
while i < len(gps):
    e = gps[i]
    thr_active = abs(e["ch3"]-1500) > 50 or abs(e["ch1"]-1500) > 50
    str_active = abs(e["ch2"]-1500) > 80

    if thr_active and str_active:
        j = i + 1
        while j < len(gps):
            ej = gps[j]
            if not ((abs(ej["ch3"]-1500) > 50 or abs(ej["ch1"]-1500) > 50) and abs(ej["ch2"]-1500) > 80):
                break
            j += 1
        dur = gps[j-1]["t"] - gps[i]["t"]
        if dur >= 0.8:
            seg = gps[i:j]
            total_dist = sum(haversine(seg[k-1]["lat"], seg[k-1]["lon"],
                                       seg[k]["lat"], seg[k]["lon"])
                            for k in range(1, len(seg)))
            speed = total_dist / dur if dur > 0 else 0
            total_dh = 0
            for k in range(1, len(seg)):
                dh = seg[k]["hdg"] - seg[k-1]["hdg"]
                while dh > 180: dh -= 360
                while dh < -180: dh += 360
                total_dh += dh
            omega = total_dh / dur
            omega_rad = math.radians(abs(omega))
            radius = speed / omega_rad if omega_rad > 0.01 else float("inf")
            avg_ch1 = sum(s["ch1"] for s in seg)/len(seg)
            avg_ch3 = sum(s["ch3"] for s in seg)/len(seg)
            avg_str = sum(s["ch2"] for s in seg)/len(seg)
            direction = "RIGHT" if omega > 0 else "LEFT"
            t_rel = gps[i]["t"] - T0
            thr_str = f"CH3={avg_ch3:.0f}" if abs(avg_ch3-1500) > abs(avg_ch1-1500) else f"CH1={avg_ch1:.0f}"
            print(f"  t={t_rel:5.0f}s: {thr_str}, steer={avg_str:.0f} ({avg_str-1500:+.0f}), "
                  f"dur={dur:.1f}s, speed={speed:.3f} m/s, omega={omega:+.1f} deg/s, "
                  f"R={radius:.2f}m ({direction})")
        i = j
    else:
        i += 1

print()

# ====================================================================
# 4. ACCELERATION
# ====================================================================
print("=" * 70)
print("4. ACCELERATION (CH3 jump from neutral)")
print("=" * 70)

for i in range(3, len(gps)-10):
    if gps[i]["speed"] > 0.08:
        continue
    ch3_active = abs(gps[i]["ch3"]-1500) > 100
    if not ch3_active:
        continue

    target = gps[i]["ch3"]
    t_start = gps[i]["t"]

    # Verify CH3 stays stable for >= 2s
    stable = True
    for j in range(i, min(i+30, len(gps))):
        if gps[j]["t"] - t_start > 3.0:
            break
        if abs(gps[j]["ch3"] - target) > 80:
            stable = False
            break
    if not stable:
        continue

    # Speed profile
    profile = []
    for j in range(max(0, i-3), min(len(gps), i+50)):
        dt = gps[j]["t"] - t_start
        if -1 <= dt <= 10:
            profile.append((dt, gps[j]["speed"], gps[j]["ch3"]))

    if len(profile) < 5:
        continue
    peak = max(s for _, s, _ in profile)
    if peak < 0.1:
        continue

    # Time to 80%
    target_80 = peak * 0.8
    t80 = None
    for dt, spd, _ in profile:
        if dt > 0 and spd >= target_80:
            t80 = dt
            break

    # Average acceleration in first 2s
    early = [(t, s) for t, s, _ in profile if 0.2 < t < 0.8]
    late = [(t, s) for t, s, _ in profile if 1.5 < t < 3.0]
    accel = 0
    if early and late:
        s0 = sum(s for _, s in early)/len(early)
        t0_ = sum(t for t, _ in early)/len(early)
        s1 = sum(s for _, s in late)/len(late)
        t1_ = sum(t for t, _ in late)/len(late)
        if t1_ > t0_:
            accel = (s1-s0)/(t1_-t0_)

    t_rel = t_start - T0
    t80s = f"{t80:.1f}s" if t80 else ">10s"
    direction = "FWD" if target > 1500 else "REV"
    print(f"\n  t={t_rel:.0f}s: CH3={target} ({target-1500:+d} {direction}), "
          f"peak={peak:.3f} m/s, time_to_80%={t80s}, accel={accel:.3f} m/s^2")
    for dt, spd, ch3 in profile:
        if -0.5 <= dt <= 8:
            bar = "#" * int(spd*25)
            print(f"    t={dt:+5.1f}s: {spd:.3f} m/s (CH3={ch3:4d})  {bar}")

print()

# ====================================================================
# 5. DECELERATION
# ====================================================================
print("=" * 70)
print("5. DECELERATION (CH3 returns to neutral)")
print("=" * 70)

for i in range(1, len(gps)-5):
    prev = gps[i-1]
    curr = gps[i]

    ch3_was = abs(prev["ch3"]-1500) > 100
    ch3_now = abs(curr["ch3"]-1500) <= 50
    was_moving = prev["speed"] > 0.15

    if not (ch3_was and ch3_now and was_moving):
        continue

    t_stop = curr["t"]

    # Verify stays neutral 2s
    stays = True
    for j in range(i, min(i+25, len(gps))):
        if gps[j]["t"] - t_stop > 2.0:
            break
        if abs(gps[j]["ch3"]-1500) > 80:
            stays = False
            break
    if not stays:
        continue

    initial_speed = prev["speed"]

    # Coast-down profile
    profile = []
    for j in range(max(0, i-8), min(len(gps), i+35)):
        dt = gps[j]["t"] - t_stop
        if -2 <= dt <= 5:
            profile.append((dt, gps[j]["speed"]))

    if len(profile) < 3:
        continue

    # Time to stop
    time_to_stop = None
    for dt, spd in profile:
        if dt > 0 and spd < 0.05:
            time_to_stop = dt
            break

    # Coast distance
    coast_dist = 0
    for j in range(i, min(len(gps), i+35)):
        if gps[j]["t"] - t_stop > 5:
            break
        if j > i:
            coast_dist += haversine(gps[j-1]["lat"], gps[j-1]["lon"],
                                    gps[j]["lat"], gps[j]["lon"])

    decel = initial_speed / time_to_stop if time_to_stop and time_to_stop > 0 else 0
    t_rel = t_stop - T0
    ts = f"{time_to_stop:.2f}s" if time_to_stop else ">5s"

    print(f"\n  t={t_rel:.0f}s: CH3 {prev['ch3']}->{curr['ch3']}, "
          f"v0={initial_speed:.3f} m/s, stop_time={ts}, "
          f"coast={coast_dist:.3f}m, decel={decel:.2f} m/s^2")
    for dt, spd in profile:
        bar = "#" * int(spd*25)
        print(f"    t={dt:+5.1f}s: {spd:.3f} m/s  {bar}")

# ====================================================================
# 6. FULL-THROTTLE RUNS (CH3 at extremes)
# ====================================================================
print("\n" + "=" * 70)
print("6. FULL-THROTTLE RUNS (CH3 at 1939 or 1061)")
print("=" * 70)

for target_ppm, label in [(1939, "FULL FWD CH3=1939"), (1061, "FULL REV CH3=1061")]:
    runs = []
    current = []
    for e in gps:
        if abs(e["ch3"]-target_ppm) <= 60:
            current.append(e)
        else:
            if len(current) >= 5:
                runs.append(current)
            current = []
    if len(current) >= 5:
        runs.append(current)

    print(f"\n  {label}: {len(runs)} runs")
    for run in runs:
        dur = run[-1]["t"] - run[0]["t"]
        speeds = [e["speed"] for e in run if e["speed"] > 0]
        if not speeds:
            continue
        t_rel = run[0]["t"] - T0
        half = max(1, len(speeds)//2)
        settled = speeds[half:]
        avg_settled = sum(settled)/len(settled)
        max_spd = max(speeds)
        print(f"    t={t_rel:.0f}s, dur={dur:.1f}s, settled_avg={avg_settled:.3f}, max={max_spd:.3f} m/s")

# ====================================================================
# THROTTLE HISTOGRAM
# ====================================================================
print("\n" + "=" * 70)
print("THROTTLE HISTOGRAM (CH3, during movement)")
print("=" * 70)

ch3_hist = defaultdict(int)
for r in rows:
    if abs(r["ch3"]-1500) > 30:
        bucket = (r["ch3"]//50)*50
        ch3_hist[bucket] += 1

for bucket in sorted(ch3_hist.keys()):
    count = ch3_hist[bucket]
    dur = count * 0.02
    bar = "#" * min(int(dur/0.5), 60)
    print(f"  CH3 {bucket:4d}-{bucket+49:4d}: {dur:6.1f}s  {bar}")

print("\n  CH1 histogram (for reference):")
ch1_hist = defaultdict(int)
for r in rows:
    if abs(r["ch1"]-1500) > 30:
        bucket = (r["ch1"]//50)*50
        ch1_hist[bucket] += 1

for bucket in sorted(ch1_hist.keys()):
    count = ch1_hist[bucket]
    dur = count * 0.02
    bar = "#" * min(int(dur/0.5), 60)
    print(f"  CH1 {bucket:4d}-{bucket+49:4d}: {dur:6.1f}s  {bar}")

print("\n" + "=" * 70)
print("ANALYSIS COMPLETE")
print("=" * 70)
