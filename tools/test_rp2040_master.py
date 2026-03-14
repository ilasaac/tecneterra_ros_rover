#!/usr/bin/env python3
"""
tools/test_rp2040_master.py
Test tool for RP2040 master firmware over USB CDC serial.

Runs on the Jetson (or any machine with the RP2040 connected via USB).
Sends heartbeats automatically, parses all RP2040 output, and displays
a live channel + mode dashboard. Also allows injecting test commands.

Usage:
  python3 tools/test_rp2040_master.py                  # auto-detect port
  python3 tools/test_rp2040_master.py --port /dev/ttyACM0
  python3 tools/test_rp2040_master.py --no-hb          # disable heartbeat (test HB timeout)

Controls (type in terminal, press Enter):
  hb       Toggle automatic heartbeat on/off
  j        Send neutral autonomous command (all 1500) — requires SWB up
  j+       Send full-throttle autonomous command (throttle=1700, others 1500)
  raw <x>  Send raw string x to RP2040  e.g.  raw <HB:1>
  q        Quit
"""

import argparse
import glob
import os
import sys
import threading
import time
import select

try:
    import serial
except ImportError:
    sys.exit("pip3 install pyserial")

# ── ANSI colours ─────────────────────────────────────────────────────────────

R  = "\033[31m"   # red
G  = "\033[32m"   # green
Y  = "\033[33m"   # yellow
B  = "\033[34m"   # blue
M  = "\033[35m"   # magenta
C  = "\033[36m"   # cyan
W  = "\033[37m"   # white
BLD= "\033[1m"
RST= "\033[0m"

MODE_COLOR = {
    "MANUAL":       G,
    "AUTONOMOUS":   C,
    "RELAY":        B,
    "EMERGENCY":    R,
    "AUTO-NO-HB":   Y,
    "AUTO-TIMEOUT": Y,
}

# ── State ────────────────────────────────────────────────────────────────────

state = {
    "channels":   [1500] * 16,
    "mode":       "UNKNOWN",
    "sbus_ok":    False,
    "sx_ok":      None,      # None = not yet seen, True/False = detected
    "hb_tx":      True,
    "hb_seq":     0,
    "hb_rx_seq":  None,
    "frame_count": 0,
    "last_frame":  time.time(),
    "raw_log":    [],
    "running":    True,
}
lock = threading.Lock()

# ── Port detection ────────────────────────────────────────────────────────────

def find_port():
    candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not candidates:
        sys.exit(f"{R}No USB serial ports found. Is the RP2040 connected?{RST}")
    if len(candidates) == 1:
        return candidates[0]
    print(f"Multiple ports found: {candidates}")
    print("Specify with --port. Using first:", candidates[0])
    return candidates[0]

# ── Bar graph helper ──────────────────────────────────────────────────────────

def bar(value, lo=1000, hi=2000, width=20):
    frac = (value - lo) / (hi - lo)
    frac = max(0.0, min(1.0, frac))
    filled = round(frac * width)
    return f"[{'█' * filled}{'░' * (width - filled)}]"

# ── Serial reader thread ──────────────────────────────────────────────────────

def reader(ser):
    buf = ""
    while state["running"]:
        try:
            raw = ser.read(ser.in_waiting or 1).decode("ascii", errors="ignore")
        except serial.SerialException:
            print(f"\n{R}Serial disconnected.{RST}")
            state["running"] = False
            break

        buf += raw
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            process_line(line)

def process_line(line):
    with lock:
        # Keep last 6 raw lines for display
        state["raw_log"].append(line)
        if len(state["raw_log"]) > 6:
            state["raw_log"].pop(0)

        # CH:... MODE:...
        if line.startswith("CH:"):
            try:
                ch_part, mode_str = line.split(" MODE:")
                chs = [int(x) for x in ch_part[3:].split(",")]
                if len(chs) == 16:
                    state["channels"] = chs
                    state["mode"]     = mode_str.strip()
                    state["frame_count"] += 1
                    state["last_frame"]  = time.time()
            except Exception:
                pass

        # SBUS status
        elif line == "[SBUS_OK]":    state["sbus_ok"] = True
        elif line == "[SBUS_LOST]":  state["sbus_ok"] = False

        # SX1278 status (from [BOOT] line)
        elif "SX1278_ERROR" in line: state["sx_ok"] = False
        elif "RC link master ready" in line and state["sx_ok"] is None:
            state["sx_ok"] = True   # booted without error → assume OK

        # Heartbeat echo
        elif line.startswith("<HB:"):
            try:
                state["hb_rx_seq"] = int(line[4:-1])
            except Exception:
                pass

# ── Heartbeat sender thread ───────────────────────────────────────────────────

def heartbeat_sender(ser):
    while state["running"]:
        if state["hb_tx"]:
            with lock:
                seq = state["hb_seq"]
                state["hb_seq"] += 1
            try:
                ser.write(f"<HB:{seq}>\n".encode())
            except serial.SerialException:
                break
        time.sleep(0.2)   # 5 Hz — well within 300 ms timeout

# ── Display ───────────────────────────────────────────────────────────────────

CH_NAMES = ["THR", "STR", "CH3", "CH4", "SWA", "SWB", "CH7", "CH8",
            "CH9", "C10", "C11", "C12", "C13", "C14", "C15", "C16"]

def display():
    while state["running"]:
        time.sleep(0.1)
        with lock:
            chs        = state["channels"][:]
            mode       = state["mode"]
            sbus_ok    = state["sbus_ok"]
            sx_ok      = state["sx_ok"]
            hb_tx      = state["hb_tx"]
            hb_rx      = state["hb_rx_seq"]
            fc         = state["frame_count"]
            age        = time.time() - state["last_frame"]
            raw_log    = state["raw_log"][:]

        # Build display string
        lines = []
        lines.append(f"\033[2J\033[H")   # clear screen + cursor home

        lines.append(f"{BLD}{'─'*60}")
        lines.append(f"  RP2040 MASTER TEST  {RST}")
        lines.append(f"{BLD}{'─'*60}{RST}")

        # Status row
        sbus_str = f"{G}SBUS OK{RST}" if sbus_ok else f"{R}SBUS LOST{RST}"
        sx_str   = (f"{G}SX1278 OK{RST}" if sx_ok
                    else f"{R}SX1278 ERR{RST}" if sx_ok is False
                    else f"{Y}SX1278 ?{RST}")
        hb_str   = (f"{G}HB ON  (echo:{hb_rx}){RST}" if hb_tx
                    else f"{Y}HB OFF{RST}")
        frame_str = f"frames:{fc}  age:{age*1000:.0f}ms"
        lines.append(f"  {sbus_str}   {sx_str}   {hb_str}")
        lines.append(f"  {frame_str}")

        # Mode
        mc = MODE_COLOR.get(mode, W)
        lines.append(f"\n  MODE: {BLD}{mc}{mode}{RST}\n")

        # Channel bars
        for i, (name, val) in enumerate(zip(CH_NAMES, chs)):
            b = bar(val)
            color = R if (i == 4 and val < 1700) else \
                    C if (i == 5 and val > 1700) else W
            lines.append(f"  {name:4s} {color}{val:4d}{RST} {b}")

        # Raw log
        lines.append(f"\n{BLD}  Recent output:{RST}")
        for l in raw_log[-4:]:
            lines.append(f"  {C}{l}{RST}")

        # Help
        lines.append(f"\n{Y}  Commands: hb | j | j+ | raw <x> | q{RST}")
        lines.append(f"  > ")

        print("".join(lines), end="", flush=True)

# ── Command interface ─────────────────────────────────────────────────────────

def cmd_loop(ser):
    while state["running"]:
        # Non-blocking stdin check (works on Linux/macOS; on Windows use msvcrt)
        if select.select([sys.stdin], [], [], 0.05)[0]:
            try:
                line = sys.stdin.readline().strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not line:
                continue

            if line == "q":
                state["running"] = False

            elif line == "hb":
                with lock:
                    state["hb_tx"] = not state["hb_tx"]

            elif line == "j":
                cmd = "<J:1500,1500,1500,1500,1500,1500,1500,1500>\n"
                ser.write(cmd.encode())

            elif line == "j+":
                cmd = "<J:1700,1500,1500,1500,1500,1500,1500,1500>\n"
                ser.write(cmd.encode())

            elif line.startswith("raw "):
                payload = line[4:] + "\n"
                ser.write(payload.encode())

            else:
                with lock:
                    state["raw_log"].append(f"[unknown cmd: {line}]")

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",  default=None)
    parser.add_argument("--no-hb", action="store_true",
                        help="Disable heartbeat (tests HB timeout behaviour)")
    args = parser.parse_args()

    port = args.port or find_port()
    print(f"Connecting to {port}...")

    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"{R}Cannot open {port}: {e}{RST}")

    if args.no_hb:
        state["hb_tx"] = False

    print(f"{G}Connected.{RST} Waiting for RP2040 output...\n")
    time.sleep(0.5)

    threads = [
        threading.Thread(target=reader,           args=(ser,), daemon=True),
        threading.Thread(target=heartbeat_sender, args=(ser,), daemon=True),
        threading.Thread(target=display,          daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        cmd_loop(ser)
    except KeyboardInterrupt:
        pass
    finally:
        state["running"] = False
        time.sleep(0.2)
        ser.close()
        print(f"\n{RST}Disconnected.")

if __name__ == "__main__":
    main()
