#!/bin/bash
# Start the full RV1 simulation stack in one step.
#
# What this does:
#   1. Starts nmea_wifi_rx.py (creates /tmp/rv1_gps_pri and /tmp/rv1_gps_sec)
#   2. Waits for the virtual serial ports to be ready
#   3. Starts the rover1 Docker container (reads GPS ports from rover1_params.yaml)
#
# On the simulator Jetson, run separately:
#   python3 tools/simulator.py --rv1-ip <this-ip> --rv2-ip <rv2-ip> --ppm-port /dev/ttyACM0
#
# Usage:
#   bash tools/start_rover1_sim.sh

set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."

PRI_SYMLINK=/tmp/rv1_gps_pri
SEC_SYMLINK=/tmp/rv1_gps_sec

cleanup() {
    echo ""
    echo "[sim] Stopping..."
    docker compose stop rover1 2>/dev/null || true
    kill "$NMEA_PID" 2>/dev/null || true
    rm -f "$PRI_SYMLINK" "$SEC_SYMLINK"
    echo "[sim] Done."
}
trap cleanup EXIT INT TERM

# ── Step 1: start nmea_wifi_rx.py ──────────────────────────────────────────────
echo "[sim] Starting NMEA WiFi receiver..."
python3 tools/nmea_wifi_rx.py \
    --pri-port 5000 --sec-port 5001 \
    --pri-symlink "$PRI_SYMLINK" \
    --sec-symlink "$SEC_SYMLINK" &
NMEA_PID=$!

# ── Step 2: wait for symlinks ──────────────────────────────────────────────────
echo "[sim] Waiting for virtual serial ports..."
for i in $(seq 1 20); do
    if [ -e "$PRI_SYMLINK" ] && [ -e "$SEC_SYMLINK" ]; then
        echo "[sim] Virtual ports ready:"
        echo "      primary:   $PRI_SYMLINK -> $(readlink $PRI_SYMLINK)"
        echo "      secondary: $SEC_SYMLINK -> $(readlink $SEC_SYMLINK)"
        break
    fi
    sleep 0.3
done

if [ ! -e "$PRI_SYMLINK" ]; then
    echo "[ERR] Virtual ports not created — nmea_wifi_rx.py may have failed"
    exit 1
fi

# ── Step 3: start rover1 container ────────────────────────────────────────────
echo "[sim] Starting rover1 Docker container..."
docker compose up rover1
