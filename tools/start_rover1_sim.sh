#!/bin/bash
# Start the full RV1 simulation stack in one step.
#
# What this does:
#   1. Starts the rover1 Docker container (detached)
#   2. Runs nmea_wifi_rx.py INSIDE the container so PTY devices live in the
#      container's own /dev/pts (host devpts is separate — host PTYs not visible)
#   3. gps_driver retries every 5 s until the PTY symlinks appear
#   4. Follows container logs (Ctrl+C stops everything)
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
    docker compose down rover1 2>/dev/null || true
    rm -f "$PRI_SYMLINK" "$SEC_SYMLINK"
    echo "[sim] Done."
}
trap cleanup EXIT INT TERM

# ── Step 1: start rover1 container (detached) ──────────────────────────────────
echo "[sim] Starting rover1 Docker container..."
CAMERA_SOURCE=test docker compose up --force-recreate -d rover1

# Wait for container to be running
for i in $(seq 1 20); do
    if docker inspect -f '{{.State.Running}}' agri_rover_rv1 2>/dev/null | grep -q true; then
        echo "[sim] Container running"
        break
    fi
    sleep 0.5
done
if ! docker inspect -f '{{.State.Running}}' agri_rover_rv1 2>/dev/null | grep -q true; then
    echo "[ERR] Container failed to start"
    exit 1
fi

# ── Step 2: run nmea_wifi_rx.py inside container ───────────────────────────────
# PTYs created here live in the container's /dev/pts — gps_driver can open them.
# Symlinks appear at /tmp/rv1_gps_{pri,sec} (shared via /tmp:/tmp mount).
echo "[sim] Starting NMEA WiFi receiver inside container..."
docker exec -d agri_rover_rv1 \
    python3 /workspaces/isaac_ros-dev/src/tools/nmea_wifi_rx.py \
        --pri-port 5000 --sec-port 5001 \
        --pri-symlink "$PRI_SYMLINK" \
        --sec-symlink "$SEC_SYMLINK"

# ── Step 3: follow logs (Ctrl+C → cleanup trap runs) ──────────────────────────
echo "[sim] Following logs (Ctrl+C to stop)..."
docker compose logs -f rover1
