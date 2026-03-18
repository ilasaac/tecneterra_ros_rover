#!/bin/bash
# Start the full RV2 simulation stack in one step.
#
# What this does:
#   1. Starts the rover2 Docker container (detached)
#   2. Runs nmea_wifi_rx.py INSIDE the container so PTY devices live in the
#      container's own /dev/pts (host devpts is separate — host PTYs not visible)
#   3. gps_driver retries every 5 s until the PTY symlinks appear
#   4. Follows container logs (Ctrl+C stops everything)
#
# On the simulator Jetson, run separately:
#   python3 tools/simulator.py --rv1-ip <rv1-ip> --rv2-ip <this-ip> --ppm-port /dev/ttyACM0
#
# Usage:
#   bash tools/start_rover2_sim.sh

set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."

PRI_SYMLINK=/tmp/rv2_gps_pri
SEC_SYMLINK=/tmp/rv2_gps_sec

cleanup() {
    echo ""
    echo "[sim] Stopping..."
    # Remove symlinks from inside the container (they are owned by root — host rm fails)
    docker exec agri_rover_rv2 rm -f "$PRI_SYMLINK" "$SEC_SYMLINK" 2>/dev/null || true
    docker compose down --timeout 5 rover2 2>/dev/null || true
    echo "[sim] Done."
}
trap cleanup EXIT INT TERM

# ── Step 0: disable WiFi power-save ────────────────────────────────────────────
# WiFi power-save causes the AP to buffer incoming unicast packets (MISSION_ITEM_INT)
# until the Jetson polls, adding 100–900 ms per mission item.  TX traffic (telemetry)
# does NOT prevent this — the Jetson can transmit while its RX is sleeping.
WIFI_IF=$(iw dev 2>/dev/null | awk '/Interface/{print $2}' | head -1)
if [ -n "$WIFI_IF" ]; then
    sudo iw dev "$WIFI_IF" set power_save off 2>/dev/null && \
        echo "[sim] WiFi power-save disabled on $WIFI_IF" || \
        echo "[sim] WARN: could not disable WiFi power-save (no sudo?)"
fi

# ── Step 1: start rover2 container (detached) ──────────────────────────────────
echo "[sim] Starting rover2 Docker container..."
CAMERA_SOURCE=test docker compose up --force-recreate -d rover2

# Wait for container to be running
for i in $(seq 1 20); do
    if docker inspect -f '{{.State.Running}}' agri_rover_rv2 2>/dev/null | grep -q true; then
        echo "[sim] Container running"
        break
    fi
    sleep 0.5
done
if ! docker inspect -f '{{.State.Running}}' agri_rover_rv2 2>/dev/null | grep -q true; then
    echo "[ERR] Container failed to start"
    exit 1
fi

# ── Step 2: run nmea_wifi_rx.py inside container ───────────────────────────────
# PTYs created here live in the container's /dev/pts — gps_driver can open them.
# Symlinks appear at /tmp/rv2_gps_{pri,sec} (shared via /tmp:/tmp mount).
echo "[sim] Starting NMEA WiFi receiver inside container..."
docker exec -d agri_rover_rv2 \
    python3 /workspaces/isaac_ros-dev/tools/nmea_wifi_rx.py \
        --pri-port 5000 --sec-port 5001 \
        --pri-symlink "$PRI_SYMLINK" \
        --sec-symlink "$SEC_SYMLINK"

# ── Step 3: follow logs (Ctrl+C → cleanup trap runs) ──────────────────────────
echo "[sim] Following logs (Ctrl+C to stop)..."
docker compose logs -f rover2
