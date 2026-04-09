#!/bin/bash
# Start RV2 for real-hardware field operation.
#
# What this does:
#   1. Disables WiFi power-save (prevents 100-900 ms MAVLink latency)
#   2. Starts the rover2 Docker container
#   3. Starts rtk_forwarder.py on the host to pipe NTRIP corrections
#      to /dev/rv2_gps_pri and /dev/rv2_gps_sec
#   4. Follows container logs (Ctrl+C stops everything cleanly)
#
# NTRIP credentials are loaded from tools/.env.rtk (gitignored).
# Copy tools/.env.rtk.example to tools/.env.rtk and fill in your credentials.
#
# Usage:
#   bash tools/start_rover2.sh

set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."

# ── Load NTRIP credentials ──────────────────────────────────────────────────
ENV_FILE="tools/.env.rtk"
if [ ! -f "$ENV_FILE" ]; then
    echo "[ERR] $ENV_FILE not found."
    echo "      Copy tools/.env.rtk.example to tools/.env.rtk and fill in credentials."
    exit 1
fi
# shellcheck source=/dev/null
source "$ENV_FILE"

: "${NTRIP_HOST:?tools/.env.rtk must set NTRIP_HOST}"
: "${NTRIP_PORT:?tools/.env.rtk must set NTRIP_PORT}"
: "${NTRIP_USER:?tools/.env.rtk must set NTRIP_USER}"
: "${NTRIP_PASS:?tools/.env.rtk must set NTRIP_PASS}"
: "${NTRIP_MOUNT:?tools/.env.rtk must set NTRIP_MOUNT}"
: "${APPROX_LAT:?tools/.env.rtk must set APPROX_LAT}"
: "${APPROX_LON:?tools/.env.rtk must set APPROX_LON}"

RTK_PID=""

cleanup() {
    echo ""
    echo "[rv2] Stopping..."
    [ -n "$RTK_PID" ] && kill "$RTK_PID" 2>/dev/null || true
    docker compose down --timeout 5 rover2 2>/dev/null || true
    echo "[rv2] Done."
}
trap cleanup EXIT INT TERM

# ── Step 0: disable WiFi power-save ────────────────────────────────────────
WIFI_IF=$(iw dev 2>/dev/null | awk '/Interface/{print $2}' | head -1)
if [ -n "$WIFI_IF" ]; then
    sudo iw dev "$WIFI_IF" set power_save off 2>/dev/null && \
        echo "[rv2] WiFi power-save disabled on $WIFI_IF" || \
        echo "[rv2] WARN: could not disable WiFi power-save (no sudo?)"
fi

# ── Step 1: start rover2 container (detached) ──────────────────────────────
echo "[rv2] Starting rover2 Docker container..."
docker compose up --force-recreate -d rover2

for i in $(seq 1 20); do
    if docker inspect -f '{{.State.Running}}' agri_rover_rv2 2>/dev/null | grep -q true; then
        echo "[rv2] Container running"
        break
    fi
    sleep 0.5
done
if ! docker inspect -f '{{.State.Running}}' agri_rover_rv2 2>/dev/null | grep -q true; then
    echo "[ERR] Container failed to start"
    exit 1
fi

# ── Step 2: start RTK forwarder on host ────────────────────────────────────
# Runs on the host (not inside Docker) — accesses /dev/rv2_gps_{pri,sec} directly.
echo "[rv2] Starting RTK forwarder → $NTRIP_HOST:$NTRIP_PORT/$NTRIP_MOUNT"
python3 tools/rtk_forwarder.py \
    --source ntrip \
    --ntrip-host "$NTRIP_HOST" \
    --ntrip-port "$NTRIP_PORT" \
    --ntrip-user "$NTRIP_USER" \
    --ntrip-pass "$NTRIP_PASS" \
    --mountpoint  "$NTRIP_MOUNT" \
    --approx-lat  "$APPROX_LAT" \
    --approx-lon  "$APPROX_LON" \
    --gps-ports /dev/ttyUSB0 /dev/ttyUSB1 \
    --log-file /tmp/rtk_rv2.log &
RTK_PID=$!
echo "[rv2] RTK forwarder PID=$RTK_PID  (log: /tmp/rtk_rv2.log)"

# ── Step 3: follow logs (Ctrl+C → cleanup trap runs) ──────────────────────
echo "[rv2] Following logs (Ctrl+C to stop)..."
docker compose logs -f rover2
