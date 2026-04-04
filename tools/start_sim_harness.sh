#!/bin/bash
# Start sim-only stack: navigator + mavlink_bridge + sim_harness
# Usage: bash tools/start_sim_harness.sh [timeout_seconds]
#   Default timeout: 300s (5 min)
#
# The sim_harness node auto-exits when the navigator publishes wp_active=-1
# (mission complete) or when the timeout expires.

set -e
TIMEOUT=${1:-300}
cd "$(dirname "${BASH_SOURCE[0]}")/.."

echo "[sim_harness] Stopping production stack if running..."
docker compose down rover1 2>/dev/null || true
sleep 1

echo "[sim_harness] Building (if needed)..."
docker compose build sim1 2>/dev/null || true

echo "[sim_harness] Starting sim stack (timeout=${TIMEOUT}s)..."
docker compose up -d sim1

# Follow logs until container exits or timeout
timeout "$TIMEOUT" docker compose logs -f sim1 2>&1 || true

EXIT_CODE=$(docker inspect -f '{{.State.ExitCode}}' agri_rover_rv1_sim 2>/dev/null || echo 1)

echo "[sim_harness] Cleaning up..."
docker compose down sim1 2>/dev/null || true

echo "[sim_harness] Done (exit code: $EXIT_CODE)"
exit "$EXIT_CODE"
