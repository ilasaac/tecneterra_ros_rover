#!/bin/bash
# Run sim_harness inside the already-running rover1 container.
# Usage: bash tools/start_sim_harness.sh [timeout_seconds]
#   Default timeout: 300s (5 min)
#
# Requires: docker compose up rover1 (already running)
# Upload the mission AFTER this script starts (sim_harness must be
# subscribed to topics before mission items are published).
#
# The sim_harness node auto-exits when the navigator publishes wp_active=-1.
# Rover1 keeps running — no restart needed.

TIMEOUT=${1:-300}

echo "[sim_harness] Running inside agri_rover_rv1 (timeout=${TIMEOUT}s)..."
echo "[sim_harness] Upload mission NOW (sim_harness is listening)..."
timeout "$TIMEOUT" docker exec agri_rover_rv1 bash -c \
  'source /opt/ros/*/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 run agri_rover_simulator sim_harness --ros-args -r __ns:=/rv1 --params-file /workspaces/isaac_ros-dev/install/agri_rover_bringup/share/agri_rover_bringup/config/rover1_params.yaml' \
  2>&1

echo "[sim_harness] Done (exit code: $?)"
