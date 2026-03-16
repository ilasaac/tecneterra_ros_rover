#!/bin/bash
# Source ROS2 + Isaac ROS + project workspace overlays, then exec the command.
source /opt/ros/humble/setup.bash
source ${ISAAC_ROS_WS}/install/setup.bash
exec "$@"
