#!/bin/bash
# Source ROS2 + Isaac ROS + project workspace overlays, then exec the command.
ROS_SETUP=$(ls /opt/ros/*/setup.bash 2>/dev/null | head -1)
source ${ROS_SETUP}
source ${ISAAC_ROS_WS}/install/setup.bash
exec "$@"
