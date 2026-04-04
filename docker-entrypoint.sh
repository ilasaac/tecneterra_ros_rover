#!/bin/bash
# Source ROS2 + Isaac ROS + project workspace overlays, then exec the command.
ROS_SETUP=$(ls /opt/ros/*/setup.bash 2>/dev/null | head -1)
source ${ROS_SETUP}
source ${ISAAC_ROS_WS}/install/setup.bash

# Start rosbridge in background if available (non-fatal if missing)
if ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
    echo "[entrypoint] rosbridge_server started on port 9090"
fi

exec "$@"
