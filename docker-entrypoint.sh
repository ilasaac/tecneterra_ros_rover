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

# Discovery beacon — broadcast rover identity on UDP 5555 every 2s.
# GQC and mission_planner listen for this to auto-discover rover IPs.
# Extracts rover_id from the launch command (rover1 → 1, rover2 → 2).
ROVER_ID=0
case "$@" in
    *rover1*) ROVER_ID=1 ;;
    *rover2*) ROVER_ID=2 ;;
    *sim_harness*) ROVER_ID=0 ;;
esac
if [ "$ROVER_ID" -gt 0 ]; then
    (while true; do
        echo -n "{\"sysid\":$ROVER_ID,\"rosbridge\":9090}" | \
            socat - UDP-DATAGRAM:255.255.255.255:5555,broadcast 2>/dev/null || \
        python3 -c "
import socket,time
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
s.sendto(b'{\"sysid\":$ROVER_ID,\"rosbridge\":9090}',('255.255.255.255',5555))
s.close()
" 2>/dev/null
        sleep 2
    done) &
    echo "[entrypoint] Discovery beacon started (sysid=$ROVER_ID, port 5555)"
fi

exec "$@"
