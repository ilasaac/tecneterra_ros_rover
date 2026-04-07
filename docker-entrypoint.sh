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

# Discovery beacon — broadcast rover identity + GPS position on UDP 5555 every 2s.
# GQC and mission_planner listen for this to auto-discover rover IPs and track position.
# Subscribes to local ROS2 fix/heading topics (DDS loopback — no network traffic).
ROVER_ID=0
case "$@" in
    *rover1*) ROVER_ID=1 ;;
    *rover2*) ROVER_ID=2 ;;
    *sim_harness*) ROVER_ID=0 ;;
esac
if [ "$ROVER_ID" -gt 0 ]; then
    python3 -u -c "
import socket, time, json, threading

sysid = $ROVER_ID
ns = '/rv$ROVER_ID'
pos = {}  # {lat, lon, hdg}

def ros_listener():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Float32, Int32, Bool
    rclpy.init()
    node = rclpy.create_node('beacon_$ROVER_ID')
    def on_fix(msg):
        if msg.latitude != 0.0:
            pos['lat'] = msg.latitude
            pos['lon'] = msg.longitude
    def on_hdg(msg):
        pos['hdg'] = msg.data
    def on_wp(msg):
        pos['wp_active'] = msg.data
    def on_armed(msg):
        pos['armed'] = msg.data
    node.create_subscription(NavSatFix, ns + '/fix', on_fix, 10)
    node.create_subscription(Float32, ns + '/heading', on_hdg, 10)
    node.create_subscription(Int32, ns + '/wp_active', on_wp, 10)
    node.create_subscription(Bool, ns + '/armed', on_armed, 10)
    rclpy.spin(node)

t = threading.Thread(target=ros_listener, daemon=True)
t.start()
time.sleep(1)  # let ROS2 init

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
while True:
    pkt = {'sysid': sysid, 'rosbridge': 9090}
    if 'lat' in pos:
        pkt['lat'] = round(pos['lat'], 8)
        pkt['lon'] = round(pos['lon'], 8)
    if 'hdg' in pos:
        pkt['hdg'] = round(pos['hdg'], 2)
    if 'wp_active' in pos:
        pkt['wp_active'] = pos['wp_active']
    if 'armed' in pos:
        pkt['armed'] = pos['armed']
    s.sendto(json.dumps(pkt).encode(), ('255.255.255.255', 5555))
    time.sleep(2)
" &
    echo "[entrypoint] Discovery beacon started (sysid=$ROVER_ID, port 5555, with GPS)"
fi

# Run main command in background and forward signals so beacon stays alive
"$@" &
MAIN_PID=$!
trap 'kill $MAIN_PID 2>/dev/null; wait $MAIN_PID' TERM INT
wait $MAIN_PID
