#!/usr/bin/env python3
"""
upload_mission.py — Load a mission JSON file and publish on ROS2 topics.

Runs INSIDE the rover Docker container. Reads a JSON file from disk and
publishes it so the navigator loads the mission — same as if mavlink_bridge
received it from GQC.

Usage (inside container):
  python3 /workspaces/isaac_ros-dev/tools/upload_mission.py /tmp/mission.json
  python3 /workspaces/isaac_ros-dev/tools/upload_mission.py /tmp/mission.json --ns /rv2

Supports two mission types (auto-detected from JSON):
  - Corridor: publishes on /{ns}/corridor_mission (String)
  - Waypoint: publishes on /{ns}/mission (MissionWaypoint) per waypoint

JSON format:
  Corridor: {"corridors": [...], "min_turn_radius": 3.0, ...}
  Waypoint: {"waypoints": [{"lat":..., "lon":..., "speed":..., "hold_secs":...}, ...],
             "obstacles": [[...], ...]}
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from agri_rover_interfaces.msg import MissionWaypoint


class MissionLoader(Node):
    def __init__(self, ns: str):
        super().__init__('mission_loader')
        self.pub_corridor = self.create_publisher(String, f'{ns}/corridor_mission', 10)
        self.pub_mission = self.create_publisher(MissionWaypoint, f'{ns}/mission', 10)
        self.pub_fence = self.create_publisher(String, f'{ns}/mission_fence', 10)

    def publish_corridor(self, json_str: str):
        msg = String()
        msg.data = json_str
        self.pub_corridor.publish(msg)
        self.get_logger().info(f'Published corridor mission ({len(json_str)} bytes)')

    def publish_waypoints(self, waypoints: list, obstacles: list | None = None):
        for i, wp in enumerate(waypoints):
            msg = MissionWaypoint()
            msg.seq = i
            msg.latitude = float(wp['lat'])
            msg.longitude = float(wp['lon'])
            msg.speed = float(wp.get('speed', 0))
            msg.hold_secs = float(wp.get('hold_secs', 0))
            msg.acceptance_radius = float(wp.get('acceptance_radius', 0))
            self.pub_mission.publish(msg)
            time.sleep(0.01)
        self.get_logger().info(f'Published {len(waypoints)} waypoints')

        if obstacles:
            fence = json.dumps({'polygons': obstacles})
            msg = String()
            msg.data = fence
            self.pub_fence.publish(msg)
            self.get_logger().info(f'Published {len(obstacles)} obstacle polygon(s)')


def main():
    parser = argparse.ArgumentParser(description='Load mission JSON onto ROS2 topics')
    parser.add_argument('file', help='Path to mission JSON file')
    parser.add_argument('--ns', default='/rv1', help='ROS2 namespace (default: /rv1)')
    args = parser.parse_args()

    with open(args.file) as f:
        data = json.load(f)

    rclpy.init()
    node = MissionLoader(args.ns)

    # Wait for topic discovery
    time.sleep(1.5)

    # Auto-detect mission type
    if 'corridors' in data:
        # Corridor mission — publish raw JSON string
        node.publish_corridor(json.dumps(data))
    elif 'waypoints' in data:
        # Waypoint mission
        node.publish_waypoints(data['waypoints'], data.get('obstacles'))
    else:
        node.get_logger().error(f'Unknown mission format — expected "corridors" or "waypoints" key')
        sys.exit(1)

    # Give subscribers time to process
    time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()
    print('Done.')


if __name__ == '__main__':
    main()
