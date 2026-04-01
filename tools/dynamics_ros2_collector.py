#!/usr/bin/env python3
"""
Dynamics data collector via ROS2 topics (runs INSIDE the rover container).

Records PPM commands + GPS at 25 Hz into a single CSV for model tuning.
Subscribes to rc_input (manual PPM) and cmd_override (autonomous PPM),
fix (GPS), and heading.

Usage:
  docker exec -d agri_rover_rv2 python3 /workspaces/tools/dynamics_ros2_collector.py \
      --ns rv2 --output /tmp/dynamics_log.csv

  # Then drive manually or run autonomous. Ctrl+C or kill to stop.
  # Copy out: docker cp agri_rover_rv2:/tmp/dynamics_log.csv .
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

# Import after ROS2 init to get the correct message types
from agri_rover_interfaces.msg import RCInput


class DynamicsCollector(Node):

    def __init__(self, ns: str, output: str):
        super().__init__('dynamics_collector')

        self._output = output
        self._file = open(output, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            't', 'ppm_throttle', 'ppm_steer',
            'ppm_ch3', 'ppm_ch4', 'ppm_ch5', 'ppm_ch6', 'ppm_ch7', 'ppm_ch8',
            'lat', 'lon', 'heading',
            'source',  # 'rc' or 'cmd' — which PPM source
        ])

        self._lat = 0.0
        self._lon = 0.0
        self._heading = 0.0
        self._fix_time = 0.0
        self._count = 0

        # Subscribe to topics
        self.create_subscription(RCInput,  f'/{ns}/rc_input',     self._cb_rc,      10)
        self.create_subscription(RCInput,  f'/{ns}/cmd_override', self._cb_cmd,     10)
        self.create_subscription(NavSatFix, f'/{ns}/fix',         self._cb_fix,     10)
        self.create_subscription(Float32,  f'/{ns}/heading',      self._cb_heading, 10)

        self.get_logger().info(f'Dynamics collector started -> {output}')
        self.get_logger().info(f'Subscribed to /{ns}/rc_input, cmd_override, fix, heading')

    def _cb_fix(self, msg: NavSatFix):
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._fix_time = time.time()

    def _cb_heading(self, msg: Float32):
        self._heading = msg.data

    def _cb_rc(self, msg: RCInput):
        self._write_row(msg, 'rc')

    def _cb_cmd(self, msg: RCInput):
        self._write_row(msg, 'cmd')

    def _write_row(self, msg: RCInput, source: str):
        ch = list(msg.channels) + [1500] * (16 - len(msg.channels))
        self._writer.writerow([
            round(time.time(), 4),
            ch[0], ch[1],  # throttle, steer
            ch[2], ch[3], ch[4], ch[5], ch[6], ch[7],
            round(self._lat, 8),
            round(self._lon, 8),
            round(self._heading, 2),
            source,
        ])
        self._count += 1
        if self._count % 250 == 0:
            self._file.flush()
            self.get_logger().info(f'Recorded {self._count} rows')

    def close(self):
        self._file.flush()
        self._file.close()
        self.get_logger().info(f'Saved {self._count} rows to {self._output}')


def main():
    parser = argparse.ArgumentParser(description='ROS2 dynamics collector')
    parser.add_argument('--ns', default='rv2', help='Rover namespace (rv1 or rv2)')
    parser.add_argument('--output', '-o', default='/tmp/dynamics_log.csv',
                        help='Output CSV path')
    args = parser.parse_args()

    rclpy.init()
    node = DynamicsCollector(args.ns, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
