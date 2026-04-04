"""
Launch rosbridge WebSocket server.

Run separately from the main rover launch to avoid crash if not installed:
  ros2 launch agri_rover_bringup rosbridge.launch.py
  ros2 launch agri_rover_bringup rosbridge.launch.py port:=9091

The rover1/rover2 launch files do NOT include this — start it manually
or via docker-entrypoint after verifying the package is installed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090',
                              description='WebSocket port'),
        Node(package='rosbridge_server',
             executable='rosbridge_websocket',
             parameters=[{'port': LaunchConfiguration('port')}],
             output='screen'),
    ])
