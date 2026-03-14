"""
Launch file for Rover 1 (Master).
Run on the Jetson connected to the HM30 air unit.

  ros2 launch agri_rover_bringup rover1.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('agri_rover_bringup'),
        'config', 'rover1_params.yaml')

    ns = '/rv1'

    return LaunchDescription([
        Node(package='agri_rover_rp2040',
             executable='rp2040_bridge',
             namespace=ns,
             parameters=[config],
             output='screen'),

        Node(package='agri_rover_gps',
             executable='gps_driver',
             namespace=ns,
             parameters=[config],
             output='screen'),

        Node(package='agri_rover_mavlink',
             executable='mavlink_bridge',
             namespace=ns,
             parameters=[config],
             output='screen'),

        Node(package='agri_rover_sensors',
             executable='sensor_node',
             namespace=ns,
             parameters=[config],
             output='screen'),

        Node(package='agri_rover_video',
             executable='video_streamer',
             namespace=ns,
             parameters=[config],
             output='screen'),

        Node(package='agri_rover_navigator',
             executable='navigator',
             namespace=ns,
             parameters=[config],
             output='screen'),
    ])
