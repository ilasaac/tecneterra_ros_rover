"""
Launch file for Rover 2 (Slave).
Run on the Jetson on the slave rover.

  ros2 launch agri_rover_bringup rover2.launch.py
  ros2 launch agri_rover_bringup rover2.launch.py camera_source:=usb
  ros2 launch agri_rover_bringup rover2.launch.py camera_source:=test

See rover1.launch.py for camera_source documentation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('agri_rover_bringup'),
        'config', 'rover2_params.yaml')

    video_pipeline_launch = os.path.join(
        get_package_share_directory('agri_rover_video'),
        'launch', 'video_pipeline.launch.py')

    ns = '/rv2'
    cam = LaunchConfiguration('camera_source')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_source',
            default_value='csi',
            description='Camera source: csi (Isaac ROS) | usb | test'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(video_pipeline_launch),
            launch_arguments={
                'namespace': ns,
                'sensor_id': '0',
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", cam, "' == 'csi'"])
            ),
        ),

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
             parameters=[config, {'camera_source': cam}],
             output='screen'),

        Node(package='agri_rover_navigator',
             executable='navigator',
             namespace=ns,
             parameters=[config],
             output='screen'),
    ])
