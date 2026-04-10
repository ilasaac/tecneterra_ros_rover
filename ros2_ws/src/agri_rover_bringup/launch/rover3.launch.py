"""
Launch file for Rover 3 (Simulation).
Run on the third Jetson with NO physical GPS or RP2040 hardware.

  ros2 launch agri_rover_bringup rover3.launch.py
  ros2 launch agri_rover_bringup rover3.launch.py camera_source:=test

This is identical to rover2.launch.py except gps_driver and rp2040_bridge
are replaced by sim_harness, which feeds simulated fix/heading/rc_input
to the navigator. Everything else (navigator, sensors, video) runs
unchanged so the rover behaves as close to a field rover as possible.

Mission upload, rosbridge discovery, GQC integration all work the same.
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
        'config', 'rover3_params.yaml')

    video_pipeline_launch = os.path.join(
        get_package_share_directory('agri_rover_video'),
        'launch', 'video_pipeline.launch.py')

    ns = '/rv3'
    cam = LaunchConfiguration('camera_source')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_source',
            default_value='test',
            description='Camera source: csi (Isaac ROS) | usb | test (default test for sim)'),

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

        # gps_driver REPLACED by sim_harness — no physical GPS hardware
        # rp2040_bridge REPLACED by sim_harness — no physical RP2040
        Node(package='agri_rover_simulator',
             executable='sim_harness',
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
