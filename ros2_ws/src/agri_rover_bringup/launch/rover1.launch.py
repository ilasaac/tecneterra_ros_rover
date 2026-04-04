"""
Launch file for Rover 1 (Master).
Run on the Jetson connected to the HM30 air unit.

  ros2 launch agri_rover_bringup rover1.launch.py
  ros2 launch agri_rover_bringup rover1.launch.py camera_source:=usb
  ros2 launch agri_rover_bringup rover1.launch.py camera_source:=test

camera_source:
  csi  — Isaac ROS NITROS pipeline (ArgusMonoNode + H264EncoderNode composable
         nodes) + GstRtspServer appsrc.  Default for Jetson CSI ribbon cable.
  usb  — GStreamer subprocess with v4l2src + x264enc. No Isaac ROS required.
  test — GStreamer subprocess with videotestsrc.  Useful without any camera.
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
        'config', 'rover1_params.yaml')

    video_pipeline_launch = os.path.join(
        get_package_share_directory('agri_rover_video'),
        'launch', 'video_pipeline.launch.py')

    ns = '/rv1'
    cam = LaunchConfiguration('camera_source')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_source',
            default_value='csi',
            description='Camera source: csi (Isaac ROS) | usb | test'),

        # ── Isaac ROS NITROS composable container (CSI only) ─────────────────
        # ArgusMonoNode + H264EncoderNode share zero-copy GPU memory.
        # Skipped automatically for usb/test — video_streamer handles those.
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

        # ── Standard ROS2 nodes ───────────────────────────────────────────────
        Node(package='agri_rover_rp2040',
             executable='rp2040_bridge',
             namespace=ns,
             parameters=[config],
             output='screen',
             respawn=True,
             respawn_delay=2.0),

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

        # video_streamer:
        #   csi  → subscribes to image_compressed (from NITROS container above)
        #          and serves RTSP via GstRtspServer appsrc
        #   usb/test → spawns gst-rtsp-server subprocess directly
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
