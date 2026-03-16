"""
video_pipeline.launch.py — Isaac ROS NITROS composable node container.

Launches ArgusMonoNode (CSI capture) and EncoderNode (H264 hardware encode)
as composable nodes sharing zero-copy NITROS memory on the Jetson GPU.

Output topic:  image_compressed  (sensor_msgs/CompressedImage, format="h264")
Consumed by:   video_streamer node → GstRtspServer appsrc → RTSP clients

Only launched when camera_source == 'csi'.
For USB / test, video_streamer spawns a GStreamer subprocess directly and
this file is not included.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace',     default_value='/rv1'),
        DeclareLaunchArgument('sensor_id',     default_value='0'),
        DeclareLaunchArgument('width',         default_value='1280'),
        DeclareLaunchArgument('height',        default_value='720'),
        DeclareLaunchArgument('bitrate',       default_value='2000000'),  # bps

        ComposableNodeContainer(
            name='video_container',
            namespace=LaunchConfiguration('namespace'),
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[

                # CSI camera — publishes sensor_msgs/Image via NITROS
                ComposableNode(
                    package='isaac_ros_argus_camera',
                    plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
                    name='argus_camera',
                    parameters=[{
                        'sensor_id':       LaunchConfiguration('sensor_id'),
                        'output_encoding': 'rgb8',
                    }],
                    remappings=[('image_raw', 'camera/image_raw')],
                ),

                # H264 hardware encoder — publishes sensor_msgs/CompressedImage via NITROS
                ComposableNode(
                    package='isaac_ros_h264_encoder',
                    plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                    name='h264_encoder',
                    parameters=[{
                        'input_width':  LaunchConfiguration('width'),
                        'input_height': LaunchConfiguration('height'),
                        'bitrate':      LaunchConfiguration('bitrate'),
                    }],
                    remappings=[
                        ('image_raw',        'camera/image_raw'),
                        ('image_compressed', 'image_compressed'),
                    ],
                ),
            ],
            output='screen',
        ),
    ])
