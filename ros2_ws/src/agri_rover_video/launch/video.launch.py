from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rover_id = LaunchConfiguration('rover_id', default='1')
    config   = os.path.join(
        get_package_share_directory('agri_rover_video'),
        'config', 'gstreamer.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('rover_id',     default_value='1'),
        DeclareLaunchArgument('rtsp_port',    default_value='8554',
                              description='RV1=8554, RV2=8555'),
        DeclareLaunchArgument('camera_source', default_value='csi'),

        Node(
            package='agri_rover_video',
            executable='video_streamer',
            name='video_streamer',
            namespace=['/rv', rover_id],
            parameters=[config, {
                'rtsp_port':     LaunchConfiguration('rtsp_port'),
                'camera_source': LaunchConfiguration('camera_source'),
            }],
            output='screen',
        ),
    ])
