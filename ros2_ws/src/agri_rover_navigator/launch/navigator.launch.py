from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rover_id = LaunchConfiguration('rover_id', default='1')
    config   = os.path.join(
        get_package_share_directory('agri_rover_navigator'),
        'config', 'navigator_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('rover_id', default_value='1'),

        Node(
            package='agri_rover_navigator',
            executable='navigator',
            name='navigator',
            namespace=['/rv', rover_id],
            parameters=[config],
            output='screen',
        ),
    ])
