from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rover_id = LaunchConfiguration('rover_id', default='1')

    return LaunchDescription([
        DeclareLaunchArgument('rover_id', default_value='1'),

        Node(
            package='agri_rover_sensors',
            executable='sensor_node',
            name='sensor_node',
            namespace=['/rv', rover_id],
            parameters=[{'publish_rate': 1.0}],
            output='screen',
        ),
    ])
