from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rover_id = LaunchConfiguration('rover_id', default='1')

    return LaunchDescription([
        DeclareLaunchArgument('rover_id',        default_value='1'),
        DeclareLaunchArgument('primary_port',    default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('secondary_port',  default_value='/dev/ttyUSB1'),

        Node(
            package='agri_rover_gps',
            executable='gps_driver',
            name='gps_driver',
            namespace=['/rv', rover_id],
            parameters=[{
                'primary_port':   LaunchConfiguration('primary_port'),
                'secondary_port': LaunchConfiguration('secondary_port'),
                'baud': 115200,
            }],
            output='screen',
        ),
    ])
