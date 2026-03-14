from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rover_id = LaunchConfiguration('rover_id', default='1')

    return LaunchDescription([
        DeclareLaunchArgument('rover_id', default_value='1',
                              description='Rover ID (1=master, 2=slave)'),
        DeclareLaunchArgument('uart_port', default_value='/dev/ttyACM0',
                              description='RP2040 USB serial port'),

        Node(
            package='agri_rover_rp2040',
            executable='rp2040_bridge',
            name='rp2040_bridge',
            namespace=['/rv', rover_id],
            parameters=[{
                'uart_port': LaunchConfiguration('uart_port'),
                'uart_baud': 115200,
                'heartbeat_interval': 0.2,
            }],
            output='screen',
        ),
    ])
