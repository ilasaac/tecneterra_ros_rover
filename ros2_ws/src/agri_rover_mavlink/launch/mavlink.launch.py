from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rover_id = LaunchConfiguration('rover_id', default='1')

    return LaunchDescription([
        DeclareLaunchArgument('rover_id',   default_value='1'),
        DeclareLaunchArgument('gqc_host',   default_value='192.168.1.255',
                              description='GQC UDP target (broadcast or direct IP)'),
        DeclareLaunchArgument('gqc_port',   default_value='14550'),
        DeclareLaunchArgument('bind_port',  default_value='14550',
                              description='RV1=14550, RV2=14551'),

        Node(
            package='agri_rover_mavlink',
            executable='mavlink_bridge',
            name='mavlink_bridge',
            namespace=['/rv', rover_id],
            parameters=[{
                'rover_id':       LaunchConfiguration('rover_id'),
                'gqc_host':       LaunchConfiguration('gqc_host'),
                'gqc_port':       LaunchConfiguration('gqc_port'),
                'bind_port':      LaunchConfiguration('bind_port'),
            }],
            output='screen',
        ),
    ])
