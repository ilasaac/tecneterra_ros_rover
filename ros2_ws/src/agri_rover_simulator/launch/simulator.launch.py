from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('agri_rover_simulator'),
        'config',
        'simulator_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config,
            description='Path to simulator_params.yaml',
        ),

        Node(
            package='agri_rover_simulator',
            executable='simulator',
            name='simulator',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
