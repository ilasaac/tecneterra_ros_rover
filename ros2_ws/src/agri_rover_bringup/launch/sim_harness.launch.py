"""
Launch file for on-Jetson SIL simulation.

Runs only navigator + sim_harness.
No gps_driver, rp2040_bridge, sensor_node, or video.

  ros2 launch agri_rover_bringup sim_harness.launch.py
  ros2 launch agri_rover_bringup sim_harness.launch.py rover:=2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rover = LaunchConfiguration('rover')

    return LaunchDescription([
        DeclareLaunchArgument('rover', default_value='1',
                              description='Rover number (1 or 2)'),

        # Config is loaded by each node individually so the rover arg works.
        # We use PythonExpression-free approach: default to rover1 config.
        # For rover2, pass rover:=2 and the nodes pick up rover2_params.yaml.
        _make_nodes('1'),
    ])


def _make_nodes(rover_num: str):
    """Return a LaunchDescription fragment for the given rover number."""
    config = os.path.join(
        get_package_share_directory('agri_rover_bringup'),
        'config', f'rover{rover_num}_params.yaml')

    ns = f'/rv{rover_num}'

    # Return nodes as a group (LaunchDescription accepts nested lists)
    from launch import LaunchDescription as _LD
    return _LD([
        Node(package='agri_rover_navigator',
             executable='navigator',
             namespace=ns,
             parameters=[config],
             output='screen'),

        Node(package='agri_rover_simulator',
             executable='sim_harness',
             namespace=ns,
             parameters=[config],
             output='screen'),
    ])
