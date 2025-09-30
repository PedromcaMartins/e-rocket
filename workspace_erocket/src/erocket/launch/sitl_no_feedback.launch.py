"""
Example to launch a controller listener node.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

# Get the parent directory of the directory containing this launch file
launch_file_dir = os.path.dirname(os.path.realpath(__file__))
config_file = os.path.realpath(os.path.join(
    launch_file_dir,
    '../../../../../src/erocket/config/sitl.yaml'
))

def generate_launch_description():

    controller_node = Node(
        package='erocket',
        executable='controller',
        output='screen',
        shell=True,
        parameters=[config_file],
    )

    simulator_node = Node(
        package='erocket',
        executable='simulator',
        output='screen',
        shell=True,
        parameters=[config_file],
    )

    mission_node = Node(
        package='erocket',
        executable='mission',
        output='screen',
        shell=True,
        parameters=[config_file],
    )

    mock_flight_mode_node = Node(
        package='erocket',
        executable='mock_flight_mode',
        output='screen',
        shell=True,
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[config_file],
    )


    return LaunchDescription([
        controller_node,
        mission_node,
        simulator_node,
        mock_flight_mode_node,
    ])
