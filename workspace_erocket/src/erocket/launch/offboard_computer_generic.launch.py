"""
Launch a offboard computer with pid controller.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

# Get the parent directory of the directory containing this launch file
launch_file_dir = os.path.dirname(os.path.realpath(__file__))
config_file = os.path.realpath(os.path.join(
    launch_file_dir,
    '../../../../../src/erocket/config/offboard.yaml'
))

def generate_launch_description():

    micro_ros_agent = ExecuteProcess(
        cmd=[[
            'micro-ros-agent udp4 --port 8888 -v '
        ]],
        shell=True
    )

    controller_generic_node = Node(
        package='erocket',
        executable='controller_generic',
        output='screen',
        shell=True,
        parameters=[config_file],
    )

    flight_mode_node = Node(
        package='erocket',
        executable='flight_mode',
        output='screen',
        shell=True,
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[config_file],
    )

    mission_node = Node(
        package='erocket',
        executable='mission',
        output='screen',
        shell=True,
        parameters=[config_file],
    )

    return LaunchDescription([
        #micro_ros_agent,
        controller_generic_node,
        flight_mode_node,
        mission_node,
    ])
