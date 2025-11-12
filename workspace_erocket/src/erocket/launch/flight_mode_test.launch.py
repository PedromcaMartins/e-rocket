"""
Example to launch a flight_mode test node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    micro_ros_agent = ExecuteProcess(
        cmd=[[
            'micro-ros-agent udp4 --port 8888 -v '
        ]],
        shell=True
    )

    flight_mode_node = Node(
        package='erocket',
        executable='flight_mode',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('erocket'), 'config', 'offboard.yaml']),
        ],
    )

    flight_mode_test_node = Node(
        package='erocket',
        executable='flight_mode_test',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('erocket'), 'config', 'offboard.yaml']),
        ],
    )

    return LaunchDescription([
        #micro_ros_agent,
        flight_mode_node,
        flight_mode_test_node,
    ])
