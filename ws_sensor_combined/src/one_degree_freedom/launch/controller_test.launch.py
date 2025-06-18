"""
Example to launch a controller listener node.
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

    controller_node = Node(
        package='one_degree_freedom',
        executable='controller',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'offboard.yaml']),
        ],
    )

    controller_tester_node = Node(
        package='one_degree_freedom',
        executable='controller_test',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'offboard.yaml']),
        ],
    )

    controller_simulator_node = Node(
        package='one_degree_freedom',
        executable='controller_simulator',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'offboard.yaml']),
        ],
    )

    return LaunchDescription([
        #micro_ros_agent,
        controller_node,
        controller_tester_node,
        controller_simulator_node
    ])
