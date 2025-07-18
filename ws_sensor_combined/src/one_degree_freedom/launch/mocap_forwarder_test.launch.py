"""
Example to launch a mocap_forwarder listener node.
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

    mocap_forwarder_node = Node(
        package='one_degree_freedom',
        executable='mocap_forwarder',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'offboard.yaml']),
        ],
    )

    mocap_forwarder_test_node = Node(
        package='one_degree_freedom',
        executable='mocap_forwarder_test',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'offboard.yaml']),
        ],
    )

    return LaunchDescription([
        #micro_ros_agent,
        mocap_forwarder_node,
        mocap_forwarder_test_node,
    ])
