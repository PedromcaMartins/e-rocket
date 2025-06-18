"""
Example to launch a px4_ros2_message_mapping listener node.
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

    px4_ros2_message_mapping_node = Node(
        package='one_degree_freedom',
        executable='px4_ros2_message_mapping',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'offboard.yaml']),
        ],
    )

    px4_ros2_message_mapping_test_node = Node(
        package='one_degree_freedom',
        executable='px4_ros2_message_mapping_test',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'offboard.yaml']),
        ],
    )

    return LaunchDescription([
        #micro_ros_agent,
        px4_ros2_message_mapping_node,
        px4_ros2_message_mapping_test_node,
    ])
