"""
Example to launch a px4_ros2_flight_mode listener node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    micro_ros_agent = ExecuteProcess(
        cmd=[[
            'micro-ros-agent udp4 --port 8888 -v '
        ]],
        shell=True
    )

    px4_ros2_flight_mode_node = Node(
        package='one_degree_freedom',
        executable='px4_ros2_flight_mode',
        output='screen',
        shell=True,
    )

    px4_ros2_flight_mode_test_node = Node(
        package='one_degree_freedom',
        executable='px4_ros2_flight_mode_test',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        #micro_ros_agent,
        px4_ros2_flight_mode_node,
        px4_ros2_flight_mode_test_node,
    ])
