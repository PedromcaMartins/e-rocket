"""
Example to launch a actuator_motors listener node.
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

    actuator_motors_node = Node(
        package='px4_ros_demos',
        executable='actuator_motors',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        #micro_ros_agent,
        actuator_motors_node
    ])
