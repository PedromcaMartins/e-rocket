"""
Example to launch a actuator_servos listener node.
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

    actuator_servos_node = Node(
        package='px4_ros_demos',
        executable='actuator_servos',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        #micro_ros_agent,
        actuator_servos_node
    ])
