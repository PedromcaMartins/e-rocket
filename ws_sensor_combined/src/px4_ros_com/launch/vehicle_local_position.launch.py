"""
Example to launch a vehicle_local_position listener node.
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

    vehicle_local_position_node = Node(
        package='px4_ros_com',
        executable='vehicle_local_position',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        #micro_ros_agent,
        vehicle_local_position_node
    ])
