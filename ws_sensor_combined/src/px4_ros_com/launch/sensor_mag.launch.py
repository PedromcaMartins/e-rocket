"""
Example to launch a sensor_mag listener node.
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

    sensor_mag_node = Node(
        package='px4_ros_com',
        executable='sensor_mag',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        #micro_ros_agent,
        sensor_mag_node
    ])
