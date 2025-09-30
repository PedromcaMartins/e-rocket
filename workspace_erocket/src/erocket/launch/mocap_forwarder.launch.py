"""
Example to launch mocap_interface's vrpn.launch.py and a mocap_forwarder listener node.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Include vrpn.launch.py from mocap_interface
    vrpn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mocap_interface'),
                'launch',
                'vrpn.launch.py'
            ])
        )
    )

    # Start the mocap_forwarder node
    mocap_forwarder_node = Node(
        package='erocket',
        executable='mocap_forwarder',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('erocket'),
                'config',
                'offboard.yaml'
            ])
        ]
    )

    return LaunchDescription([
        vrpn_launch,
        mocap_forwarder_node,
    ])
