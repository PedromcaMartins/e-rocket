"""
Example to launch a controller listener node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    controller_node = Node(
        package='one_degree_freedom',
        executable='controller',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'sitl.yaml']),
        ],
    )

    simulator_node = Node(
        package='one_degree_freedom',
        executable='simulator',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'sitl.yaml']),
        ],
    )

    mission_node = Node(
        package='one_degree_freedom',
        executable='mission',
        output='screen',
        shell=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'sitl.yaml']),
        ],
    )

    mock_flight_mode_node = Node(
        package='one_degree_freedom',
        executable='mock_flight_mode',
        output='screen',
        shell=True,
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('one_degree_freedom'), 'config', 'sitl.yaml']),
        ],
    )


    return LaunchDescription([
        controller_node,
        mission_node,
        simulator_node,
        mock_flight_mode_node,
    ])
