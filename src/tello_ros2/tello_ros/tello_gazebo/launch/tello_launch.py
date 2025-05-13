"""Spawn Tello drone into an existing Gazebo simulation"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ns = 'drone1'
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    return LaunchDescription([
        # Spawn tello.urdf into running Gazebo
        Node(
            package='tello_gazebo',
            executable='inject_entity.py',
            output='screen',
            arguments=[urdf_path, '0', '0', '1', '1.57079632679']
        ),

        # Publish static transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ),

        # Joystick driver, generates /namespace/joy messages
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            namespace=ns
        ),

        # Joystick controller, generates /namespace/cmd_vel messages
        Node(
            package='tello_driver',
            executable='tello_joy_main',
            output='screen',
            namespace=ns
        ),
    ])

