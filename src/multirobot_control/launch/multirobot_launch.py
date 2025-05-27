from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multirobot_control',
            executable='create3_controller',
            name='node_one',
            output='screen'
        ),
        Node(
            package='multirobot_control',
            executable='tello_calculator',
            name='masked_area_calculator',
            output='screen'
        ),
        Node(
            package='multirobot_control',
            executable='tello_detection',
            name='edge_detection',
            output='screen'
        ),
        Node(
            package='multirobot_control',
            executable='tello_controller_new',
            name='centroid_pid_sim',
            output='screen'
        )

    ])
