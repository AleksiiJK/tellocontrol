from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='tello_driver', executable='tello_joy_main', output='screen'),
        Node(package='tello_driver', executable='tello_driver_main', output='screen'),
        Node(
            package='fast_af',
            executable='edge_detection',
            name='edge_detection_node',
            output='screen',
        ),
        Node(
            package='fast_af',
            executable='masked_area_calculator',
            name='masked_area_calculator_node',
            output='screen',
        )     

    ])