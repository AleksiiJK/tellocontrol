# Source the terminal
source install/setup.bash
# Spawn the tello drone model
ros2 launch tello_gazebo tello_launch.py &
# Make the drone take off
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"