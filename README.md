Täällä on sitten vain hyvät ja oikeat tiedostot

Useful commands:

Simulator
ros2 launch tello_gazebo simple_launch.py

Tello driver
ros2 launch tello_driver teleop_launch.py

Drone commands

ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1