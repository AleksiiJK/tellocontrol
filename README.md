Täällä on sitten vain hyvät ja oikeat tiedostot\
("Here is only good and right files" :D)\
\
How to run the code:
- Clone src
- Build package
- Connect to drone
- Launch supporting nodes with command: ros2 launch fast_af detect.launch.py
- Start the drone racing: ./startup.sh
\
\
Useful commands:\
\
Simulator\
ros2 launch tello_gazebo simple_launch.py\
\
Tello driver\
ros2 launch tello_driver teleop_launch.py\
\
Drone commands\
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"\
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"\
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1\
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'up 30'}"\
ros2 launch tello_driver teleop_launch.py\