# Source the terminal
source install/setup.bash
# Kill any leftover processes
pkill -f gazebo
pkill -f tello
# Launch the world spawn create3 with custom arguments
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py use_rviz:='false' spawn_dock:='false' y:='6' x:='3'

