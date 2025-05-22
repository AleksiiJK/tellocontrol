# Build the packages. In case of WSL use option "--parallel-workers 1" to avoid failure due to lack of memory 
# This is not necessary on native Ubuntu, and slows down the build process
colcon build --parallel-workers 1 
# Source the build
source install/setup.bash
# Launch the world spawn a create3 robot. Disregard Rviz and the dock. Offset start y-position by 3 meters to give space for the drone 
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py use_rviz:='false' spawn_dock:='false' y:='3' &
# Small delay for stability
sleep 1.5 
# Add the tello-drone to the simulation 
ros2 launch tello_gazebo tello_launch.py &
# Another small delay
sleep 1.5
# Drone takeoff
ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
sleep 1
# Launch the controllers
# ros2 launch.. 