# Hyvää multirobot systeemiä
*("Here's some good multirobot system" :D)*

## What is happening here?
The code is developed for a multi-robot system consisting of a land and aerial robot. It replicates a case where the arerial robot needs to "dock" onto the land robot. Use cases for this include battery conservation as well as charging

 The communication logic is as follows: 

1. When the simulation is launched, the drone will take off
2. After user input, Create3 starts to move. It then messages to the drone to start honing in on the physical marker installed on it
3. When the drone is close enough, it will signal to create3 to stop
4. Once stopped, create3 will message to the drone that it is clear to land. The drone will then land onto the installed platform
5. Upon completing the landing, the drone will signal that create3 is free to move
6. Create3 will keep moving. 

*Add more description later*

## How to install the code

- Clone `src`
- Install dependencies

```bash
cd ~/create3_ws
sudo apt-get update
rosdep install --from-path src -yi
```

- Build package
- Source

## Launching simulation

### Simulator + Create 3
`ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py`

### Insert tello to simulator (simulator running)
`ros2 launch tello_gazebo tello_launch.py`

## Useful commands

### Drone commands  
`ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"`  
`ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"`  
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1`  
`ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'up 30'}"`  
`ros2 launch tello_driver teleop_launch.py`

