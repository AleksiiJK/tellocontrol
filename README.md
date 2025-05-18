# Hyvää multirobot systeemiä
*("Here's some good multirobot system" :D)*

## What kind of system are we building?
- Drone follows ground robot and lands on the ground robot when instructed
- Ground robot helps drone land as drone does not have a downward camera. 
## Findings
Simulation version of tello has the camera facing straigt, whereas the real camera is pointed slightly downwards
Forward camera is hard to use for landings.
The simulation allows for much better Tello control than is possible in real life.

## How to install the code

- Clone `src`
- Install dependencies
- In case of errors with rosdep, try rosdep update --include-eol-distros

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

## Useful commands for Create 3
Disabling Motion Control Safety

This will allow you to drive the robot (also in reverse) with teleop

ros2 param set /motion_control safety_override full

ros2 run teleop_twist_keyboard teleop_twist_keyboard 

### Drone commands  
`ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"`  
`ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"`  
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1`  
`ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'up 30'}"`  
`ros2 launch tello_driver teleop_launch.py`