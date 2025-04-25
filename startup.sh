#!/usr/bin/env bash

ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
sleep 5
#!ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'up 30'}" rikkoi loppupaskan
#!sleep 3
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.2
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
sleep 1
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
ros2 run fast_af centroid_pid

