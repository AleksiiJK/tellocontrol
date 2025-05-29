
"""A simple program that will control the movement of the Create3 robot"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

pi = np.pi
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)


# A node that will publish velocity commands to the cmd_vel topic as well as subscribe to the /status topic: 

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        
        # Create the publisher for sending velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist,'/cmd_vel',5)
        # Create the subscriber that will subscribe to the communication channel 
        self.status_subscriber = self.create_subscription(String,'drone1/status',self.status_callback,qos_profile)
        # Create the publisher for publishing status messages
        self.status_publisher = self.create_publisher(String,'/create3_status',5)
        # The initial velocity will be none
        self.velocity = None
        self.start_time = None
        self.status = None

        # Parameters for the movement pattern
        # Linear movement
        self.forward_velocity = 1.0
        self.forward_duration = 4
        # Turning speed, goal is to turn 90 degrees in 3 seconds, the angular velocity is given in radians
        self.angular_velocity = pi/6   # 30 degrees/second = pi/6 radians/second 
        self.angular_duration = 3      # Turn for 3 seconds'
        self.movement_type = "Linear"  # Start with a linear move by default
        # Message type for the movement is Twist
        self.msg = Twist()

        # Timer to control the movement times
        self.timer = self.create_timer(0.01,self.cmd_vel_callback)
        # Initial movement phase starts from "stop"
        self.movement_phase = "forward"
        self.phase_start_time = self.get_clock().now().seconds_nanoseconds()[0]


        self.get_logger().info("Square movement node started")


    def status_callback(self,msg):
        self.status = msg.data
        if self.status in ["Drone is close"]:
            self.movement_phase = "stop"
            self.get_logger().info("Stopping due to drone proximity")
        elif self.status in ["Drone following", "Start"]:
            if self.movement_phase == "stop":
                self.movement_phase = "forward"
                self.phase_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.get_logger().info("Starting square movement")

    def cmd_vel_callback(self):
        # Take the time and set initial message
        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.phase_start_time
        msg = Twist()

        # If the movement phase is stopped, stop
        if self.movement_phase == "stop":
            msg = Twist()  # Zero velocities
            self.cmd_vel_publisher.publish(msg)
            status_msg = String()
            status_msg.data = "Create3 stopped"
            self.status_publisher.publish(status_msg)
            return
        
        # Othervise move
        status_msg = String()
        status_msg.data = "Create3 moving"
        self.status_publisher.publish(status_msg)

        # Movement phases control the square movement
        if self.movement_phase == "forward":
            if elapsed < self.forward_duration:
                msg.linear.x = self.forward_velocity
            else:
                self.movement_phase = "turn"
                self.phase_start_time = now
                msg = Twist()  # Stop before turning
        elif self.movement_phase == "turn":
            if elapsed < self.angular_duration:
                msg.angular.z = self.angular_velocity
            else:
                self.movement_phase = "forward"
                self.phase_start_time = now
                msg = Twist()  # Stop before moving forward
        # Publish the velocity command
        self.cmd_vel_publisher.publish(msg)

    


            
def main(args=None):
    rclpy.init(args=args)
    controller = SquareMover()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        