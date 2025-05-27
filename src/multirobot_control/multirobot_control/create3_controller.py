
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

        self.get_logger().info("Square movement node started")

    def status_callback(self,msg):
        self.status = msg
        if self.status.data in ["Drone is close"]:
                message = Twist()
                self.cmd_vel_publisher.publish(message)
                status_msg = String()
                status_msg.data = "Create3 stopped"
                self.status_publisher.publish(status_msg)
        else:
            status_msg = String()
            status_msg.data = "Callback_check"
            self.status_publisher.publish(status_msg)

    

    # Main callback function that contains a very simple move sequence 
    def cmd_vel_callback(self):
        pass

        """
        if self.status != None:
            self.get_logger().info(self.status.data)
            if self.status.data in ["Drone following","Landed","Start"]:
                # Publish that create3 is moving:
                status_msg = String()
                status_msg.data = "Create3 moving"
                self.status_publisher.publish(status_msg)
                # Perform the movement sequence
                # 1. Move forward
                self.msg = Twist()
                start_time = time.time()
                while time.time() - start_time <= self.forward_duration:
                    self.msg.linear.x = self.forward_velocity
                    self.cmd_vel_publisher.publish(self.msg)
                    self.get_logger().info("Published velocity command")
                    time.sleep(0.05)  # Add sleep to avoid flooding the topic

                # Stop
                self.msg = Twist()
                self.cmd_vel_publisher.publish(self.msg)

                # 2. Rotate
                self.msg = Twist()
                start_time = time.time()
                while time.time() - start_time <= self.angular_duration:
                    self.msg.angular.z = self.angular_velocity
                    self.cmd_vel_publisher.publish(self.msg)
                    time.sleep(0.05)  # Add sleep here too

                # Stop
                self.msg = Twist()
                self.cmd_vel_publisher.publish(self.msg)

            elif self.status.data in ["Drone close"]:
                # Stop the movement
                self.msg = Twist()
                self.cmd_vel_publisher.publish(self.msg)
                status_msg = String()
                status_msg.data = "Create3 stopped"
                self.status_publisher.publish(status_msg)
                time.sleep(3) """




            
            
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
        