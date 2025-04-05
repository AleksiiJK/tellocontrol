

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import numpy as np
import time

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Setting the QoS profile
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

class CoordinateController(Node):

    def __init__(self):
        super().__init__('coordinate_controller')

        # Camera dimensions define center points
        self.center_y = 720 / 2
        self.center_x = 960 / 2

        # Band to define whether forward move is acceptable
        self.Y_BAND = 40
        self.X_BAND = 40

        # Velocity parameters and recovery timeout
        self.FORWARD_VELOCITY = 1.5
        """Angular and z-velocity parameters are made obsolete by PID-control"""
        #self.ANGULAR_VELOCITY = 0.2 
        #self.Z_VELOCITY = 0.3   
        self.SEARCH_ROTATION_SPEED = 0.2
        self.SEARCH_TIMEOUT = 5.0

        # PID error variables 
        self.integral_ex = 0
        self.last_ex = 0

        self.integral_ey = 0
        self.last_ey = 0

        # Time variable for calculating the derivatives
        self.last_time = time.time()

        # Initializing the variables, creating the timer, subscriber and publisher
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_seen_time = time.time()

        self.create_subscription(Point, '/centroid_locations', self.coordinate_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.create_timer(1.0, self.check_centroid_visibility)

        # Main callback function 
    def coordinate_callback(self, msg):
        self.current_x, self.current_y = msg.x, msg.y
        self.get_logger().info(f'Received coordinates: X={self.current_x}, Y={self.current_y}')
        self.control_movement()

        # Separate PID-controller functions for x and y directions, start out with small gains
    def PID_x(self, ex):
        #Kp, Ki, Kd = 0, 0, 0
        Kp, Ki, Kd = 0.001, 0.003, 0

        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 1.0

        self.integral_ex += ex * dt

        derivative_ex = (ex - self.last_ex) / dt if dt > 0 else 0.0

        self.last_ex = ex
        self.last_time = current_time

        angular_z = Kp * ex + Ki * self.integral_ex + Kd * derivative_ex
        return -angular_z  # Negative to correct direction

    def PID_y(self, ey):
        Kp, Ki, Kd = 0.0006, 0.00015, 0
        #Kp, Ki, Kd = 0.000, 0.000, 0.0000

        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 1.0

        self.integral_ey += ey * dt

        derivative_ey = (ey - self.last_ey) / dt if dt > 0 else 0.0

        self.last_ey = ey
        self.last_time = current_time

        linear_z = Kp * ey + Ki * self.integral_ey + Kd * derivative_ey
        return -linear_z  # Negative to go down when target is above

        
        # Function to control the movement by applying the controller
    def control_movement(self):
        cmd_vel = Twist()

        # Calculate error
        ex = self.current_x - self.center_x
        ey = self.current_y - self.center_y

        # Apply the PID control to center the centroid
        cmd_vel.linear.z = self.PID_y(ey)
        cmd_vel.angular.z = self.PID_x(ex)

        # Move forward only if target is centered
        if abs(ex) < self.X_BAND and abs(ey) < self.Y_BAND:
            cmd_vel.linear.x = self.FORWARD_VELOCITY

        # Publish the velocity
        self.cmd_vel_pub.publish(cmd_vel)

        # Recovery function in case centroid is lost
    def check_centroid_visibility(self):
        if time.time() - self.last_seen_time > self.SEARCH_TIMEOUT:
            cmd_vel = Twist()
            cmd_vel.angular.z = self.SEARCH_ROTATION_SPEED
            self.get_logger().warn("Centroid lost! Searching...")
            self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    controller = CoordinateController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
