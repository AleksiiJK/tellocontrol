import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tello_msgs.srv import TelloAction

# Setting the QoS profile
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
qos_profile_2 = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

"""A controller for the drone"""
# Node to send velocity commands based on the located area of interest 
class CoordinateController(Node):

    def __init__(self):
        super().__init__('centroid_pid_sim')

        # Status attribute for multi-robot communication
        self.status = String(data="")
        self.percentage_threshold = 5
        # Camera dimensions define center points
        self.center_y = 720 / 2
        self.center_x = 960 / 2
        # Band to define whether forward move is acceptable
        self.Y_BAND = 40
        self.X_BAND = 40

        # Velocity parameters and recovery timeout
        self.FORWARD_VELOCITY = 0.1
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
        # Timer to check if the drone has lost the centroid


        # Create the subscriptions
        self.create_subscription(Point, '/centroid_locations', self.coordinate_callback, qos_profile)
        self.create_subscription(String,'/create3_status',self.status_callback,qos_profile)
        self.create_subscription(Int32,'/masked_area',self.masked_area_callback,qos_profile)
        # Create the publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', qos_profile_2)
        self.status_pub = self.create_publisher(String,'/drone1/status',qos_profile)
        self.create_timer(1.0, self.check_centroid_visibility)
        self.client = self.create_client(TelloAction, '/drone1/tello_action')

        self.get_logger().info("Controller node started")

        # Main callback functions 
        # Move the drone
    def coordinate_callback(self, msg):
        if self.status.data in ["Create3 moving"]: # Only move if create3 is in motion
            self.current_x, self.current_y = msg.x, msg.y
            self.get_logger().info(f'Received coordinates: X={self.current_x}, Y={self.current_y}')
            self.control_movement()
            self.last_seen_time = time.time()
            message = String(data = "Drone following")
            self.status_pub.publish(message)
        else:
            pass
        
        # Check for area
    def masked_area_callback(self,msg):
        # Check if the target is close enough, if so stop the movement and publish status: 
        if msg.data >= self.percentage_threshold:
            velocity_command = Twist()
            self.cmd_vel_pub.publish(velocity_command)
            message = String(data = "Drone is close")
            self.status_pub.publish(message)
        
        # Check the status
    def status_callback(self,msg):
        self.status = msg
        self.get_logger().info(msg.data)
        if msg.data == "Create3 stopped":
            self.request = TelloAction.Request()
            self.request.cmd = 'land'
            self.client.call_async(self.request)
            message = String(data="Drone has landed")
            self.status_pub.publish(message)


        
        # Other utility functions used in callbacks:

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
        Kp, Ki, Kd = 0.0006, 0.00001, 0
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
        print("Checking visbility")
        if self.status.data in ["Create3 moving"]:
            print(self.status.data)
            if time.time() - self.last_seen_time > self.SEARCH_TIMEOUT:
                cmd_vel = Twist()
                cmd_vel.angular.z = self.SEARCH_ROTATION_SPEED
                self.get_logger().warn("Centroid lost! Searching...")
                self.cmd_vel_pub.publish(cmd_vel)
        else: 
            pass

def main(args = None):
     rclpy.init()
     node = CoordinateController()
     try:
          rclpy.spin(node)
     except KeyboardInterrupt:
          pass
     finally:
          node.destroy_node

if __name__ == '__main__':
    main()