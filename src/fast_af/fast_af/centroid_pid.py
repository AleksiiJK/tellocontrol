import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
import numpy as np
import time
from tello_msgs.srv import TelloAction


from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Setting the QoS profile
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
qos_0 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)

class CoordinateController(Node):
    def __init__(self):
        super().__init__('centroid_pid')

        # Camera dimensions define center points
        self.center_y = 720 / 2 - 122 #125 stable, up go brr
        self.center_x = 960 / 2

        # Band to define whether forward move is acceptable
        self.Y_BAND = 40
        self.X_BAND = 40

        # Velocity parameters and recovery timeout
        self.FORWARD_VELOCITY = 0.15 #NOTE: anything over 1 will stop the drone
        self.fixed_spd = 0.4 #forward speed
           
        self.SEARCH_ROTATION_SPEED = -0.3
        self.SEARCH_RISING_SPEED = 0.04
        self.SEARCH_TIMEOUT = 3.0

        # PID error variables 
        self.integral_ex = 0
        self.last_ex = 0
        self.integral_ey = 0
        self.last_ey = 0

        # Parameters to check whether there is enough masked area near the edges, meaning that the drone can "Sprint" through the gate
        # Additional variables to enable overriding the velocities are also added
        self.percentage_treshold = 4
        self.override_duration = 3.2 # This is how long the drone will sprint 3->3.5
        self.override_active = False
        self.override_counter = 0
        self.mode = 1 # Default 1
        self.brute_force = True #turns right to find stop sign for this course

        # Parameter for red
        self.percentage_treshold_red = 6 #veri good determination basis forSEARC this value

        # Time variable for calculating the derivatives
        self.last_time = time.time()

        # Initializing the variables, creating the timer, subscriber and publisher
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_seen_time = time.time()

        self.create_timer(1.0, self.check_centroid_visibility)
        self.create_subscription(Point, '/centroid_locations', self.coordinate_callback, qos_profile)
        self.create_subscription(Int32, '/masked_area', self.masked_area_callback, qos_0)
        self.create_subscription(Int32, '/masked_area_red', self.masked_area_red_callback, qos_0)
        self.create_subscription(Int32, '/qr_min_dist', self.qr_sprint_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(Int32, '/mode', 10)

        self.client = self.create_client(TelloAction, '/tello_action')

        self.get_logger().info("Centroid pid node started")
        
        
        # Main callback functions 
    def coordinate_callback(self, msg):
        self.current_x, self.current_y = msg.x, msg.y
        #self.get_logger().info(f'Received coordinates: X={self.current_x}, Y={self.current_y}')
        self.last_seen_time = time.time()
        self.control_movement()

    def masked_area_callback(self, msg):
        self.area = msg.data
        self.percentage = (float((msg.data))/(720*960))*100
        #self.get_logger().info(f'The area percentage is {self.percentage}')
        self.check_for_area()

        # Set mode based on the override counter
        if self.override_counter < 3:
            self.mode = 1
        elif self.override_counter == 3:
            self.mode = 2
        else:
            self.mode = 3
        
        mode = Int32()
        mode.data = self.mode
        self.mode_pub.publish(mode)

    def masked_area_red_callback(self, msg):
        self.area_red = msg.data
        self.percentage_red = (float((msg.data))/(720*960))*100
        #self.get_logger().info(f'The area percentage is {self.percentage}')
        self.check_for_area_red()

        # Area check and override functions
    def check_for_area(self):
        if self.mode == 1:
            if self.percentage > self.percentage_treshold and self.override_active == False:
                self.get_logger().info("Locking velocity")
                self.override_active = True
                self.activate_override()
            else:
                pass   

    def check_for_area_red(self):
        if self.mode == 3:
            if self.brute_force == True:
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = -0.3
                self.cmd_vel_pub.publish(twist)
                self.brute_force = False
                time.sleep(1)

            if self.percentage_red > self.percentage_treshold_red:
                print("Landed") # Replace with landing service call
                self.get_logger().info(f'The area percentage is {self.percentage_red}')
                self.request = TelloAction.Request()
                self.request.cmd = 'land'
                self.client.call_async(self.request)
                       
    def activate_override(self):
        self.override_counter += 1
        self.get_logger().info(f"Sprint {self.override_counter}")
        twist = Twist()
        twist.linear.x = self.fixed_spd  # Fixed forward speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(self.override_duration)

        self.override_active = False
        self.get_logger().info("Override done. Control returned.")

    def qr_sprint_callback(self, msg):
        if self.mode == 2:
            self.get_logger().info(f"Vittu: {msg.data}")
            if msg.data > 400: #Used to be 380
                mode = Int32()
                mode.data = 1
                self.mode = 1
                self.mode_pub.publish(mode)
                self.activate_override()

        # Separate PID-controller functions for x and y directions, start out with small gains
    def PID_x(self, ex):
        #Kp, Ki, Kd = 0, 0, 0
        Kp, Ki, Kd = 0.001, 0.001, 0

        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 1.0

        self.integral_ex += ex * dt

        derivative_ex = (ex - self.last_ex) / dt if dt > 0 else 0.0

        self.last_ex = ex
        self.last_time = current_time

        angular_z = Kp * ex + Ki * self.integral_ex + Kd * derivative_ex
        return -angular_z  # Negative to correct direction

    def PID_y(self, ey):
        #Kp, Ki, Kd = 0.0006, 0.00015, 0
        Kp, Ki, Kd = 0.003, 0.0, 0.0

        if self.mode == 2:
            ey -= 19   #isompi siirtää ylöspäin, 17.5 on sellane et menee just ja just kahella koodilla.

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
        #limit velocities tbetween
        cmd_vel.linear.z = max(min(self.PID_y(ey),1.0),-1.0)
        cmd_vel.angular.z = max(min(self.PID_x(ex),1.0),-1.0)

        # Move forward only if target is centered
        if abs(ex) < self.X_BAND and abs(ey) < self.Y_BAND:
            #cmd_vel.linear.x = self.FORWARD_VELOCITY
            cmd_vel.linear.x = self.FORWARD_VELOCITY
        else: 
            cmd_vel.linear.x = 0.0
            #self.cmd_vel_pub.publish(forward)    
            #time.sleep(0.5)
        
        # Publish the velocity
        self.cmd_vel_pub.publish(cmd_vel)

        # Recovery function in case centroid is lost
    def check_centroid_visibility(self):
        #pass
        if time.time() - self.last_seen_time > self.SEARCH_TIMEOUT:
            cmd_vel = Twist()
            cmd_vel.angular.z = self.SEARCH_ROTATION_SPEED
            cmd_vel.linear.z = self.SEARCH_RISING_SPEED
            self.get_logger().warn("Centroid lost! Searching...")
            self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
