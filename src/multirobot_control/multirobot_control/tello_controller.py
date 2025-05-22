
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from tello_msgs.srv import TelloAction
from rclpy.executors import MultiThreadedExecutor

# Setting the QoS profile
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
qos_0 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)

""" Functions and utilities for the nodes: """
# Function to calculate the visible pixels and centroid of the desired color area
def average_green(image):
    # Convert to hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Mask and contour finding parameters
    lower_green = np.array([45, 70, 20])
    upper_green = np.array([85, 255, 200])
    mask = cv2.inRange(hsv, lower_green, upper_green)

    moments = cv2.moments(mask)
    visible_pixels = int(cv2.countNonZero(mask))

    if visible_pixels < 500:
        centroid = None
    elif moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        centroid = (cx, cy)
        cv2.circle(image, centroid, 5, (0, 0, 255), -1)
    else:
        centroid = None
    return centroid, image



"""A node that will recognize the desired color and publish it's centroid location as camera coordinates  """

class EdgeDetector(Node):
    def __init__(self):
        super().__init__('edge_detection')

        # Params for smoothing the centroid
        self.alpha = 0.5  # Smoothing factor (adjustable)
        self.prev_x = None
        self.prev_y = None
        self.max_change = 120  # Maximum allowed jump in pixels (adjust as needed)
        self.filter_treshold = 60 # How many iterations are needed for the centroid to be allowed to jump to another location
        # More parameter for centroid filtering
        self.previous_large_x = 0
        self.previous_large_y = 0
        self.previous_centroid = Point()
        self.previous_centroid.x,self.previous_centroid.y,self.previous_centroid.z = 0.0,0.0,0.0
        self.iterations = 0
        self.iteration_limit = 8
        self.mode = 1 # Default 1

        # ROS2 Publisher for centroid locations
        self.centroid_publisher = self.create_publisher(Point, '/centroid_locations', qos_profile)
        # ROS2 Subscribers for images and camera info
        self.subscription = self.create_subscription(Image, 'drone1/image_raw', self.image_callback, qos_profile)
        self.camera_info_sub = self.create_subscription(CameraInfo, 'drone1/camera_info', self.camera_info_callback, qos_profile)
        # Initial camera params and info message
        self.bridge = CvBridge()
        self.camera_info = None
        self.get_logger().info("Edge detection node started")

        # Callback functions
    def camera_info_callback(self, msg):
            self.camera_info = msg

    def image_callback(self, msg):
            # Convert the ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Initial centroid is None
            centroid = None
            # Check whether there is centroid:
            centroid, processed_frame = average_green(frame)
            # If a centroid was found, publish its location
            # Filter out the sudden changes and other noise
            if centroid: 
                new_x = float(centroid[0])
                new_y = float(centroid[1])

                if self.prev_x is None or self.prev_y is None:
                    # First-time initialization, this will prevent errors
                    self.prev_x = new_x
                    self.prev_y = new_y

                # Compute the change in position
                delta_x = abs(new_x - self.prev_x)
                delta_y = abs(new_y - self.prev_y)

                # Dealint with large changes, which might indicate noise disturbance
                if delta_x > self.max_change or delta_y > self.max_change:
                    
                # Reject if change is too large, unless the values repeat, which indicates that new relevant area has been detected
                    if abs(new_x-self.previous_large_x) < self.filter_treshold or abs(new_y-self.previous_large_y) < self.filter_treshold:
                            pass
                    else: 
                        print("Rejected sudden centroid jump:", new_x, new_y)                    
                        self.previous_large_x = new_x
                        self.previous_large_y = new_y # Store the values
                        new_x = self.prev_x  # Keep previous stable value
                        new_y = self.prev_y
                        
                # Apply exponential moving average filter
                filtered_x = self.alpha * new_x + (1 - self.alpha) * self.prev_x
                filtered_y = self.alpha * new_y + (1 - self.alpha) * self.prev_y

                # Update stored values
                self.prev_x = filtered_x
                self.prev_y = filtered_y
            
                # Publish filtered centroid
                centroid_msg = Point()
                centroid_msg.x = filtered_x
                centroid_msg.y = filtered_y
                centroid_msg.z = 0.0  # Can be set to any value as needed
                # Visualize the filtered centroid
                cv2.circle(processed_frame, (int(filtered_x), int(filtered_y)), 5, (0, 255, 0), -1)
        
                # If a centroid is lost, previous one will still be published for a set number of iterations
                self.previous_centroid = centroid_msg
                self.new_centroid_found = True
                self.iterations = 0
                self.centroid_publisher.publish(centroid_msg)

            if not centroid:
                centroid_msg = self.previous_centroid
                if self.iterations < self.iteration_limit:
                    self.centroid_publisher.publish(centroid_msg)
                    self.iterations += 1
                else:
                    pass

            # Show the processed image

            cv2.imshow('Edge and Area Detection', processed_frame)
            cv2.waitKey(1)


"""A node to calculate how much of the visible area in central bounding box belongs to the desired colour. 
This can be used to check whether it is time to land"""

class MaskedAreaCalculator(Node):
    def __init__(self):
        super().__init__('masked_area_calculator')
    
    # Create the subscriber for raw_image feed
        self.subscription = self.create_subscription(Image,'/image_raw',self.listener_callback,qos_profile)
    
    # Create the publisher
        self.publisher_ = self.create_publisher(Int32, 'masked_area', qos_0)
        self.bridge = CvBridge()

    # Parameter for bounding box (x,y,width,heigth), the x and y coordinates represent how much area from the edge we want to use

    # Bounding box for green:
        slice_x = 200
        slice_y = 100  #original 80
        self.bbox = (slice_x,slice_y,960-2*slice_x,720-2*slice_y)

        self.get_logger().info("Calculator node started")

    def listener_callback(self,msg):

        # Convert the ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Mask and contour finding parameters (Green)
        lower_green = np.array([45, 70, 0])
        upper_green = np.array([85, 255, 200])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bounding box region (Green):
        x, y, w, h = self.bbox
        height, width = mask.shape
        x_end = min(x + w, width)
        y_end = min(y + h, height)
        x_start = max(x, 0)
        y_start = max(y, 0)
        
        # Defining the region of interest (ROI)
        roi = mask[y_start:y_end, x_start:x_end]

        # Count non-zero pixels in ROI
        visible_area = int(cv2.countNonZero(roi))
        area_percentage = (visible_area/(self.bbox[-1]*self.bbox[-2]))*100
        # Publish result
        area_msg = Int32(data=area_percentage)
        self.publisher_.publish(area_msg)
        # Visualize the bounding box and masked frame
        frame_vis = frame.copy()
        cv2.rectangle(frame_vis, (x, y), (x_end, y_end), (0, 255, 0), 2)
        cv2.imshow('Frame with BBox', frame_vis)
        cv2.waitKey(1)



"""A controller for the drone"""
# Node to send velocity commands based on the located area of interest 
class CoordinateController(Node):

    def __init__(self):
        super().__init__('centroid_pid_sim')

        # Status attribute for multi-robot communication
        self.status = String(data="")
        self.percentage_threshold = 50
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
        # Timer to check if the drone has lost the centroid
        self.create_timer(1.0, self.check_centroid_visibility)


        # Create the subscriptions
        self.create_subscription(Point, '/centroid_locations', self.coordinate_callback, qos_profile)
        self.create_subscription(String,'/create3_status',self.status_callback,qos_profile)
        self.create_subscription(Int32,'/masked_area',self.masked_area_callback,qos_profile)
        # Create the publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.status_pub = self.create_publisher(String,'/drone1/status',10)
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
            message = String(data = "Drone following")
            self.status_pub.publish(message)
        else:
            pass
        
        # Check for area
    def masked_area_callback(self,msg):
        # Check if the target is close enough, if so stop the movement and publish status: 
        if self.percentage_treshold >= msg:
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


# Since many nodes are packaged into this script, there is a need to use the Multi-thread executor 
def main(args=None):
    rclpy.init(args=args)

    controller = CoordinateController()
    detector = EdgeDetector()
    calculator = MaskedAreaCalculator()

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(detector)
    executor.add_node(calculator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        detector.destroy_node()
        calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()







