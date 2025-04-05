import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Setting the QoS profile 
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

# Function to detect sharp edges, find areas, and return the centroid of a largest area
def detect_edges_and_boundaries(image):

    # Convert to hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Mask and contour finding parameters
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # For example, another color to mask could be added like this:

    """lower_brown = np.array([10, 10, 180])  # Adjust as needed
    upper_brown = np.array([30, 60, 255])  # Adjust as needed
    mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)

    # Combine both masks
    combined_mask = cv2.bitwise_or(mask_green, mask_brown)"""

    # Contour finding process
    gray = cv2.cvtColor(cv2.bitwise_and(image, image, mask=mask_green), cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1)
    edges = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Filter out the Largest area
    largest_contour = None
    largest_area = 0

    for contour in contours:
        if cv2.contourArea(contour) > 300:  # Filter out small areas
            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Keep track of the largest area
            if area > largest_area:
                largest_area = area
                largest_contour = contour

            # Draw the contour
            cv2.drawContours(image, [contour], 0, (0, 255, 0), 2)  # Green contour lines
            
            # Fill the area
            cv2.drawContours(image, [contour], 0, (0, 255, 255), -1)  # Yellow fill inside the area
    
    # If the largest contour exists, find its centroid   
    centroid = None
    if largest_contour is not None:
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroid = (cx, cy)
            cv2.circle(image, centroid, 5, (0, 0, 255), -1)  # Red dot at the centroid

    return image, centroid


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


        # ROS2 Publisher for centroid locations
        self.centroid_publisher = self.create_publisher(Point, 'centroid_locations', qos_profile)
        
        # ROS2 Subscriber for camera feed
        self.subscription = self.create_subscription(
            Image, 'drone1/image_raw', self.image_callback, qos_profile)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'drone1/camera_info', self.camera_info_callback, qos_profile)
        
        self.bridge = CvBridge()
        self.camera_info = None
        self.get_logger().info("Edge detection node started")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def image_callback(self, msg):
        # Convert the ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
        # Convert to HSV and create a mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        """Uncomment these to see what different masks are doing:"""
        # Show the masked frame
        #cv2.imshow('Masked Frame', mask)  # Black and white mask
        #cv2.imshow('Masked Frame (Colored)', cv2.bitwise_and(frame, frame, mask=mask))  # Mask applied to the original frame
        # Process the frame to detect edges and boundaries
        processed_frame, centroid = detect_edges_and_boundaries(frame)
        
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

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()