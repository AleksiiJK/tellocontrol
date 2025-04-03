import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Setting the QoS profile
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# Function to detect sharp edges, find areas, and return the largest centroid
def detect_edges_and_boundaries(image):

    # Mask and contour finding parameters

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    gray = cv2.cvtColor(cv2.bitwise_and(image, image, mask=mask), cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1)
    edges = cv2.Canny(blurred, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter out the Largest area

    largest_contour = None
    largest_area = 0

    for contour in contours:
        if cv2.contourArea(contour) > 100:  # Filter out small areas
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
        
        # ROS2 Publisher for centroid locations
        self.centroid_publisher = self.create_publisher(Point, 'centroid_locations', qos_profile)
        
        # ROS2 Subscriber for camera feed
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, qos_profile)
        
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

        # Show the masked frame
        cv2.imshow('Masked Frame', mask)  # Black and white mask
        cv2.imshow('Masked Frame (Colored)', cv2.bitwise_and(frame, frame, mask=mask))  # Mask applied to the original frame
        
        # Process the frame to detect edges and boundaries
        processed_frame, centroid = detect_edges_and_boundaries(frame)
        
        # If a centroid was found, publish its location
        if centroid:
            centroid_msg = Point()
            centroid_msg.x = float(centroid[0])
            centroid_msg.y = float(centroid[1])
            centroid_msg.z = 0.0  # Can be set to any value as needed
            self.centroid_publisher.publish(centroid_msg)

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