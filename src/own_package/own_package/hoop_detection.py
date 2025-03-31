import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

#kytiksen implementaatio:
"""def detect_hoops(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    circles = cv2.HoughCircles(
    blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
        param1=50, param2=30, minRadius=20, maxRadius=200
    )
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            cv2.circle(image, (circle[0], circle[1]), circle[2], (0, 255, 0), 3)
            cv2.circle(image, (circle[0], circle[1]), 2, (0, 0, 255), 3)
    return image"""
#Chattibroidi:
def detect_hoops(image):
    # Convert the image to HSV color space for better color filtering
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define the range for green color in HSV
    lower_green = np.array([40, 40, 40])  # Adjust these values as needed
    upper_green = np.array([80, 255, 255])
    
    # Create a mask for green color
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Apply the mask to the original image
    green_regions = cv2.bitwise_and(image, image, mask=mask)
    
    # Convert the masked image to grayscale
    gray = cv2.cvtColor(green_regions, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
        param1=50, param2=30, minRadius=20, maxRadius=200
    )
    
    # If circles are detected, draw them on the original image
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            # Draw the outer circle
            cv2.circle(image, (circle[0], circle[1]), circle[2], (0, 255, 0), 3)
            # Draw the center of the circle
            cv2.circle(image, (circle[0], circle[1]), 2, (0, 0, 255), 3)
    
    return image

class HoopDetector(Node):
    def __init__(self):
        super().__init__('hoop_detection')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, qos_profile)
        self.bridge = CvBridge()
        self.camera_info = None
        self.get_logger().info("Image subscriber started")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def image_callback(self, msg):
        #self.get_logger().info("we are in image_callback")
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        processed_frame = detect_hoops(frame)
        
        cv2.imshow('Hoop Detection', processed_frame)
        
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HoopDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
