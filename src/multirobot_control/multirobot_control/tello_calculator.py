import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int32

# Setting the QoS profile
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
qos_profile_2 = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

class MaskedAreaCalculator(Node):
    def __init__(self):
        super().__init__('masked_area_calculator')
    
    # Create the subscriber for raw_image feed
        self.subscription = self.create_subscription(Image,'drone1/image_raw',self.listener_callback,qos_profile)
    
    # Create the publisher
        self.publisher_ = self.create_publisher(Int32, 'masked_area', qos_profile_2)
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
        area_percentage = int((visible_area/(self.bbox[-1]*self.bbox[-2]))*100)
        # Publish result
        area_msg = Int32(data=area_percentage)
        self.publisher_.publish(area_msg)
        # Visualize the bounding box and masked frame
        frame_vis = frame.copy()
        cv2.rectangle(frame_vis, (x, y), (x_end, y_end), (0, 255, 0), 2)
        cv2.imshow('Frame with BBox', frame_vis)
        cv2.waitKey(1)


def main(args = None):
     rclpy.init()
     node = MaskedAreaCalculator()
     try:
          rclpy.spin(node)
     except KeyboardInterrupt:
          pass
     finally:
          node.destroy_node

if __name__ == '__main__':
    main()