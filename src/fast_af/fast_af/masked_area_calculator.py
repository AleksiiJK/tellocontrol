import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

# Setting the QoS profile 
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class MaskedAreaCalculator(Node):
    def __init__(self):
        super().__init__('masked_area_calculator')
    
    # Create the subscriber for raw_image feed
        self.subscription = self.create_subscription(Image,'/image_raw',self.listener_callback,qos_profile)
    
    # Create the publisher
        self.publisher_ = self.create_publisher(Int32, 'masked_area', qos_profile)
        self.bridge = CvBridge()
    
    # Parameter for bounding box (x,y,width,heigth), the x and y coordinates represent how much area from the edge we want to use

        slice_x = 100
        slice_y = 80
        self.bbox = (slice_x,slice_y,960-2*slice_x,720-2*slice_y)

    def listener_callback(self,msg):

        # Convert the ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Mask and contour finding parameters
        lower_green = np.array([45, 70, 20])
        upper_green = np.array([85, 255, 200])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bounding box region:

        x, y, w, h = self.bbox
        height, width = mask.shape
        x_end = min(x + w, width)
        y_end = min(y + h, height)
        x_start = max(x, 0)
        y_start = max(y, 0)

        # Defining the region of interest (ROI)
        roi = mask[y_start:y_end, x_start:x_end]

        # Count non-zero pixels in ROI, then subtract them from the entire mask to receive pixels residing near the edges
        visible_area = int(cv2.countNonZero(roi))
        edge_pixels = int(int(cv2.countNonZero(mask))-visible_area)


        # Publish result
        area_msg = Int32(data=edge_pixels)
        self.publisher_.publish(area_msg)
        self.get_logger().info(f'Visible pixels in bbox: {edge_pixels}')
        #self.get_logger().info(f'Visible pixels: {int(cv2.countNonZero(mask))}')

        # Visualize the bounding box and masked frame
        frame_vis = frame.copy()
        cv2.rectangle(frame_vis, (x, y), (x_end, y_end), (0, 255, 0), 2)
        mask_vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.imshow('Frame with BBox', frame_vis)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = MaskedAreaCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()