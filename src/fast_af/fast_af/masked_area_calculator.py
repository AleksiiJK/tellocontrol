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
    
    # Create the publishers
        self.publisher_ = self.create_publisher(Int32, 'masked_area', qos_profile)
        self.publisher_red = self.create_publisher(Int32, 'masked_area_red',qos_profile)
        self.bridge = CvBridge()

    
    # Parameter for bounding box (x,y,width,heigth), the x and y coordinates represent how much area from the edge we want to use

    # Bounding box for green:
        slice_x = 100
        slice_y = 80
        self.bbox = (slice_x,slice_y,960-2*slice_x,720-2*slice_y)

    # Bounding box for red
        slice_x_red = 300
        slice_y_red = 250
        self.bbox_red = (slice_x_red,slice_y_red,960-2*slice_x_red,720-2*slice_y_red)

    

    def listener_callback(self,msg):

        # Convert the ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Mask and contour finding parameters (Green)
        lower_green = np.array([45, 70, 0])
        upper_green = np.array([85, 255, 200])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Mask and controur finding parameter (Red)
        # Lower red range
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        # Upper red range
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        # Combine the masks 
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)


        # Bounding box region (Green):
        x, y, w, h = self.bbox
        height, width = mask.shape
        x_end = min(x + w, width)
        y_end = min(y + h, height)
        x_start = max(x, 0)
        y_start = max(y, 0)

        # Bounding box region (Red)

        xr, yr, wr, hr = self.bbox_red 
        height, width = red_mask.shape
        x_end_red = min(xr + wr, width)
        y_end_red = min(yr + hr, height)
        x_start_red = max(xr, 0)
        y_start_red = max(yr, 0)

        # Defining the region of interest (ROI)
        roi = mask[y_start:y_end, x_start:x_end]
        roi_red = red_mask[y_start_red:y_end_red, x_start_red:x_end_red]

        # Count non-zero pixels in ROI, then subtract them from the entire mask to receive pixels residing near the edges
        visible_area = int(cv2.countNonZero(roi))
        edge_pixels = int(int(cv2.countNonZero(mask))-visible_area)

        # Calculate the red pixels at the middle bounging box:
        bbox_red_pixels = int(cv2.countNonZero(roi_red))

        # Publish result
        area_msg = Int32(data=edge_pixels)
        red_area_msg = Int32(data=bbox_red_pixels)
        self.publisher_.publish(area_msg)
        self.get_logger().info(f'Visible green pixels:{edge_pixels}, red {bbox_red_pixels} ')
        self.publisher_red.publish(red_area_msg)

        # Visualize the bounding box and masked frame
        frame_vis = frame.copy()
        cv2.rectangle(frame_vis, (x, y), (x_end, y_end), (0, 255, 0), 2)
        frame_vis_red = frame.copy()
        cv2.rectangle(frame_vis_red, (xr, yr), (x_end_red, y_end_red), (0, 255, 0), 2)
        #mask_vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.imshow('Frame with BBox', frame_vis)
        cv2.imshow('Frame with RedBBox', frame_vis_red)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = MaskedAreaCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()