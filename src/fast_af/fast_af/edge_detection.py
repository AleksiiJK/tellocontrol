import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Setting the QoS profile 
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)


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
        self.n_sprints = 0

        # Parameters for tag detection:
        # A predefined dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # 3. Create detector parameters
        self.parameters = cv2.aruco.DetectorParameters()


        # ROS2 Publisher for centroid locations
        self.centroid_publisher = self.create_publisher(Point, 'centroid_locations', qos_profile)
        
        # ROS2 Subscriber for camera feed and sprints
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, qos_profile)
        self.sprint_sub = self.create_subscription(Int32, '/sprints',self.sprint_callback, qos_profile)
        
        self.bridge = CvBridge()
        self.camera_info = None
        self.get_logger().info("Edge detection node started")

    def detectTags(self,frame):
        # Convert the video frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect the markers
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
        # Draw the detected markers on the frame
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return frame, corners

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def sprint_callback(self, msg):
        self.n_sprints = msg.data

    def qr_centroid(self, frame):
        tag_frame, tag_coords = self.detectTags(frame)
        centroid = 0
        return centroid, tag_frame

    def image_callback(self, msg):
        # Convert the ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        """A function to detect the fiducial markers"""
        

        # Sprint counting logic:
        if self.n_sprints < 5:
            # Green
            centroid, processed_frame = average_green(frame)
        elif self.n_sprints >= 5:
            # QR
            centroid, processed_frame = self.qr_centroid(frame)
            pass
        else:
            # Red
            pass


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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()