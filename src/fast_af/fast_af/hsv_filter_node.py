# Used only for tuning the HSV parameters

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange


# Setting the QoS profile 
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)

def make_slider_descriptor(min_val, max_val, step=1, description=""):
    int_range = IntegerRange(
        from_value=min_val,
        to_value=max_val,
        step=step
    )
    return ParameterDescriptor(
        description=description,
        integer_range=[int_range]
    )

class HSVFilterNode(Node):
    def __init__(self):
        super().__init__('hsv_filter_node')

        # Parameters with default HSV thresholds
        self.declare_parameter(
            "h_min", 0,
            descriptor=make_slider_descriptor(0, 179, 1, "Minimum Hue")
        )
        self.declare_parameter(
            "h_max", 179,
            descriptor=make_slider_descriptor(0, 179, 1, "Maximum Hue")
        )
        self.declare_parameter(
            "s_min", 0,
            descriptor=make_slider_descriptor(0, 255, 1, "Minimum Saturation")
        )
        self.declare_parameter(
            "s_max", 255,
            descriptor=make_slider_descriptor(0, 255, 1, "Maximum Saturation")
        )
        self.declare_parameter(
            "v_min", 0,
            descriptor=make_slider_descriptor(0, 255, 1, "Minimum Value")
        )
        self.declare_parameter(
            "v_max", 255,
            descriptor=make_slider_descriptor(0, 255, 1, "Maximum Value")
        )

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile)

        self.get_logger().info("HSV Filter Node Started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get dynamic parameters
        h_min = self.get_parameter("h_min").value
        h_max = self.get_parameter("h_max").value
        s_min = self.get_parameter("s_min").value
        s_max = self.get_parameter("s_max").value
        v_min = self.get_parameter("v_min").value
        v_max = self.get_parameter("v_max").value

        # Filter HSV
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Show images
        cv2.imshow("Original", frame)
        cv2.imshow("mask", mask)
        cv2.imshow("Filtered", result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HSVFilterNode()
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
