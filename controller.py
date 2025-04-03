

# Take data from centroid_location

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Setting the QoS profile
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

class CoordinateController(Node):

    def __init__(self):
        super().__init__('coordinate_controller')

        # Define control parameters directly in the script
        self.center_y = 720/2
        self.center_x = 960/2
        self.Y_BAND = 50  # Acceptable range for Y coordinate
        self.X_BAND = 20  # Acceptable range for X coordinate
        self.FORWARD_VELOCITY = 0.5  # Forward velocity (m/s)
        self.ANGULAR_VELOCITY = 0.5  # Angular velocity (rad/s)
        self.Z_VELOCITY = 0.15  # Velocity along the Z-axis
        self.MOVE_DURATION = 5.0  # Time to move forward after alignment (seconds)

        # Current coordinates
        self.current_x = 0.0
        self.current_y = 0.0

        # Create a subscriber to receive coordinates from a single topic
        self.create_subscription(Point, '/centroid_locations', self.coordinate_callback,qos_profile)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'drone1/cmd_vel', 10)

    def coordinate_callback(self, msg):
        """Callback to process incoming coordinates and control movement."""

        self.current_x, self.current_y = msg.x,msg.y
        self.get_logger().info(f'Received coordinates: X={self.current_x}, Y={self.current_y}')
        self.control_movement()

    def control_movement(self):
        """Controls the movement of the robot based on the received coordinates."""
        cmd_vel = Twist()

        # Step 1: Move along the Z-axis if Y is outside the band
        if self.current_y > self.center_y+self.Y_BAND:
            cmd_vel.linear.z = -self.Z_VELOCITY
            self.get_logger().info(f'Moving along Z-axis with velocity {cmd_vel.linear.z}')

        elif self.current_y < self.center_y-self.Y_BAND:
            cmd_vel.linear.z = self.Z_VELOCITY
            self.get_logger().info(f'Moving along Z-axis with velocity {cmd_vel.linear.z}')

        # Step 2: Rotate around the Z-axis if X is outside the band
        elif self.current_x > self.center_x+self.X_BAND:
            cmd_vel.angular.z = self.ANGULAR_VELOCITY
            self.get_logger().info(f'Rotating around Z-axis with angular velocity {cmd_vel.angular.z}')

        elif self.current_x < self.center_x-self.X_BAND:
            cmd_vel.angular.z = -self.ANGULAR_VELOCITY
            self.get_logger().info(f'Rotating around Z-axis with angular velocity {cmd_vel.angular.z}')


        # Step 3: Move forward when both X and Y are within range
        else:
            self.get_logger().info('Both coordinates in target range, moving forward.')
            cmd_vel.linear.x = self.FORWARD_VELOCITY
            self.get_logger().info(f'Moving forward with velocity {cmd_vel.linear.x}')

            # Schedule a stop after the movement duration
            self.create_timer(self.MOVE_DURATION, self.stop_movement)

        self.cmd_vel_pub.publish(cmd_vel)

    def stop_movement(self):
        """Stops the robot after the forward movement duration expires."""
        self.get_logger().info("Stopping movement.")
        stop_cmd = Twist()  # Publish zero velocity to stop movement
        self.cmd_vel_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = CoordinateController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
