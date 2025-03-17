#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('started')
        
        
        
        # Subscribe to the camera image topic (adjust if necessary)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        # Publisher to send velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert the ROS image to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Convert the image to grayscale and crop the lower part
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        crop_height = 100
        crop_img = gray[height - crop_height:height, 0:width]

        # Apply binary threshold to highlight the line (adjust threshold value if needed)
        _, thresh = cv2.threshold(crop_img, 200, 255, cv2.THRESH_BINARY_INV)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        if contours:
            # Choose the largest contour as the line
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                # Calculate the centroid of the contour
                cx = int(M['m10'] / M['m00'])
                # Determine error from the center of the image
                error = cx - (width / 2)
                twist.linear.x = 0.2  # Constant forward speed
                twist.angular.z = -float(error) / 100  # Proportional control for steering
                self.get_logger().info(f"Line detected. Error: {error}, Angular z: {twist.angular.z:.2f}")
            else:
                self.get_logger().warn("Zero moment, cannot compute centroid.")
        else:
            # If no line is found, stop the robot
            self.get_logger().info("Line not found, stopping robot.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the Twist message
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
