import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_following')
        self.get_logger().info('Line follower node started.')
        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # For debugging: show a resized camera feed.
        large_view = cv2.resize(cv_image, (0, 0), fx=1.0, fy=1.5)
        cv2.imshow("Camera Feed", large_view)

        # Process the image: convert to grayscale and crop the lower part.
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        crop_height = 100
        crop_img = gray[height - crop_height:height, 0:width]

        # Apply a binary threshold (inverting so that dark line appears white)
        ret, thresh = cv2.threshold(crop_img, 200, 255, cv2.THRESH_BINARY_INV)

        # For debugging: enlarge the binary mask window.
        mask_display = cv2.resize(thresh, (0, 0), fx=1.0, fy=2.0)
        cv2.imshow("Binary Mask", mask_display)
        cv2.waitKey(5)

        # Find contours in the threshold image.
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        if contours:
            # Choose the largest contour as the line.
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                error = cx - (width / 2)
                twist.linear.x = 0.05  # Reduced forward speed
                twist.angular.z = -float(error) / 150
                self.get_logger().info(f"Line detected. Error: {error}, Angular z: {twist.angular.z:.2f}")
            else:
                self.get_logger().warn("Zero moment, cannot compute centroid.")
        else:
            self.get_logger().info("Line not found, stopping robot.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

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