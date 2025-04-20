# CONTROLLER THAT WORKS 

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np

# class LineFollower(Node):
#     def __init__(self):
#         super().__init__('line_following')
#         self.get_logger().info('Line follower node started.')
#         # Subscribe to the camera image topic
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.image_callback,
#             10)
#         # Publisher for velocity commands
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.bridge = CvBridge()

#     def image_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             self.get_logger().error(f"CvBridge Error: {e}")
#             return

#         # For debugging: show a resized camera feed.
#         large_view = cv2.resize(cv_image, (0, 0), fx=1.0, fy=1.5)
#         cv2.imshow("Camera Feed", large_view)

#         # Process the image: convert to grayscale and crop the lower part.
#         gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#         height, width = gray.shape
#         crop_height = 100
#         crop_img = gray[height - crop_height:height, 0:width]

#         # Apply a binary threshold (inverting so that dark line appears white)
#         ret, thresh = cv2.threshold(crop_img, 200, 255, cv2.THRESH_BINARY_INV)

#         # For debugging: enlarge the binary mask window.
#         mask_display = cv2.resize(thresh, (0, 0), fx=1.0, fy=2.0)
#         cv2.imshow("Binary Mask", mask_display)
#         cv2.waitKey(5)

#         # Find contours in the threshold image.
#         contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         twist = Twist()
#         if contours:
#             # Choose the largest contour as the line.
#             c = max(contours, key=cv2.contourArea)
#             M = cv2.moments(c)
#             if M['m00'] != 0:
#                 cx = int(M['m10'] / M['m00'])
#                 error = cx - (width / 2)
#                 twist.linear.x = 0.05  # Reduced forward speed
#                 twist.angular.z = -float(error) / 150
#                 self.get_logger().info(f"Line detected. Error: {error}, Angular z: {twist.angular.z:.2f}")
#             else:
#                 self.get_logger().warn("Zero moment, cannot compute centroid.")
#         else:
#             self.get_logger().info("Line not found, stopping robot.")
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0

#         self.publisher.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LineFollower()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# Controller 2.0 with collision logic

#!/usr/bin/env python3
"""
A ROS2 node that follows a line with basic camera-based collision avoidance.
Compatible with multiple robots via relative topics under namespaces.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Line-following parameters
LINEAR_SPEED = 0.05
TURN_GAIN    = 1.0 / 150.0
CROP_HEIGHT  = 100

# Collision avoidance parameters
COLLISION_AREA_THRESHOLD = 5000  # px²
COLLISION_ZONE_WIDTH     = 100   # px around image center

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_following')
        self.get_logger().info('Line follower + collision avoidance node started.')
        self.bridge = CvBridge()
        # Use relative topics for multi-robot namespace remapping
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.publisher    = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Debug view
        dbg = cv2.resize(frame, (0, 0), fx=1.0, fy=1.5)
        cv2.imshow('Camera Feed', dbg)

        # Grayscale + crop bottom stripe
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        crop = gray[h - CROP_HEIGHT : h, 0 : w]

        # Collision detection via edges in center zone
        edges = cv2.Canny(crop, 50, 150)
        z0 = w//2 - COLLISION_ZONE_WIDTH//2
        z1 = w//2 + COLLISION_ZONE_WIDTH//2
        zone = edges[:, z0:z1]
        cnts, _ = cv2.findContours(zone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) > COLLISION_AREA_THRESHOLD:
                twist = Twist()
                twist.linear.x  = 0.0
                twist.angular.z = 0.5
                self.get_logger().warn('Collision detected! Avoiding...')
                self.publisher.publish(twist)
                # visualize
                cv2.rectangle(frame, (z0, h - CROP_HEIGHT), (z1, h), (0,0,255), 2)
                cv2.imshow('Collision Zone', cv2.resize(zone, (0, 0), fx=1.0, fy=2.0))
                cv2.waitKey(5)
                return

        # Fallback to line-following
        _, thresh = cv2.threshold(crop, 200, 255, cv2.THRESH_BINARY_INV)
        cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                err = cx - (w / 2)
                twist.linear.x  = LINEAR_SPEED
                twist.angular.z = -err * TURN_GAIN
                self.get_logger().info(f'Line detected. err={err:.1f}, angz={twist.angular.z:.2f}')
                cv2.circle(frame, (cx, h - CROP_HEIGHT//2), 5, (0,255,0), -1)
        else:
            self.get_logger().info('Line lost. Stopping.')
            twist.linear.x  = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)

        # Visualization
        cv2.imshow('Line Mask', cv2.resize(thresh, (0, 0), fx=1.0, fy=2.0))
        cv2.imshow('Output', frame)
        cv2.waitKey(5)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
