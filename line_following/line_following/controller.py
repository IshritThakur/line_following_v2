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


"""
A ROS2 node that follows a line with basic camera-based collision avoidance and full stop on detection.
Compatible with multiple robots via relative topics under namespaces.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Line-following parameters
LINEAR_SPEED = 0.05
TURN_GAIN    = 1.0 / 150.0
CROP_HEIGHT  = 100

# Collision avoidance parameters
COLLISION_AREA_THRESHOLD = 30000  # px² (ignore thin line edges)
COLLISION_ZONE_WIDTH     = 100    # px around image center

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_following')
        self.get_logger().info('Line follower + collision avoidance node started.')
        self.bridge = CvBridge()
        # Activation and collision flags
        self.should_move = True
        self.collision   = False

        # ROS interfaces
        self.create_service(Empty, 'start_line_follower', self.start_callback)
        self.create_service(Empty, 'stop_line_follower',  self.stop_callback)
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.publisher    = self.create_publisher(Twist, 'cmd_vel', 10)

    def start_callback(self, request, response):
        self.should_move = True
        self.get_logger().info('Start service called, movement enabled.')
        return response

    def stop_callback(self, request, response):
        self.should_move = False
        self.get_logger().info('Stop service called, movement disabled.')
        self.publisher.publish(Twist())
        return response

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

        # Collision detection via edges in upper third of zone
        edges = cv2.Canny(crop, 50, 150)
        z0 = w//2 - COLLISION_ZONE_WIDTH//2
        z1 = w//2 + COLLISION_ZONE_WIDTH//2
        h_crop,_ = edges.shape
        zone = edges[:h_crop//3, z0:z1]
        cnts, _ = cv2.findContours(zone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        collision_detected = any(cv2.contourArea(c) > COLLISION_AREA_THRESHOLD for c in cnts)

        if collision_detected:
            # On first detection, log warning
            if not self.collision:
                self.get_logger().warn('Collision detected! Stopping completely.')
            self.collision = True
            twist = Twist()
            # Stop both linear and angular
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            # Visualization
            cv2.rectangle(frame, (z0, h - CROP_HEIGHT), (z1, h), (0,0,255), 2)
            cv2.imshow('Collision Zone', cv2.resize(zone, (0, 0), fx=1.0, fy=2.0))
            cv2.waitKey(5)
            return
        else:
            if self.collision:
                self.get_logger().info('Obstacle cleared, resuming line-follow.')
            self.collision = False

        # Fallback to line-following only if movement enabled and no collision
        _, thresh = cv2.threshold(crop, 200, 255, cv2.THRESH_BINARY_INV)
        cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()
        if self.should_move and not self.collision and cnts:
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
            twist.linear.x  = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)

        # Visualization of mask and output
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
