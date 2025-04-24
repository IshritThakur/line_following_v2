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



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from std_srvs.srv import Empty
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np

# # Line-following parameters
# LINEAR_SPEED = 0.05
# TURN_GAIN    = 1.0 / 150.0
# CROP_HEIGHT  = 100

# # Collision avoidance parameters
# COLLISION_AREA_THRESHOLD = 30000  # px² (ignore thin line edges)
# COLLISION_ZONE_WIDTH     = 100    # px around image center

# class LineFollower(Node):
#     def __init__(self):
#         super().__init__('line_following')
#         self.get_logger().info('Line follower + collision avoidance node started.')
#         self.bridge = CvBridge()
#         # Activation and collision flags
#         self.should_move = True
#         self.collision   = False

#         # ROS interfaces
#         self.create_service(Empty, 'start_line_follower', self.start_callback)
#         self.create_service(Empty, 'stop_line_follower',  self.stop_callback)
#         self.subscription = self.create_subscription(
#             Image, 'camera/image_raw', self.image_callback, 10)
#         self.publisher    = self.create_publisher(Twist, 'cmd_vel', 10)

#     def start_callback(self, request, response):
#         self.should_move = True
#         self.get_logger().info('Start service called, movement enabled.')
#         return response

#     def stop_callback(self, request, response):
#         self.should_move = False
#         self.get_logger().info('Stop service called, movement disabled.')
#         self.publisher.publish(Twist())
#         return response

#     def image_callback(self, msg):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#         except CvBridgeError as e:
#             self.get_logger().error(f'CvBridge Error: {e}')
#             return

#         # Debug view
#         dbg = cv2.resize(frame, (0, 0), fx=1.0, fy=1.5)
#         cv2.imshow('Camera Feed', dbg)

#         # Grayscale + crop bottom stripe
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         h, w = gray.shape
#         crop = gray[h - CROP_HEIGHT : h, 0 : w]

#         # Collision detection via edges in upper third of zone
#         edges = cv2.Canny(crop, 50, 150)
#         z0 = w//2 - COLLISION_ZONE_WIDTH//2
#         z1 = w//2 + COLLISION_ZONE_WIDTH//2
#         h_crop,_ = edges.shape
#         zone = edges[:h_crop//3, z0:z1]
#         cnts, _ = cv2.findContours(zone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         collision_detected = any(cv2.contourArea(c) > COLLISION_AREA_THRESHOLD for c in cnts)

#         if collision_detected:
#             # On first detection, log warning
#             if not self.collision:
#                 self.get_logger().warn('Collision detected! Stopping completely.')
#             self.collision = True
#             twist = Twist()
#             # Stop both linear and angular
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.publisher.publish(twist)
#             # Visualization
#             cv2.rectangle(frame, (z0, h - CROP_HEIGHT), (z1, h), (0,0,255), 2)
#             cv2.imshow('Collision Zone', cv2.resize(zone, (0, 0), fx=1.0, fy=2.0))
#             cv2.waitKey(5)
#             return
#         else:
#             if self.collision:
#                 self.get_logger().info('Obstacle cleared, resuming line-follow.')
#             self.collision = False

#         # Fallback to line-following only if movement enabled and no collision
#         _, thresh = cv2.threshold(crop, 200, 255, cv2.THRESH_BINARY_INV)
#         cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         twist = Twist()
#         if self.should_move and not self.collision and cnts:
#             c = max(cnts, key=cv2.contourArea)
#             M = cv2.moments(c)
#             if M['m00'] > 0:
#                 cx = int(M['m10']/M['m00'])
#                 err = cx - (w / 2)
#                 twist.linear.x  = LINEAR_SPEED
#                 twist.angular.z = -err * TURN_GAIN
#                 self.get_logger().info(f'Line detected. err={err:.1f}, angz={twist.angular.z:.2f}')
#                 cv2.circle(frame, (cx, h - CROP_HEIGHT//2), 5, (0,255,0), -1)
#         else:
#             twist.linear.x  = 0.0
#             twist.angular.z = 0.0

#         self.publisher.publish(twist)

#         # Visualization of mask and output
#         cv2.imshow('Line Mask', cv2.resize(thresh, (0, 0), fx=1.0, fy=2.0))
#         cv2.imshow('Output', frame)
#         cv2.waitKey(5)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LineFollower()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     cv2.destroyAllWindows()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
"""
Line-following controller with HYBRID obstacle detection:

    • dark-blob mask (large dark regions)
    • dilated Canny edges
    • width + area filters to ignore the track line

Opens three OpenCV debug windows per robot.
"""

import atexit
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

# ── tuning ──────────────────────────────────────────────────────────────
LINEAR_SPEED        = 0.05
TURN_GAIN           = 1.0 / 150.0
CROP_H              = 120      # analysed stripe height (px)

# obstacle detector
DARK_THR            = 100      # pixel value below ⇒ “dark”
MIN_AREA_OBS        = 25_000   # px² – bigger than line blobs
MIN_WIDTH_OBS       = 80       # px   – ignore thin contours
ROI_HEIGHT_FRACTION = 0.6      # 60 % of crop height → early detection
# ─────────────────────────────────────────────────────────────────────────


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.ns   = self.get_namespace().lstrip('/') or 'robot'
        self.cv   = CvBridge()
        self.run  = False      # enabled by /start_line_follower
        self.block = False     # true while obstacle in view

        # services
        self.create_service(Empty, 'start_line_follower', self.srv_start)
        self.create_service(Empty, 'stop_line_follower',  self.srv_stop)

        # topics
        self.sub = self.create_subscription(Image, 'camera/image_raw',
                                            self.cb_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # GUI
        cv2.namedWindow(f'{self.ns}-Cam',      cv2.WINDOW_NORMAL)
        cv2.namedWindow(f'{self.ns}-Mask',     cv2.WINDOW_NORMAL)
        cv2.namedWindow(f'{self.ns}-Obstacle', cv2.WINDOW_NORMAL)
        atexit.register(cv2.destroyAllWindows)

        self.get_logger().info('Hybrid line-follower ready (await /start_line_follower).')

    # ───────── services ─────────────────────────────────────────────────
    def srv_start(self, _req, _res):
        self.run = True
        self.get_logger().info('▶  movement ENABLED')
        return _res

    def srv_stop(self, _req, _res):
        self.run = False
        self.halt()
        self.get_logger().info('■  movement DISABLED')
        return _res

    # ───────── image callback ───────────────────────────────────────────
    def cb_image(self, msg: Image):
        try:
            img = self.cv.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge: {e}')
            return

        H, W, _ = img.shape
        crop = img[H - CROP_H:H, :]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

        # --- build dark-blob mask --------------------------------------
        roi_top = 0
        roi_bot = int(CROP_H * ROI_HEIGHT_FRACTION)
        roi = gray[roi_top:roi_bot, :]

        _, dark = cv2.threshold(roi, DARK_THR, 255, cv2.THRESH_BINARY_INV)
        dark = cv2.medianBlur(dark, 5)
        kernel = np.ones((5, 5), np.uint8)
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel, iterations=2)

        # --- build edge map --------------------------------------------
        edges = cv2.Canny(roi, 50, 150)
        edges = cv2.dilate(edges, None, iterations=2)

        # OR-combine both masks
        obs_mask = cv2.bitwise_or(dark, edges)

        # --- contour filtering -----------------------------------------
        obstacle = False
        cnts, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) < MIN_AREA_OBS:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w < MIN_WIDTH_OBS:
                continue
            obstacle = True
            cv2.rectangle(crop, (x, y), (x + w, y + h),
                          (0, 0, 255), 2)
            break

        if obstacle:
            if not self.block:
                self.get_logger().warn('Obstacle ahead – STOP')
            self.block = True
            self.halt()
        elif self.block:
            self.block = False
            self.get_logger().info('Path clear – resume')

        # --- line-following --------------------------------------------
        twist = Twist()
        if self.run and not self.block:
            _, mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    err = cx - (W / 2)
                    twist.linear.x  = LINEAR_SPEED
                    twist.angular.z = -err * TURN_GAIN
                    cv2.circle(crop, (cx, CROP_H // 2), 4, (0, 255, 0), -1)
            cv2.imshow(f'{self.ns}-Mask', mask)
        else:
            cv2.imshow(f'{self.ns}-Mask', np.zeros_like(gray))

        self.pub.publish(twist)

        # --- debug windows ---------------------------------------------
        cv2.imshow(f'{self.ns}-Cam', cv2.resize(img, (0, 0), fx=1.2, fy=1.2))
        cv2.imshow(f'{self.ns}-Obstacle',
                   cv2.resize(obs_mask, (0, 0), fx=2.0, fy=2.0))
        cv2.waitKey(1)

    # -------------------------------------------------------------------
    def halt(self):
        self.pub.publish(Twist())


# ───────── main ─────────────────────────────────────────────────────────
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
