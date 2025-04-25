# #!/usr/bin/env python3
# """
# Binary-threshold line follower with FILLED-BLOB obstacle avoidance
# and debounce.

# Runs at LINEAR_SPEED = 0.10 m/s with TURN_GAIN = 1/90 rad/s per pixel error.
# """

# import atexit
# import cv2
# import numpy as np
# import rclpy
# from cv_bridge import CvBridge, CvBridgeError
# from geometry_msgs.msg import Twist
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_srvs.srv import Empty

# # ── tuning ──────────────────────────────────────────────────────────────
# LINEAR_SPEED        = 0.10        # <- faster
# TURN_GAIN           = 1.0 / 90.0  # <- tighter turns
# CROP_H              = 120

# # obstacle detector
# DARK_THR            = 100
# MIN_AREA_OBS        = 25_000
# MIN_WIDTH_OBS       = 80
# ROI_HEIGHT_FRACTION = 0.6

# # debounce
# OBS_FRAMES          = 3
# CLEAR_FRAMES        = 5

# # line detector
# LINE_ROI_FRAC       = 0.30
# MAX_LINE_HEIGHT     = 25
# # ─────────────────────────────────────────────────────────────────────────


# class LineFollower(Node):
#     def __init__(self):
#         super().__init__('line_follower')
#         self.ns = self.get_namespace().lstrip('/') or 'robot'

#         self.cv      = CvBridge()
#         self.enabled = False
#         self.block   = False
#         self.obs_count   = 0
#         self.clear_count = 0

#         # services
#         self.create_service(Empty, 'start_line_follower', self.srv_start)
#         self.create_service(Empty, 'stop_line_follower',  self.srv_stop)

#         # topics
#         self.sub = self.create_subscription(
#             Image, 'camera/image_raw', self.cb_image, 10)
#         self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

#         # GUI
#         cv2.namedWindow(f'{self.ns}-Cam',       cv2.WINDOW_NORMAL)
#         cv2.namedWindow(f'{self.ns}-Mask',      cv2.WINDOW_NORMAL)
#         cv2.namedWindow(f'{self.ns}-Obstacle',  cv2.WINDOW_NORMAL)
#         atexit.register(cv2.destroyAllWindows)

#         self.get_logger().info('Controller ready (faster speed, sharper turns).')

#     # ── service callbacks ───────────────────────────────────────────────
#     def srv_start(self, _rq, _rs):
#         self.enabled = True
#         self.get_logger().info('▶ movement ENABLED')
#         return _rs

#     def srv_stop(self, _rq, _rs):
#         self.enabled = False
#         self._halt()
#         self.get_logger().info('■ movement DISABLED')
#         return _rs

#     # ── main image callback ─────────────────────────────────────────────
#     def cb_image(self, msg: Image):
#         try:
#             img = self.cv.imgmsg_to_cv2(msg, 'bgr8')
#         except CvBridgeError as e:
#             self.get_logger().error(f'CvBridge: {e}')
#             return

#         H, W, _ = img.shape
#         crop = img[H - CROP_H:H, :]
#         gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

#         # ---------- filled-blob obstacle mask --------------------------
#         roi_bot = int(CROP_H * ROI_HEIGHT_FRACTION)
#         roi     = gray[:roi_bot, :]

#         _, dark = cv2.threshold(roi, DARK_THR, 255, cv2.THRESH_BINARY_INV)
#         dark = cv2.medianBlur(dark, 5)
#         dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE,
#                                 np.ones((5, 5), np.uint8), iterations=2)

#         edges = cv2.Canny(roi, 50, 150)
#         edges = cv2.dilate(edges, None, iterations=2)

#         rough = cv2.bitwise_or(dark, edges)

#         filled = np.zeros_like(rough)
#         cnts, _ = cv2.findContours(rough, cv2.RETR_EXTERNAL,
#                                    cv2.CHAIN_APPROX_SIMPLE)
#         for c in cnts:
#             cv2.drawContours(filled, [c], -1, 255, cv2.FILLED)

#         # obstacle?
#         obstacle = False
#         for c in cnts:
#             if cv2.contourArea(c) < MIN_AREA_OBS:
#                 continue
#             x, y, w, h = cv2.boundingRect(c)
#             if w < MIN_WIDTH_OBS:
#                 continue
#             obstacle = True
#             cv2.rectangle(crop, (x, y), (x + w, y + h), (0, 0, 255), 2)
#             break

#         # debounce
#         if obstacle:
#             self.obs_count += 1
#             self.clear_count = 0
#         else:
#             self.clear_count += 1
#             self.obs_count = 0

#         if self.obs_count >= OBS_FRAMES and not self.block:
#             self.block = True
#             self.get_logger().warn('Obstacle confirmed – STOP')
#             self._halt()

#         if self.block and self.clear_count >= CLEAR_FRAMES:
#             self.block = False
#             self.get_logger().info('Path clear – RESUME')

#         # ---------- line following -------------------------------------
#         twist = Twist()
#         vis_full = np.zeros_like(gray)

#         if self.enabled and not self.block:
#             roi_start = CROP_H - int(CROP_H * LINE_ROI_FRAC)
#             line_roi  = gray[roi_start:CROP_H, :]

#             _, mask = cv2.threshold(line_roi, 200, 255, cv2.THRESH_BINARY_INV)
#             line_cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
#                                             cv2.CHAIN_APPROX_SIMPLE)

#             best = None
#             for c in line_cnts:
#                 _, _, _, h = cv2.boundingRect(c)
#                 if h > MAX_LINE_HEIGHT:
#                     continue
#                 if best is None or cv2.contourArea(c) > cv2.contourArea(best):
#                     best = c

#             if best is not None:
#                 M = cv2.moments(best)
#                 if M['m00'] > 0:
#                     cx = int(M['m10'] / M['m00'])
#                     err = cx - (W / 2)
#                     twist.linear.x  = LINEAR_SPEED
#                     twist.angular.z = -err * TURN_GAIN
#                     cv2.circle(crop, (cx, CROP_H // 2),
#                                4, (0, 255, 0), -1)
#                 local_vis = np.zeros_like(mask)
#                 cv2.drawContours(local_vis, [best], -1, 255, cv2.FILLED)
#                 vis_full[roi_start:CROP_H, :] = local_vis

#         self.pub.publish(twist)

#         # ---------- display --------------------------------------------
#         cv2.imshow(f'{self.ns}-Mask', vis_full)
#         cv2.imshow(f'{self.ns}-Cam', cv2.resize(img, (0, 0), fx=1.2, fy=1.2))
#         cv2.imshow(f'{self.ns}-Obstacle',
#                    cv2.resize(filled, (0, 0), fx=2.0, fy=2.0))
#         cv2.waitKey(1)

#     # -------------------------------------------------------------------
#     def _halt(self):
#         self.pub.publish(Twist())


# # ── main ────────────────────────────────────────────────────────────────
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

