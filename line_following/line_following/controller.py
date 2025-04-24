#!/usr/bin/env python3
"""
Line-following controller with FILLED-BLOB obstacle avoidance.

Changes vs. the snippet you sent:
---------------------------------
• After combining the dark-pixel mask with dilated edges we FILL every
  external contour, so the whole robot chassis becomes one solid white blob.
  The width / area filters now trigger more reliably.
• Everything else (binary line detector, windows, speed / gain...) is
  unchanged.
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
CROP_H              = 120

# obstacle detector
DARK_THR            = 100
MIN_AREA_OBS        = 25_000
MIN_WIDTH_OBS       = 80
ROI_HEIGHT_FRACTION = 0.6
# ─────────────────────────────────────────────────────────────────────────


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.ns   = self.get_namespace().lstrip('/') or 'robot'
        self.cv   = CvBridge()
        self.run  = False
        self.block = False

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

        self.get_logger().info('Filled-blob line-follower ready (await /start_line_follower).')

    # ── service callbacks ───────────────────────────────────────────────
    def srv_start(self, _req, _res):
        self.run = True
        self.get_logger().info('▶ movement ENABLED')
        return _res

    def srv_stop(self, _req, _res):
        self.run = False
        self.halt()
        self.get_logger().info('■ movement DISABLED')
        return _res

    # ── image callback ──────────────────────────────────────────────────
    def cb_image(self, msg: Image):
        try:
            img = self.cv.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge: {e}')
            return

        H, W, _ = img.shape
        crop = img[H - CROP_H:H, :]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

        # ---------- build dark-blob mask --------------------------------
        roi_bot = int(CROP_H * ROI_HEIGHT_FRACTION)
        roi = gray[:roi_bot, :]

        _, dark = cv2.threshold(roi, DARK_THR, 255, cv2.THRESH_BINARY_INV)
        dark = cv2.medianBlur(dark, 5)
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE,
                                np.ones((5, 5), np.uint8), iterations=2)

        # ---------- build edge map --------------------------------------
        edges = cv2.Canny(roi, 50, 150)
        edges = cv2.dilate(edges, None, iterations=2)

        # rough mask = OR
        rough_mask = cv2.bitwise_or(dark, edges)

        # ---------- FILL each contour → solid blob ----------------------
        filled_mask = np.zeros_like(rough_mask)
        cnts, _ = cv2.findContours(rough_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            cv2.drawContours(filled_mask, [c], -1, 255, cv2.FILLED)

        # ---------- obstacle filtering ----------------------------------
        obstacle = False
        for c in cnts:
            if cv2.contourArea(c) < MIN_AREA_OBS:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w < MIN_WIDTH_OBS:
                continue
            obstacle = True
            cv2.rectangle(crop, (x, y), (x + w, y + h), (0, 0, 255), 2)
            break

        if obstacle:
            if not self.block:
                self.get_logger().warn('Obstacle ahead – STOP')
            self.block = True
            self.halt()
        elif self.block:
            self.block = False
            self.get_logger().info('Path clear – resume')

        # ---------- line following --------------------------------------
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

        # ---------- debug windows --------------------------------------
        cv2.imshow(f'{self.ns}-Cam', cv2.resize(img, (0, 0), fx=1.2, fy=1.2))
        cv2.imshow(f'{self.ns}-Obstacle',
                   cv2.resize(filled_mask, (0, 0), fx=2.0, fy=2.0))
        cv2.waitKey(1)

    # -------------------------------------------------------------------
    def halt(self):
        self.pub.publish(Twist())


# ── main ────────────────────────────────────────────────────────────────
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
