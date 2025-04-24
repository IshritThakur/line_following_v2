#!/usr/bin/env python3
"""
Hybrid line-following controller for multi-robot Gazebo demos.

• Obstacle detection = dark-blob mask  ∨  dilated Canny edges
• Line detection     = bottom LINE_ROI_FRAC of frame, contours shorter
                       than MAX_LINE_HEIGHT
• Three OpenCV debug windows per robot

Tweak the constants in the “Tuning” section to suit your lighting/model.
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

# ── Tuning ──────────────────────────────────────────────────────────────
LINEAR_SPEED        = 0.05
TURN_GAIN           = 1.0 / 150.0
CROP_H              = 120      # height of analysed stripe (px)

# obstacle detector
DARK_THR            = 100      # value below → “dark”
MIN_AREA_OBS        = 25_000   # px²
MIN_WIDTH_OBS       = 80       # px
ROI_HEIGHT_FRACTION = 0.6      # analyse top 60 % of crop

# line detector
LINE_ROI_FRAC       = 0.30     # bottom 30 % of crop
MAX_LINE_HEIGHT     = 25       # px – taller blobs are ignored
# ─────────────────────────────────────────────────────────────────────────


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.ns     = self.get_namespace().lstrip('/') or 'robot'
        self.bridge = CvBridge()
        self.enabled = False     # becomes True via start service
        self.blocked = False     # True while obstacle ahead

        # Services
        self.create_service(Empty, 'start_line_follower', self._srv_start)
        self.create_service(Empty, 'stop_line_follower',  self._srv_stop)

        # Topics
        self.sub = self.create_subscription(Image, 'camera/image_raw',
                                            self._cb_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # GUI windows
        cv2.namedWindow(f'{self.ns}-Cam',       cv2.WINDOW_NORMAL)
        cv2.namedWindow(f'{self.ns}-Mask',      cv2.WINDOW_NORMAL)
        cv2.namedWindow(f'{self.ns}-Obstacle',  cv2.WINDOW_NORMAL)
        atexit.register(cv2.destroyAllWindows)

        self.get_logger().info('Controller ready – call /start_line_follower to begin.')

    # ── Service callbacks ───────────────────────────────────────────────
    def _srv_start(self, _rq, _rs):
        self.enabled = True
        self.get_logger().info('▶ movement ENABLED')
        return _rs

    def _srv_stop(self, _rq, _rs):
        self.enabled = False
        self._halt()
        self.get_logger().info('■ movement DISABLED')
        return _rs

    # ── Image callback ──────────────────────────────────────────────────
    def _cb_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge: {e}')
            return

        H, W, _ = img.shape
        crop = img[H - CROP_H : H, :]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

        # ── HYBRID OBSTACLE MASK ───────────────────────────────────────
        roi_end = int(CROP_H * ROI_HEIGHT_FRACTION)
        roi = gray[0 : roi_end, :]

        # dark-blob part
        _, dark = cv2.threshold(roi, DARK_THR, 255, cv2.THRESH_BINARY_INV)
        dark = cv2.medianBlur(dark, 5)
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE,
                                np.ones((5, 5), np.uint8), iterations=2)

        # edge part
        edges = cv2.Canny(roi, 50, 150)
        edges = cv2.dilate(edges, None, iterations=2)

        obs_mask = cv2.bitwise_or(dark, edges)

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
            cv2.rectangle(crop, (x, y), (x + w, y + h), (0, 0, 255), 2)
            break

        if obstacle and not self.blocked:
            self.blocked = True
            self.get_logger().warn('Obstacle detected – STOP')
            self._halt()
        elif not obstacle and self.blocked:
            self.blocked = False
            self.get_logger().info('Path clear – RESUME')

        # ── LINE FOLLOWING (bottom strip) ───────────────────────────────
        twist = Twist()
        if self.enabled and not self.blocked:
            roi_start = CROP_H - int(CROP_H * LINE_ROI_FRAC)
            line_roi  = gray[roi_start : CROP_H, :]

            _, mask = cv2.threshold(line_roi, 200, 255, cv2.THRESH_BINARY_INV)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

            best = None
            for c in cnts:
                x, y, w, h = cv2.boundingRect(c)
                if h > MAX_LINE_HEIGHT:
                    continue
                if best is None or cv2.contourArea(c) > cv2.contourArea(best):
                    best = c

            if best is not None:
                M = cv2.moments(best)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    err = cx - (W / 2)
                    twist.linear.x  = LINEAR_SPEED
                    twist.angular.z = -err * TURN_GAIN
                    cv2.circle(crop,
                               (cx, CROP_H - int(LINE_ROI_FRAC * CROP_H / 2)),
                               4, (0, 255, 0), -1)

            # visual mask in full-crop coordinates
            full_mask = np.zeros_like(gray)
            full_mask[roi_start : CROP_H, :] = mask
            cv2.imshow(f'{self.ns}-Mask', full_mask)
        else:
            cv2.imshow(f'{self.ns}-Mask', np.zeros_like(gray))

        self.pub.publish(twist)

        # ── Debug windows ───────────────────────────────────────────────
        cv2.imshow(f'{self.ns}-Cam',
                   cv2.resize(img, (0, 0), fx=1.2, fy=1.2))
        cv2.imshow(f'{self.ns}-Obstacle',
                   cv2.resize(obs_mask, (0, 0), fx=2.0, fy=2.0))
        cv2.waitKey(1)

    # -------------------------------------------------------------------
    def _halt(self):
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
