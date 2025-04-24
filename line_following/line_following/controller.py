#!/usr/bin/env python3
"""
Hybrid line-following controller for namespaced robots.

Obstacle detector
-----------------
  • Dark-blob mask (pixels darker than DARK_THR)
  • Dilated Canny edges
  • Area & width filters (MIN_AREA_OBS / MIN_WIDTH_OBS)

Line detector
-------------
  • Uses only the bottom LINE_ROI_FRAC of the crop
  • Rejects contours taller than MAX_LINE_HEIGHT
  • Publishes cmd_vel to steer along the line centre

Debug windows (one set per robot namespace)
-------------------------------------------
  * <ns>-Cam       – annotated camera view
  * <ns>-Mask      – the single contour actually used for steering
  * <ns>-Obstacle  – combined obstacle mask
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
LINEAR_SPEED        = 0.10
TURN_GAIN           = 1.0 / 80.0
CROP_H              = 120      # height (px) of bottom stripe analysed

# obstacle detector
DARK_THR            = 100      # px value below → “dark”
MIN_AREA_OBS        = 25_000   # px²
MIN_WIDTH_OBS       = 80       # px
ROI_HEIGHT_FRACTION = 0.6      # % of crop height

# line detector
LINE_ROI_FRAC       = 0.30     # bottom 30 % of crop used for line
MAX_LINE_HEIGHT     = 25       # px – taller contours ignored
# ─────────────────────────────────────────────────────────────────────────


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.ns = self.get_namespace().lstrip('/') or 'robot'

        self.cv      = CvBridge()
        self.enabled = False
        self.block   = False

        # services
        self.create_service(Empty, 'start_line_follower', self.srv_start)
        self.create_service(Empty, 'stop_line_follower',  self.srv_stop)

        # topics
        self.sub = self.create_subscription(Image, 'camera/image_raw',
                                            self.cb_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # debug GUI
        cv2.namedWindow(f'{self.ns}-Cam',       cv2.WINDOW_NORMAL)
        cv2.namedWindow(f'{self.ns}-Mask',      cv2.WINDOW_NORMAL)
        cv2.namedWindow(f'{self.ns}-Obstacle',  cv2.WINDOW_NORMAL)
        atexit.register(cv2.destroyAllWindows)

        self.get_logger().info('Hybrid controller ready (call /start_line_follower to move).')

    # ────────────────── services ────────────────────────────────────────
    def srv_start(self, _req, _res):
        self.enabled = True
        self.get_logger().info('▶ movement ENABLED')
        return _res

    def srv_stop(self, _req, _res):
        self.enabled = False
        self._halt()
        self.get_logger().info('■ movement DISABLED')
        return _res

    # ────────────────── image callback ──────────────────────────────────
    def cb_image(self, msg: Image):
        try:
            img = self.cv.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge: {e}')
            return

        H, W, _ = img.shape
        crop = img[H - CROP_H:H, :]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

        # ---------- hybrid obstacle mask --------------------------------
        roi_bot = int(CROP_H * ROI_HEIGHT_FRACTION)
        roi     = gray[0:roi_bot, :]

        _, dark = cv2.threshold(roi, DARK_THR, 255, cv2.THRESH_BINARY_INV)
        dark = cv2.medianBlur(dark, 5)
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE,
                                np.ones((5, 5), np.uint8), iterations=2)

        edges = cv2.Canny(roi, 50, 150)
        edges = cv2.dilate(edges, None, iterations=2)

        obs_mask = cv2.bitwise_or(dark, edges)

        obstacle = False
        for c in cv2.findContours(obs_mask, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(c) < MIN_AREA_OBS:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w < MIN_WIDTH_OBS:
                continue
            obstacle = True
            cv2.rectangle(crop, (x, y), (x + w, y + h), (0, 0, 255), 2)
            break

        if obstacle and not self.block:
            self.block = True
            self.get_logger().warn('Obstacle ahead – STOP')
            self._halt()
        elif not obstacle and self.block:
            self.block = False
            self.get_logger().info('Path clear – RESUME')

        # ---------- line following --------------------------------------
        twist = Twist()
        vis_full = np.zeros_like(gray)  # for mask display

        if self.enabled and not self.block:
            roi_start = CROP_H - int(CROP_H * LINE_ROI_FRAC)
            line_roi  = gray[roi_start:CROP_H, :]

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
                # draw ONLY the accepted contour on the visual mask
                local_mask = np.zeros_like(mask)
                cv2.drawContours(local_mask, [best], -1, 255, cv2.FILLED)
                vis_full[roi_start:CROP_H, :] = local_mask

        cv2.imshow(f'{self.ns}-Mask', vis_full)
        self.pub.publish(twist)

        # ---------- GUI windows -----------------------------------------
        cv2.imshow(f'{self.ns}-Cam', cv2.resize(img, (0, 0), fx=1.2, fy=1.2))
        cv2.imshow(f'{self.ns}-Obstacle',
                   cv2.resize(obs_mask, (0, 0), fx=2.0, fy=2.0))
        cv2.waitKey(1)

    # -------------------------------------------------------------------
    def _halt(self):
        self.pub.publish(Twist())


# ────────────────── main ────────────────────────────────────────────────
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
