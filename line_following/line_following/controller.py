#!/usr/bin/env python3
"""
Hybrid line-following controller.

• Obstacle mask  = (dark blob  OR  dilated edges) on upper 60 % of crop
• Line mask      = bottom 40 % of crop; if none found → fall back to
                   whole crop
"""

import atexit, cv2, numpy as np, rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

# ── tuning ──────────────────────────────────────────────────────────────
LINEAR_SPEED   = 0.05
TURN_GAIN      = 1.0 / 150.0
CROP_H         = 120           # analysed stripe height (px)

# obstacle detector
DARK_THR       = 100
MIN_AREA_OBS   = 25_000        # px²
MIN_WIDTH_OBS  = 80            # px
ROI_FRAC_OBS   = 0.6           # top 60 % of crop

# line detector
LINE_ROI_FRAC  = 0.40          # bottom 40 % of crop
MAX_LINE_H     = 40            # px – taller blobs ignored
# ─────────────────────────────────────────────────────────────────────────


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.ns       = self.get_namespace().lstrip('/') or 'robot'
        self.bridge   = CvBridge()
        self.enabled  = False
        self.blocked  = False

        # services
        self.create_service(Empty, 'start_line_follower', self._srv_start)
        self.create_service(Empty, 'stop_line_follower',  self._srv_stop)

        # topics
        self.sub = self.create_subscription(Image, 'camera/image_raw',
                                            self._cb_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # debug windows
        for w in (f'{self.ns}-Cam', f'{self.ns}-Mask', f'{self.ns}-Obstacle'):
            cv2.namedWindow(w, cv2.WINDOW_NORMAL)
        atexit.register(cv2.destroyAllWindows)

        self.get_logger().info('Controller up; call /start_line_follower to move.')

    # ── services ────────────────────────────────────────────────────────
    def _srv_start(self, *_):
        self.enabled = True
        self.get_logger().info('▶ movement ENABLED')
        return Empty.Response()

    def _srv_stop(self, *_):
        self.enabled = False
        self._halt()
        self.get_logger().info('■ movement DISABLED')
        return Empty.Response()

    # ── image callback ─────────────────────────────────────────────────
    def _cb_image(self, msg: Image):
        try:    img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e));  return

        H, W, _ = img.shape
        crop  = img[H - CROP_H:H, :]
        gray  = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

        # ── obstacle mask (upper part) ─────────────────────────────────
        roi_end = int(CROP_H * ROI_FRAC_OBS)
        roi = gray[:roi_end, :]

        _, dark = cv2.threshold(roi, DARK_THR, 255, cv2.THRESH_BINARY_INV)
        dark = cv2.morphologyEx(cv2.medianBlur(dark,5),
                                cv2.MORPH_CLOSE, np.ones((5,5),np.uint8),2)
        edges = cv2.dilate(cv2.Canny(roi,50,150), None,2)
        obs_mask = cv2.bitwise_or(dark, edges)

        obstacle = False
        for c in cv2.findContours(obs_mask, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(c) < MIN_AREA_OBS: continue
            x,y,w,h = cv2.boundingRect(c)
            if w < MIN_WIDTH_OBS: continue
            obstacle = True
            cv2.rectangle(crop,(x,y),(x+w,y+h),(0,0,255),2)
            break

        if obstacle and not self.blocked:
            self.blocked=True; self._halt()
            self.get_logger().warn('Obstacle – STOP')
        elif not obstacle and self.blocked:
            self.blocked=False
            self.get_logger().info('Cleared – RESUME')

        # ── line following ─────────────────────────────────────────────
        twist = Twist()
        if self.enabled and not self.blocked:
            cx_found = self._find_line_cx(gray, W)
            if cx_found is not None:
                err = cx_found - W/2
                twist.linear.x  = LINEAR_SPEED
                twist.angular.z = -err * TURN_GAIN
                cv2.circle(crop,(cx_found,CROP_H-int(LINE_ROI_FRAC*CROP_H/2)),
                           4,(0,255,0),-1)
        self.pub.publish(twist)

        # windows
        cv2.imshow(f'{self.ns}-Cam', cv2.resize(img,(0,0),fx=1.2,fy=1.2))
        cv2.imshow(f'{self.ns}-Obstacle', cv2.resize(obs_mask,(0,0),fx=2,fy=2))
        cv2.waitKey(1)

    # ------------------------------------------------------------------
    def _find_line_cx(self, gray, W):
        """Return x-centre of line or None; updates Mask window."""
        roi_start = CROP_H - int(CROP_H * LINE_ROI_FRAC)
        strip = gray[roi_start:CROP_H, :]

        _, mask = cv2.threshold(strip, 200, 255, cv2.THRESH_BINARY_INV)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

        best = None
        for c in cnts:
            if cv2.boundingRect(c)[3] > MAX_LINE_H:  # too tall
                continue
            if best is None or cv2.contourArea(c) > cv2.contourArea(best):
                best = c

        if best is None:
            # Fallback: whole crop (legacy behaviour)
            _, mask_full = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
            cnts, _ = cv2.findContours(mask_full, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            best = max(cnts, key=cv2.contourArea, default=None)
            full_mask = mask_full
        else:
            full_mask = np.zeros_like(gray)
            full_mask[roi_start:CROP_H, :] = mask

        cv2.imshow(f'{self.ns}-Mask', full_mask)
        if best is not None:
            M = cv2.moments(best)
            return int(M['m10']/M['m00']) if M['m00'] else None
        return None

    # ------------------------------------------------------------------
    def _halt(self):
        self.pub.publish(Twist())


# ── main ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:               rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
