#!/usr/bin/env python3
# CONTROLLER V4.0 - Enhanced with start/stop services, track marks, and finalization
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# User-defined parameters
MIN_AREA = 500               # Minimum contour area to consider
MIN_AREA_TRACK = 5000        # Minimum area for track contour
LINEAR_SPEED = 0.2          # Forward speed when line is detected
KP = 1.5 / 100               # Proportional gain for turning
LOSS_FACTOR = 1.2            # Error compensation when line is lost
TIMER_PERIOD = 0.06          # Timer period in seconds
FINALIZATION_PERIOD = 4      # Time to move after second right mark (seconds)
MAX_ERROR = 30               # Max error to consider robot on straight line
# BGR color range for line detection
lower_bgr_values = np.array([31,  42,  53])
upper_bgr_values = np.array([255, 255, 255])

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_following')
        self.get_logger().info('Line follower node started.')

        # ROS interfaces
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge = CvBridge()

        # Internal state
        self.image_input = None
        self.error = 0
        self.just_seen_line = False
        self.just_seen_right_mark = False
        self.should_move = False
        self.right_mark_count = 0
        self.finalization_countdown = None

        # Services to start/stop the follower
        self.create_service(Empty, 'start_follower', self.start_follower_callback)
        self.create_service(Empty, 'stop_follower', self.stop_follower_callback)

        # Timer for processing loop
        self.create_timer(TIMER_PERIOD, self.timer_callback)

    def start_follower_callback(self, request, response):
        self.should_move = True
        self.right_mark_count = 0
        self.finalization_countdown = None
        return response

    def stop_follower_callback(self, request, response):
        self.should_move = False
        self.finalization_countdown = None
        return response

    def image_callback(self, msg):
        try:
            self.image_input = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def crop_size(self, height, width):
        # Crop bottom two-thirds vertically and middle half horizontally
        return (height // 3, height, width // 4, 3 * width // 4)

    def get_contour_data(self, mask, out_img, crop_w_start):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mark = {}
        line = {}

        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > MIN_AREA:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                if M['m00'] > MIN_AREA_TRACK:
                    # Track contour
                    line['x'] = crop_w_start + cx
                    line['y'] = cy
                    cv2.drawContours(out_img, contour, -1, (255,255,0), 1)
                    cv2.putText(out_img, str(int(M['m00'])), (cx, cy),
                                cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)
                else:
                    # Track mark contour (nearest to robot)
                    if (not mark) or (mark['y'] > cy):
                        mark['x'] = crop_w_start + cx
                        mark['y'] = cy
                        cv2.drawContours(out_img, contour, -1, (255,0,255), 1)
                        cv2.putText(out_img, str(int(M['m00'])), (cx, cy),
                                    cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)

        mark_side = None
        if mark and line:
            mark_side = 'right' if mark['x'] > line['x'] else 'left'
        return line, mark_side

    def timer_callback(self):
        # Ensure we have an image to process
        if self.image_input is None or not isinstance(self.image_input, np.ndarray):
            return

        image = self.image_input.copy()
        height, width, _ = image.shape
        h_start, h_stop, w_start, w_stop = self.crop_size(height, width)
        crop = image[h_start:h_stop, w_start:w_stop]

        # Color filtering
        mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

        # Analyze contours
        line, mark_side = self.get_contour_data(mask, image[h_start:h_stop, w_start:w_stop], w_start)

        cmd = Twist()
        if line:
            x_pos = line['x']
            self.error = x_pos - (width // 2)
            cmd.linear.x = LINEAR_SPEED
            self.just_seen_line = True
            cv2.circle(image, (x_pos, h_start + line['y']), 5, (0,255,0), 7)
        else:
            # Lost the line: spin in place to relocate
            if self.just_seen_line:
                self.just_seen_line = False
                self.error *= LOSS_FACTOR
            cmd.linear.x = 0.0

        # Handle track marks for finalization
        if mark_side:
            self.get_logger().info(f'Mark side: {mark_side}')
            if mark_side == 'right' and self.finalization_countdown is None \
               and abs(self.error) <= MAX_ERROR and not self.just_seen_right_mark:
                self.right_mark_count += 1
                if self.right_mark_count > 1:
                    self.finalization_countdown = int(FINALIZATION_PERIOD / TIMER_PERIOD) + 1
                    self.get_logger().info('Finalization process has begun!')
                self.just_seen_right_mark = True
        else:
            self.just_seen_right_mark = False

        # Proportional turning
        cmd.angular.z = -float(self.error) * KP
        self.get_logger().info(f'Error: {self.error} | Angular Z: {cmd.angular.z}')

        # Visualization
        cv2.rectangle(image, (w_start, h_start), (w_stop, h_stop), (0,0,255), 2)
        cv2.imshow('output', image)
        cv2.waitKey(5)

        # Handle finalization countdown
        if self.finalization_countdown is not None:
            if self.finalization_countdown > 0:
                self.finalization_countdown -= 1
            else:
                self.should_move = False

        # Publish velocity
        if self.should_move:
            self.publisher.publish(cmd)
        else:
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
        pass
    finally:
        # Stop robot on shutdown
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
