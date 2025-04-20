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
A ROS2 node with minimal line-following plus collision recovery logic.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum

# ─── Parameters ───────────────────────────────────────────────────────────────
MIN_AREA = 500
MIN_AREA_TRACK = 5000
LINEAR_SPEED = 0.2
KP = 1.5 / 100
LOSS_FACTOR = 1.2
TIMER_PERIOD = 0.06
FINALIZATION_PERIOD = 4
MAX_ERROR = 30

lower_bgr_values = np.array([31, 42, 53])
upper_bgr_values = np.array([255, 255, 255])

# Collision detection params
COLLISION_AREA_THRESHOLD = 8000   # min pixels for obstacle
COLLISION_ZONE_WIDTH    = 100     # px around center

# Recovery timings (seconds)
STOP_WAIT   = 1.0
BACKUP_TIME = 0.8
TURN_TIME   = 1.2

# Convert to nanoseconds for comparisons
STOP_WAIT_NS   = int(STOP_WAIT   * 1e9)
BACKUP_TIME_NS = int(BACKUP_TIME * 1e9)
TURN_TIME_NS   = int(TURN_TIME   * 1e9)

# Crop helper
def crop_size(height, width):
    return (height // 3, height, width // 4, 3 * width // 4)

# ─── State Machine ────────────────────────────────────────────────────────────
class State(Enum):
    FOLLOW  = 0
    STOPPED = 1
    BACKUP  = 2
    TURN    = 3

# ─── Globals ──────────────────────────────────────────────────────────────────
state = State.FOLLOW
state_start_ns = 0
image_input = None
error = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None
bridge = CvBridge()
node = None
publisher = None

# ─── Services ────────────────────────────────────────────────────────────────
def start_line_follower_callback(request, response):
    global should_move, right_mark_count, finalization_countdown, state, state_start_ns
    should_move = True
    right_mark_count = 0
    finalization_countdown = None
    state = State.FOLLOW
    state_start_ns = 0
    node.get_logger().info("Start service called. Robot will move.")
    return response


def stop_line_follower_callback(request, response):
    global should_move
    should_move = False
    node.get_logger().info("Stop service called. Robot will stop.")
    return response

# ─── Callbacks ─────────────────────────────────────────────────────────────────
def image_callback(msg):
    global image_input
    try:
        image_input = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        node.get_logger().error(f"Failed to convert image: {e}")

# ─── Contour Detection ────────────────────────────────────────────────────────
def get_contour_data(mask, out, crop_w_start):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    mark, line = {}, {}
    for c in contours:
        M = cv2.moments(c)
        if M['m00'] <= MIN_AREA:
            continue
        cx = crop_w_start + int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        if M['m00'] > MIN_AREA_TRACK:
            line = {'x': cx, 'y': cy}
            cv2.drawContours(out, [c], -1, (255,255,0), 2)
        else:
            if not mark or mark['y'] > cy:
                mark = {'x': cx, 'y': cy}
                cv2.drawContours(out, [c], -1, (255,0,255), 2)
    side = None
    if mark and line:
        side = 'right' if mark['x'] > line['x'] else 'left'
    return line, side

# ─── Collision Detection ─────────────────────────────────────────────────────
def detect_collision(inv_mask, crop, crop_w_start):
    h, w = inv_mask.shape
    z0 = w//2 - COLLISION_ZONE_WIDTH//2
    z1 = w//2 + COLLISION_ZONE_WIDTH//2
    zone = inv_mask[:, z0:z1]
    cnts, _ = cv2.findContours(zone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        if cv2.contourArea(c) > COLLISION_AREA_THRESHOLD:
            cv2.rectangle(crop, (z0,0), (z1,h), (0,0,255), 2)
            return True
    return False

# ─── Timer Loop ───────────────────────────────────────────────────────────────
def timer_callback():
    global state, state_start_ns, error, just_seen_line, just_seen_right_mark
    global right_mark_count, finalization_countdown

    if not isinstance(image_input, np.ndarray):
        return
    now_ns = node.get_clock().now().nanoseconds
    h, w, _ = image_input.shape
    img = image_input.copy()
    ch0, ch1, cw0, cw1 = crop_size(h, w)
    crop = img[ch0:ch1, cw0:cw1]

    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)
    inv_mask = cv2.bitwise_not(mask)

    # State transitions
    if state == State.FOLLOW:
        if detect_collision(inv_mask, crop, cw0):
            state = State.STOPPED
            state_start_ns = now_ns
            node.get_logger().warn("Collision! Stopping.")
            publisher.publish(Twist())
            return

    elif state == State.STOPPED:
        if now_ns - state_start_ns >= STOP_WAIT_NS:
            state = State.BACKUP
            state_start_ns = now_ns
        else:
            publisher.publish(Twist())
            return

    elif state == State.BACKUP:
        msg = Twist()
        msg.linear.x = -0.1
        publisher.publish(msg)
        if now_ns - state_start_ns >= BACKUP_TIME_NS:
            state = State.TURN
            state_start_ns = now_ns
        return

    elif state == State.TURN:
        msg = Twist()
        msg.angular.z = 0.5
        publisher.publish(msg)
        if now_ns - state_start_ns >= TURN_TIME_NS:
            state = State.FOLLOW
        return

    # FOLLOW logic
    line, mark_side = get_contour_data(mask, img[ch0:ch1, cw0:cw1], cw0)
    cmd = Twist()
    if line:
        x = line['x']
        error = x - w//2
        cmd.linear.x = LINEAR_SPEED
        just_seen_line = True
        cv2.circle(img, (x, ch0 + line['y']), 5, (0,255,0), 7)
    else:
        if just_seen_line:
            just_seen_line = False
            error *= LOSS_FACTOR
        cmd.linear.x = 0.0

    if mark_side:
        node.get_logger().info(f"Track mark detected on {mark_side}")
        if mark_side == 'right' and finalization_countdown is None and abs(error) <= MAX_ERROR and not just_seen_right_mark:
            right_mark_count += 1
            just_seen_right_mark = True
            if right_mark_count > 1:
                finalization_countdown = int(FINALIZATION_PERIOD/TIMER_PERIOD) + 1
                node.get_logger().info("Finalization countdown started.")
    else:
        just_seen_right_mark = False

    cmd.angular.z = -float(error) * KP
    publisher.publish(cmd if should_move else Twist())

    cv2.rectangle(img, (cw0, ch0), (cw1, ch1), (0,0,255), 2)
    cv2.imshow("Line Follower", img)
    cv2.waitKey(5)

# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    global node, publisher
    node = Node('follower')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    node.create_subscription(Image, '/camera/image_raw', image_callback, rclpy.qos.qos_profile_sensor_data)
    node.create_service(Empty, 'start_line_follower', start_line_follower_callback)
    node.create_service(Empty, 'stop_line_follower', stop_line_follower_callback)
    node.create_timer(TIMER_PERIOD, timer_callback)

    node.get_logger().info("Line follower + recovery node started.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
