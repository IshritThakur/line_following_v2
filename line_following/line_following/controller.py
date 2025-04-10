# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np

# class LineFollower(Node):
#     def __init__(self):
#         super().__init__('line_follower')
#         self.get_logger().info('started')
        
        
        
#         # Subscribe to the camera image topic (adjust if necessary)
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.image_callback,
#             10)
#         # Publisher to send velocity commands
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.bridge = CvBridge()

#     def image_callback(self, msg):
#         try:
#             # Convert the ROS image to an OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             self.get_logger().error(f"CvBridge Error: {e}")
#             return

#         # Convert the image to grayscale and crop the lower part
#         gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#         height, width = gray.shape
#         crop_height = 100
#         crop_img = gray[height - crop_height:height, 0:width]

#         # Apply binary threshold to highlight the line (adjust threshold value if needed)
#         _, thresh = cv2.threshold(crop_img, 200, 255, cv2.THRESH_BINARY_INV)

#         # Find contours in the thresholded image
#         contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         twist = Twist()
#         if contours:
#             # Choose the largest contour as the line
#             c = max(contours, key=cv2.contourArea)
#             M = cv2.moments(c)
#             if M['m00'] != 0:
#                 # Calculate the centroid of the contour
#                 cx = int(M['m10'] / M['m00'])
#                 # Determine error from the center of the image
#                 error = cx - (width / 2)
#                 twist.linear.x = 0.2  # Constant forward speed
#                 twist.angular.z = -float(error) / 100  # Proportional control for steering
#                 self.get_logger().info(f"Line detected. Error: {error}, Angular z: {twist.angular.z:.2f}")
#             else:
#                 self.get_logger().warn("Zero moment, cannot compute centroid.")
#         else:
#             # If no line is found, stop the robot
#             self.get_logger().info("Line not found, stopping robot.")
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0

#         # Publish the Twist message
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










# CONTROLLER V2.0


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from std_srvs.srv import Empty

# import numpy as np
# import cv2
# import cv_bridge

# # Bridge between ROS and OpenCV
# bridge = cv_bridge.CvBridge()

# # User-defined parameters (tweak these as needed)
# MIN_CONTOUR_AREA = 200      # Minimum area for any detected contour
# MIN_TRACK_AREA   = 2000     # Minimum area for contour to be considered the main line
# FORWARD_SPEED    = 0.25     # Forward speed (m/s)
# TURNING_FACTOR   = 2.0/100  # Proportional turning factor (multiplied by error)
# LOST_LINE_MULTIPLIER = 1.3  # Multiplier when the line is momentarily lost
# TIMER_INTERVAL   = 0.05     # Timer interval in seconds
# FINAL_COUNTDOWN_SEC = 3     # Duration to continue after final marker detection
# CENTER_TOLERANCE = 25       # Tolerance for error (in pixels)

# # For a black line, we use a low range of BGR values
# LINE_LOWER_BOUNDS = np.array([0, 0, 0])
# LINE_UPPER_BOUNDS = np.array([50, 50, 50])

# def crop_region(image_height, image_width):
#     """
#     Compute cropping boundaries for the image.
#     Returns (crop_top, crop_bottom, crop_left, crop_right).
#     Currently cropping the bottom half, with some horizontal margin.
#     """
#     return (
#         image_height // 2,  # top
#         image_height,       # bottom
#         image_width // 5,   # left
#         4 * image_width // 5 # right
#     )

# # Global variables for state management
# latest_image      = None
# current_error     = 0
# detected_line     = False
# detected_marker   = False
# should_drive      = False
# marker_count      = 0
# stop_countdown    = None

# def start_line_follower(request, response):
#     """
#     Service callback to start line following.
#     """
#     global should_drive, marker_count, stop_countdown
#     should_drive      = True
#     marker_count      = 0
#     stop_countdown    = None
#     return response

# def stop_line_follower(request, response):
#     """
#     Service callback to stop line following.
#     """
#     global should_drive, stop_countdown
#     should_drive   = False
#     stop_countdown = None
#     return response

# def image_callback(msg):
#     global latest_image
#     latest_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#     cv2.imshow("Processed Output", latest_image)
#     cv2.waitKey(5)


# def process_contours(binary_mask, output_image, crop_x_offset):
#     """
#     Process contours to find the main line and any markers.
#     Returns:
#       - main_line: centroid of the primary line contour (if found)
#       - marker_side: side ("left" or "right") of a detected marker (if any)
#     """
#     contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     main_line = {}
#     marker   = {}

#     for cnt in contours:
#         moments = cv2.moments(cnt)
#         if moments['m00'] > MIN_CONTOUR_AREA:
#             centroid_x = crop_x_offset + int(moments["m10"] / moments["m00"])
#             centroid_y = int(moments["m01"] / moments["m00"])
#             if moments['m00'] > MIN_TRACK_AREA:
#                 # This contour is considered the track line
#                 main_line['x'] = centroid_x
#                 main_line['y'] = centroid_y
#                 cv2.drawContours(output_image, [cnt], -1, (0, 255, 255), 2)
#                 cv2.putText(output_image, f"{int(moments['m00'])}", (centroid_x, centroid_y),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
#             else:
#                 # This contour is considered a marker
#                 if (not marker) or (marker.get('y', float('inf')) > centroid_y):
#                     marker['x'] = centroid_x
#                     marker['y'] = centroid_y
#                     cv2.drawContours(output_image, [cnt], -1, (255, 0, 255), 2)
#                     cv2.putText(output_image, f"{int(moments['m00'])}", (centroid_x, centroid_y),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)

#     if marker and main_line:
#         marker_side = "right" if marker['x'] > main_line['x'] else "left"
#     else:
#         marker_side = None

#     return main_line, marker_side

# def timer_callback():
#     """
#     Timer callback to process the image and publish velocity commands.
#     """
#     global latest_image, current_error, detected_line, detected_marker
#     global should_drive, marker_count, stop_countdown

#     if latest_image is None:
#         return

#     img_height, img_width, _ = latest_image.shape
#     image_copy = latest_image.copy()

#     # Crop the region of interest
#     crop_top, crop_bottom, crop_left, crop_right = crop_region(img_height, img_width)
#     cropped_img = image_copy[crop_top:crop_bottom, crop_left:crop_right]

#     # Create a binary mask for the black line
#     # Create a binary mask for the black line
#     mask = cv2.inRange(cropped_img, LINE_LOWER_BOUNDS, LINE_UPPER_BOUNDS)
#     # Invert the mask so that the dark line becomes white (if that’s what you need)
#     mask = cv2.bitwise_not(mask)

#     # Display the binary mask for debugging
#     cv2.imshow("Binary Mask", mask)
#     cv2.waitKey(5)


#     main_line, marker_side = process_contours(mask, image_copy[crop_top:crop_bottom, crop_left:crop_right], crop_left)
    
#     cmd = Twist()

#     if main_line:
#         x_pos = main_line['x']
#         current_error = x_pos - (img_width // 2)
#         cmd.linear.x = FORWARD_SPEED
#         detected_line = True
#         cv2.circle(image_copy, (main_line['x'], crop_top + main_line['y']), 5, (0, 255, 0), 7)
#     else:
#         # If line is lost, stop forward motion, but keep turning to find it
#         if detected_line:
#             detected_line = False
#             current_error *= LOST_LINE_MULTIPLIER
#         cmd.linear.x = 0.0

#     if marker_side is not None:
#         print(f"Marker detected on: {marker_side}")
#         if marker_side == "right" and stop_countdown is None and abs(current_error) <= CENTER_TOLERANCE and not detected_marker:
#             marker_count += 1
#             if marker_count > 1:
#                 stop_countdown = int(FINAL_COUNTDOWN_SEC / TIMER_INTERVAL) + 1
#                 print("Initiating stop countdown!")
#             detected_marker = True
#     else:
#         detected_marker = False

#     cmd.angular.z = -TURNING_FACTOR * float(current_error)
#     print(f"Error: {current_error} | Angular: {cmd.angular.z}")
    
#     # Draw crop boundaries for visualization
#     cv2.rectangle(image_copy, (crop_left, crop_top), (crop_right, crop_bottom), (0, 0, 255), 2)
#     cv2.imshow("Processed Output", image_copy)
#     cv2.waitKey(5)

#     if stop_countdown is not None:
#         if stop_countdown > 0:
#             stop_countdown -= 1
#         else:
#             should_drive = False

#     if should_drive:
#         publisher.publish(cmd)
#     else:
#         publisher.publish(Twist())

# def main():
#     rclpy.init()
#     global node, publisher
#     node = Node('line_follower')
#     publisher = node.create_publisher(Twist, 'cmd_vel', 10)
#     node.create_subscription(Image, 'camera/image_raw', image_callback, 10)
#     node.create_timer(TIMER_INTERVAL, timer_callback)
#     node.create_service(Empty, 'start_line_follower', start_line_follower)
#     node.create_service(Empty, 'stop_line_follower', stop_line_follower)
#     rclpy.spin(node)

# if __name__ == '__main__':
#     try:
#         main()
#     except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
#         publisher.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()




# CONTROLLER V3.0


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import tf2_ros
from tf2_ros import TransformException

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('Line follower node with TF collision avoidance started.')

        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Publisher to send velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        # TF2 setup: Create a buffer and listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # List of TF frames of other robots to avoid.
        # Adjust this list based on the unique namespaces of the other robots.
        self.other_robot_frames = ["robot2/base_link", "robot3/base_link"]

    def compute_tf_repulsive_angular(self):
        """
        Queries TF for each other robot frame relative to our "base_link" and computes
        a repulsive angular adjustment if another robot is too close.
        Returns:
           A repulsive angular term (in radians per second) to steer away.
        """
        SAFETY_THRESHOLD = 1.0   # meters – tune this based on your simulation scale
        AVOIDANCE_GAIN = 0.5     # gain factor; adjust as needed
        repulsive_sum = 0.0

        # Iterate over all the other robot frames
        for frame in self.other_robot_frames:
            try:
                # Query transform from our "base_link" to the other robot's base_link.
                # Note: Adjust the target/source frames if your TF tree is organized differently.
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform("base_link", frame, now, timeout=rclpy.duration.Duration(seconds=0.5))
                # Get translation components (in meters)
                dx = transform.transform.translation.x
                dy = transform.transform.translation.y
                distance = math.hypot(dx, dy)
                if distance < SAFETY_THRESHOLD and distance > 0:
                    # Strength scales inversely with distance
                    strength = (SAFETY_THRESHOLD - distance) / SAFETY_THRESHOLD
                    # Compute the angle to the other robot relative to our frame
                    angle = math.atan2(dy, dx)
                    # Use sin(angle) for a horizontal component.
                    # The negative sign ensures steering away from the other robot.
                    repulsive = AVOIDANCE_GAIN * strength * (-math.sin(angle))
                    repulsive_sum += repulsive
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed for {frame}: {ex}")
                continue

        return repulsive_sum

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # For debugging: Show the camera feed
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(5)

        # Convert the image to grayscale and crop the lower part
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        crop_height = 100
        crop_img = gray[height - crop_height:height, 0:width]

        # Apply binary threshold (THRESH_BINARY_INV so that a dark line becomes white)
        ret, thresh = cv2.threshold(crop_img, 200, 255, cv2.THRESH_BINARY_INV)

        # For debugging: Enlarge the binary mask window for a better view (taller window)
        mask_display = cv2.resize(thresh, (0, 0), fx=1.0, fy=2.0)
        cv2.imshow("Binary Mask", mask_display)
        cv2.waitKey(5)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        # Compute nominal line-following command from camera image
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                error = cx - (width / 2)
                twist.linear.x = 0.05  # Reduced forward speed
                line_following_angular = -float(error) / 150.0  # Nominal angular command
                self.get_logger().info(f"Line detected. Error: {error}, Base Angular: {line_following_angular:.2f}")
            else:
                line_following_angular = 0.0
                twist.linear.x = 0.0
        else:
            line_following_angular = 0.0
            twist.linear.x = 0.0

        # Compute TF-based repulsive angular component for collision avoidance
        repulsive_angular = self.compute_tf_repulsive_angular()
        self.get_logger().info(f"Repulsive Angular: {repulsive_angular:.2f}")

        # Fuse the two angular commands
        twist.angular.z = line_following_angular + repulsive_angular

        self.publisher.publish(twist)

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