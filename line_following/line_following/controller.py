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
# REVERT TO THIS AFTER SS
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np

# class LineFollower(Node):
#     def __init__(self):
#         # The node name remains 'line_following'. It will be remapped to a specific
#         # namespace using your launch file.
#         super().__init__('line_following')
#         self.get_logger().info('Line follower node started.')

#         # Subscribe to the camera image topic (a relative topic will resolve to e.g. /robot1/camera/image_raw)
#         self.subscription = self.create_subscription(
#             Image,
#             'camera/image_raw',
#             self.image_callback,
#             10)
        
#         # Publisher for velocity commands on a relative topic "cmd_vel"
#         # This ensures that under a namespace, it will resolve to e.g. /robot1/cmd_vel.
#         self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
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



# CONTROLLER V4.0
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





# CONTROLLER V5.0 hopefully final
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from cv_bridge import CvBridge
import cv2
import numpy as np

# Create a bridge between ROS and OpenCV
bridge = cv_bridge.CvBridge()

## User-defined parameters: (Update these values to your liking)
# Minimum size for a contour to be considered anything
MIN_AREA = 500 

# Minimum size for a contour to be considered part of the track
MIN_AREA_TRACK = 5000

# Robot's speed when following the line
LINEAR_SPEED = 0.2

# Proportional constant to be applied on speed when turning 
# (Multiplied by the error value)
KP = 1.5/100 

# If the line is completely lost, the error value shall be compensated by:
LOSS_FACTOR = 1.2

# Send messages every $TIMER_PERIOD seconds
TIMER_PERIOD = 0.06

# When about to end the track, move for ~$FINALIZATION_PERIOD more seconds
FINALIZATION_PERIOD = 4

# The maximum error value for which the robot is still in a straight line
MAX_ERROR = 30

# BGR values to filter only the selected color range
lower_bgr_values = np.array([31,  42,  53])
upper_bgr_values = np.array([255, 255, 255])

def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
     Width_left_boundary, Width_right_boundary)
    """
    ## Update these values to your liking.

    return (1*height//3, height, width//4, 3*width//4)


# Global vars. initial values
image_input = 0
error = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None


def start_follower_callback(request, response):
    """
    Start the robot.
    In other words, allow it to move (again)
    """
    global should_move
    global right_mark_count
    global finalization_countdown
    should_move = True
    right_mark_count = 0
    finalization_countdown = None
    return response

def stop_follower_callback(request, response):
    """
    Stop the robot
    """
    global should_move
    global finalization_countdown
    should_move = False
    finalization_countdown = None
    return response

def image_callback(msg):
    """
    Function to be called whenever a new Image message arrives.
    Update the global variable 'image_input'
    """
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    # node.get_logger().info('Received image')

def get_contour_data(mask, out):
    """
    Return the centroid of the largest contour in
    the binary image 'mask' (the line) 
    and return the side in which the smaller contour is (the track mark) 
    (If there are any of these contours),
    and draw all contours on 'out' image
    """ 
    # get a list of contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    mark = {}
    line = {}

    for contour in contours:
        
        M = cv2.moments(contour)
        # Search more about Image Moments on Wikipedia :)

        if M['m00'] > MIN_AREA:
        # if countor.area > MIN_AREA:

            if (M['m00'] > MIN_AREA_TRACK):
                # Contour is part of the track
                line['x'] = crop_w_start + int(M["m10"]/M["m00"])
                line['y'] = int(M["m01"]/M["m00"])

                # plot the area in light blue
                cv2.drawContours(out, contour, -1, (255,255,0), 1) 
                cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)
            
            else:
                # Contour is a track mark
                if (not mark) or (mark['y'] > int(M["m01"]/M["m00"])):
                    # if there are more than one mark, consider only 
                    # the one closest to the robot 
                    mark['y'] = int(M["m01"]/M["m00"])
                    mark['x'] = crop_w_start + int(M["m10"]/M["m00"])

                    # plot the area in pink
                    cv2.drawContours(out, contour, -1, (255,0,255), 1) 
                    cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)


    if mark and line:
    # if both contours exist
        if mark['x'] > line['x']:
            mark_side = "right"
        else:
            mark_side = "left"
    else:
        mark_side = None


    return (line, mark_side)

def timer_callback():
    """
    Function to be called when the timer ticks.
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """

    global error
    global image_input
    global just_seen_line
    global just_seen_right_mark
    global should_move
    global right_mark_count
    global finalization_countdown

    # Wait for the first image to be received
    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape

    image = image_input.copy()

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    # get the bottom part of the image (matrix slicing)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    
    # get a binary picture, where non-zero values represent the line.
    # (filter the color values so only the contour is seen)
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

    # get the centroid of the biggest contour in the picture,
    # and plot its detail on the cropped part of the output image
    output = image
    line, mark_side = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])  
    # also get the side in which the track mark "is"
    
    message = Twist()
    
    if line:
    # if there even is a line in the image:
    # (as the camera could not be reading any lines)      
        x = line['x']

        # error:= The difference between the center of the image
        # and the center of the line
        error = x - width//2

        message.linear.x = LINEAR_SPEED
        just_seen_line = True

        # plot the line centroid on the image
        cv2.circle(output, (line['x'], crop_h_start + line['y']), 5, (0,255,0), 7)

    else:
        # There is no line in the image. 
        # Turn on the spot to find it again. 
        if just_seen_line:
            just_seen_line = False
            error = error * LOSS_FACTOR
        message.linear.x = 0.0

    if mark_side != None:
        print("mark_side: {}".format(mark_side))

        if (mark_side == "right") and (finalization_countdown == None) and \
            (abs(error) <= MAX_ERROR) and (not just_seen_right_mark):

            right_mark_count += 1

            if right_mark_count > 1:
                # Start final countdown to stop the robot
                finalization_countdown = int(FINALIZATION_PERIOD / TIMER_PERIOD) + 1
                print("Finalization Process has begun!")

            
            just_seen_right_mark = True
    else:
        just_seen_right_mark = False

    
    # Determine the speed to turn and get the line in the center of the camera.
    message.angular.z = float(error) * -KP
    print("Error: {} | Angular Z: {}, ".format(error, message.angular.z))
    



    # Plot the boundaries where the image was cropped
    cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)

    # Uncomment to show the binary picture
    #cv2.imshow("mask", mask)

    # Show the output image to the user
    cv2.imshow("output", output)
    # Print the image for 5milis, then resume execution
    cv2.waitKey(5)

    # Check for final countdown
    if finalization_countdown != None:
        if finalization_countdown > 0:
            finalization_countdown -= 1

        elif finalization_countdown == 0:
            should_move = False


    # Publish the message to 'cmd_vel'
    if should_move:
        publisher.publish(message)
    else:
        empty_message = Twist()
        publisher.publish(empty_message)


def main():
    rclpy.init()
    global node
    node = Node('follower')

    global publisher
    publisher = node.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
    subscription = node.create_subscription(Image, 'camera/image_raw',
                                            image_callback,
                                            rclpy.qos.qos_profile_sensor_data)

    timer = node.create_timer(TIMER_PERIOD, timer_callback)

    start_service = node.create_service(Empty, 'start_follower', start_follower_callback)
    stop_service = node.create_service(Empty, 'stop_follower', stop_follower_callback)

    rclpy.spin(node)

try:
    main()
except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
    empty_message = Twist()
    publisher.publish(empty_message)

    node.destroy_node()
    rclpy.shutdown()
    exit()