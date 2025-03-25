import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class TurtlebotVisionController:
    def __init__(self):
        rospy.init_node("turtlebot_vision_controller", anonymous=True)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to USB camera topic
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        # Publisher for velocity commands
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Publisher for velocity commands
        self.image_publisher = rospy.Publisher("/ball_image", Image, queue_size=10)

        #Publisher for sound effects
        self.sound_publisher = rospy.Publisher("/sound_type", String, queue_size=10)

        # Define movement speed
        self.forward_speed = 0.05
        self.turn_speed = 0.5
        self.going_to_ball = False
        self.buffer_movement_cmd = 0
        self.threshold = 1000

    def image_callback(self, msg):
        """
        Callback function to process the received image and detect red objects.
        """
        try:
                if not self.going_to_ball :

                        # Process the image to detect red color
                        movement_cmd = self.process_image(msg)
                        self.buffer_movement_cmd = movement_cmd

                        # Publish movement command
                        self.publisher.publish(movement_cmd)
                else:
                        self.publisher.publish(self.buffer_movement_cmd)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def process_image(self, image):
        """
        Detect red color in the image and determine movement.
        """
        twist_msg = Twist()
        #Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        # Convert image to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the red color range (OpenCV uses HSV for better color segmentation)
        lower_red1 = np.array([0, 97, 132])   # Lower boundary for red
        upper_red1 = np.array([6, 255, 255]) # Upper boundary for red
        #lower_red2 = np.array([170, 120, 70]) # Second lower boundary (red appears at both ends of HSV spectrum)
        #upper_red2 = np.array([180, 255, 255])# Second upper boundary

        # Create masks to detect red color in the image
        mask = cv2.inRange(hsv, lower_red1, upper_red1)
        #mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        #mask = mask1 + mask2  # Combine both masks

        # Find contours of detected red regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest red object
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            # Get bounding box around the red object
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2  # Find center of red object
            img_center_x = cv_image.shape[1] // 2  # Center of the image

            # Define movement logic based on object position
            if area > self.threshold:  # Ignore small objects (filter out noise)
                if abs(center_x - img_center_x) < 50:
                    twist_msg.linear.x = self.forward_speed  # Move forward
                    twist_msg.angular.z = 0.0
                    
                    rospy.loginfo(f"Item area = {area}")
                    rospy.loginfo("Red object detected! Skibidiing toward it.")
                    self.going_to_ball = True

                    self.image_publisher.publish(image)
                    self.sound_publisher.publish("SKIBIDI FORWARD")
                elif center_x < img_center_x:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = self.turn_speed  # Turn left
                else:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = -self.turn_speed  # Turn right
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.turn_speed  # Rotate to search

        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.turn_speed  # Rotate to search

        return twist_msg

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = TurtlebotVisionController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
