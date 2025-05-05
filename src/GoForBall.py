import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time
import argparse

class TurtlebotVisionController:
    def __init__(self, mode):
        rospy.init_node("turtlebot_vision_controller", anonymous=True)

        self.mode = mode
        print(self.mode)

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

        #Publisher for explorer
        self.explorer_publisher = rospy.Publisher("/ball_status", Int32, queue_size=10)
        
        rospy.Subscriber("/obstacle_status", Bool, self.obstacle_callback)

        
        self.obstacle_detected = False

        # Define movement speed
        self.forward_speed = 0.4
        self.minAngle = 0
        self.maxAngle = 45
        self.search_turn_speed = 0.9

        self.sizeThreshold = 300

        self.exploringFlag = False
        self.searchingFlag = False
        self.goingForBallFlag = False

        self.ballSeen = False

        self.delta = 0
        self.last_time = None
        self.firstImageFlag = False

        self.maxTimeSinceBallSeen = 5.0
        self.timeBallSeen = 0.0

        print("Go For Ball Node Launched")

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg
        pass

    def computeFPS(self):
        current_time = time.time()

        if self.last_time is not None:
            self.delta = 1.0 / (current_time - self.last_time)
            #rospy.loginfo(f"Current FPS: {self.delta:.2f}")

        self.last_time = current_time
        pass

    def image_callback(self, msg):
        """
        Callback function to process the received image and detect red objects.
        """
        self.computeFPS()
        try:
            if self.firstImageFlag:
                # No obstacle → normal movement
                movement_cmd = self.process_image(msg)
                self.buffer_movement_cmd = movement_cmd

                if not self.exploringFlag:
                         self.publisher.publish(movement_cmd)
            else:
                # On first image, give a small nudge
                msg = Twist()
                msg.linear.x = 0.01
                self.publisher.publish(msg)
                self.firstImageFlag = True

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
    def process_image(self, image):
        """
        Detect red color in the image and determine movement.
        """
        twist_msg = Twist()
        #Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        frame_height, frame_width, channels = cv_image.shape

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
                self.timeBallSeen = time.time()
                self.explorer_publisher.publish(1)
                # Find the largest red object
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                #  Get bounding box around the red object
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2  # Find center of red object
                img_center_x = cv_image.shape[1] // 2  # Center of the image

                # Define movement logic based on object position
                self.image_publisher.publish(image)
                if area > self.sizeThreshold:  # Ignore small objects (filter out noise)
                        angle_to_ball = self.calculate_horizontal_angle(center_x, frame_width, self.maxAngle)
                        self.search_turn_speed = -self.computeTurnSpeed(angle_to_ball) #adjust turn speed to follow ball
                        self.ballSeen = True
                        self.explorer_publisher.publish(1)

                        if abs(center_x - img_center_x) < 80:
                                twist_msg.linear.x = self.forward_speed  # Move forward
                                twist_msg.angular.z = 0.0
                    
                                if not self.goingForBallFlag:
                                        if self.mode == "DEBUG":
                                                rospy.loginfo(f"Item area = {area}")
                                                rospy.loginfo("Red object detected! Skibidiing toward it.")
                                        self.searchingFlag = False
                                        self.exploringFlag = False
                                self.goingForBallFlag= True
                                self.sound_publisher.publish("SKIBIDI FORWARD")
                        elif center_x < img_center_x:
                                twist_msg.linear.x = self.forward_speed
                                twist_msg.angular.z = self.search_turn_speed # Turn left
                        else:
                                twist_msg.linear.x = self.forward_speed
                                twist_msg.angular.z = self.search_turn_speed  # Turn right
                else:
                        twist_msg = self.searching(twist_msg)
        else:
                twist_msg = self.searching(twist_msg)
        if self.mode == "DEBUG-0MOV":
            return Twist()
        else:
            return twist_msg

    def searching(self, twist_msg):
            self.sound_publisher.publish("SEARCHING")
            if time.time() - self.timeBallSeen < self.maxTimeSinceBallSeen and self.ballSeen:
                    twist_msg.angular.z = self.search_turn_speed  # Rotate to search
                    #twist_msg.linear.x = self.forward_speed
                    if not self.searchingFlag:
                        self.goingForBallFlag = False
                        self.exploringFalg = False 
                        if self.mode == "DEBUG":
                                rospy.loginfo("SEARCHING")

                    self.searchingFlag = True

            else:
                if not self.exploringFlag:
                    self.goingForBallFlag = False
                    self.searchingFlag = False
                    if self.mode == "DEBUG":
                            rospy.loginfo("MESSAGE TO EXPLORER")
 
                self.exploringFlag = True
                self.explorer_publisher.publish(0) #random explorer
            return twist_msg

    def calculate_horizontal_angle(self, x_center, frame_width, max_angle):
            """
            Calculate the angle from the center of the frame.
            Negative = left, Positive = right.
            """
            threshold = 1
            offset = x_center - (frame_width / 2)
            normalized_offset = offset / (frame_width / 2)
            angle = normalized_offset * max_angle #link to time between frames

            if abs(angle) < threshold:
                angle = 0
            return angle

    def computeTurnSpeed(self, angle):
        speed = 0
        max_speed = 100
        min_speed = 10
        speed = (angle - self.minAngle) / (self.maxAngle - self.minAngle) * (max_speed - min_speed) + min_speed
        return speed/self.delta

    def run(self):
        rospy.spin()
        

if __name__ == "__main__":
    try:
        myParser = argparse.ArgumentParser()
        myParser.add_argument("-m", help='message to be sent', type=str)
        args = (myParser.parse_args())
        controller = TurtlebotVisionController(args.m)
        controller.run()
    except rospy.ROSInterruptException:
        pass
