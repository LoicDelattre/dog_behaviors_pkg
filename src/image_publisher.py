#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

def main():
    rospy.init_node("ball_image_publisher", anonymous=True)

    # Create a publisher (does not send data yet)
    pub = rospy.Publisher("/ball_image", Image, queue_size=10)

    rospy.spin()  # Keep the node alive

if __name__ == "__main__":
    main()
