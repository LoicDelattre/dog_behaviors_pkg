#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

def main():
    rospy.init_node("sound_publisher", anonymous=True)

    # Create a publisher (does not send data yet)
    pub = rospy.Publisher("/sound_type", String, queue_size=10)

    rospy.spin()  # Keep the node alive

if __name__ == "__main__":
    main()
