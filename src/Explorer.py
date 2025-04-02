import rospy
import random
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

def movement_callback(msg):
        if msg.data == 0:  # Check if the received message is 0
                rospy.loginfo("Exploring...")
                twist = Twist()
                twist.linear.x = random.uniform(0.1, 0.5)  # Random linear velocity
                twist.angular.z = random.uniform(-1.0, 1.0)  # Random angular velocity
                Explorer.publisher.publish(twist)  # Publish the Twist message

class Explorer:
        publisher = None

def main():
        rospy.init_node('explorer', anonymous=True)

        # Create a publisher for /cmd_vel
        Explorer.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # Create a subscriber for /ball_status
        rospy.Subscriber("/ball_status", Int32, movement_callback)
        
        rospy.spin()

if __name__ == '__main__':
        main()
