import rospy
import random
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

def movement_callback(msg):
        if msg.data == 0:  # Check if the received message is 0
                rospy.loginfo("Exploring...")
                twist = Twist()
                # Randomly decide the direction of movement
                direction = random.choice(["forward", "backward", "left", "right"])
                
                if direction == "forward":
                        twist.linear.x = 0.5  # Move forward
                        twist.angular.z = 0.0
                elif direction == "backward":
                        twist.linear.x = -0.5  # Move backward
                        twist.angular.z = 0.0
                elif direction == "left":
                        twist.linear.x = 0.0
                        twist.angular.z = 1.0  # Turn left
                elif direction == "right":
                        twist.linear.x = 0.0
                        twist.angular.z = -1.0  # Turn right
                
                Explorer.publisher.publish(twist)  # Publish the Twist message
                
                # Use a timer to delay the next movement without blocking
                rospy.sleep(5)

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
