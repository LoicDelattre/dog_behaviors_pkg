import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import time

def exploring_callback(msg):
        if msg == 0: 
                twist_msg = Twist()
                print("exploring")
        pass

def main():
    rospy.init_node('explorer', anonymous=True)

    # Create a publisher (does not send data yet)
    self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    sub = rospy.Subscriber("/ball_status", Int32, exploring_callback)
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
