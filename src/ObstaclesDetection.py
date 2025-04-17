import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Int32

class ObstacleDetection:

    def __init__(self):
        rospy.init_node('obstacle_detection', anonymous=True)

        self.obstacle_status_pub = rospy.Publisher("/obstacle_status", Bool, queue_size=10)
        self.subscriber_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.obstacle_distance_threshold = 0.25  # meters
        self.obstacle_detected = False

    def scan_callback(self, scan_msg):
        scan_range = scan_msg.ranges
        angle_range = 20

        center_index = len(scan_range) // 2
        start_index = max(center_index - angle_range, 0)
        end_index = min(center_index + angle_range, len(scan_range))

        front_ranges = [r for r in scan_range[start_index:end_index] if 0.0 < r < float('inf')]

        if front_ranges:
            min_distance = min(front_ranges)
            is_obstacle = min_distance < self.obstacle_distance_threshold
            twist_msg = Twist()
            if is_obstacle != self.obstacle_detected:
                self.obstacle_detected = is_obstacle
                self.obstacle_status_pub.publish(Bool(data=is_obstacle))
                if is_obstacle:
                        rospy.logwarn("Obstacle detected at %.2f meters!", min_distance)
                        twist_msg.linear.x = 0.0
                        twist_msg.angular.z = 0.5
                        self.publisher.publish(twist_msg)
                else:
                        rospy.loginfo("Obstacle cleared.")
        else:
            rospy.loginfo("No valid LiDAR data in front range.")
            self.obstacle_detected = False
            self.obstacle_status_pub.publish(Bool(data=False))

if __name__ == '__main__':
    try:
        ObstacleDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
