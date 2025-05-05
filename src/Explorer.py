import rospy
import random
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import time

class Explorer():
	def __init__(self):
		self.exploringTime = 2.0
		self.timeStartExplore = 0.0
		self.directionDecidedFlag = False
		self.direction = "forward"
		
		self.ballSeen = False

		# Create a publisher for /cmd_vel
		self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		pass

	def decide_direction(self):
		direction = random.choice(["forward", "left", "right", "forward-left", "forward-right"])
		return direction

	def movement_callback(self, msg):
		if msg.data == 0:  # Check if the received message is 0
			self.ballSeen = False
			twist = Twist()
			direction = ""
			if not self.directionDecidedFlag:
				self.direction  = self.decide_direction()
				self.directionDecidedFlag = True
				self.timeStartExplore = time.time()
				rospy.loginfo(f"Exploring {self.direction}")

			if time.time() - self.timeStartExplore > self.exploringTime:
				self.directionDecidedFlag = False
			
			if self.direction == "forward":
				twist.linear.x = 0.3  # Move forward
				twist.angular.z = 0.0
			elif self.direction == "backward":
				twist.linear.x = -0.5  # Move backward
				twist.angular.z = 0.0
			elif self.direction == "forward-left":
				twist.linear.x = 0.3  # Move backward
				twist.angular.z = 1
			elif self.direction == "forward-right":
				twist.linear.x = 0.3 # Move backward
				twist.angular.z = -1
			elif self.direction == "left":
				twist.linear.x = 0.0
				twist.angular.z = 1.0  # Turn left
			elif self.direction == "right":
				twist.linear.x = 0.0
				twist.angular.z = -1.0  # Turn right
			
			self.publisher.publish(twist)  # Publish the Twist message
		if msg.data == 1:
			if not self.ballSeen:
				rospy.loginfo(f"Stop exploring, Ball seen")
				self.ballSeen = True


	def main(self):
		print("Captain Teemo au rapport (l'explorer tsais)")
		rospy.init_node('explorer', anonymous=True)
		self.explorer_publisher = rospy.Publisher("/ball_status", Int32, queue_size=10)
		# Create a subscriber for /ball_status
		rospy.Subscriber("/ball_status", Int32, self.movement_callback)
		
		rospy.spin()

if __name__ == '__main__':
	exp = Explorer()
	exp.main()
