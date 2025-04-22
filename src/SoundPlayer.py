import rospy
from sound_play.libsoundplay import SoundClient
from std_srvs.srv import Empty
from std_msgs.msg import String
import time
from pydub.utils import mediainfo


class SoundPlayer():
	def __init__(self):
		self.stop_sound = rospy.ServiceProxy('/sound_play/stopAll', Empty)

		self.soundhandle = SoundClient(sound_action="/robotsound")
		self.path_follow = '/home/ubuntu/catkin_ws/src/dog_behaviors_pkg/sounds/animals.wav'
		self.path_search = '/home/ubuntu/catkin_ws/src/dog_behaviors_pkg/sounds/searching3.wav'
		self.buffer_msg = ""

		self.soundPlaying = False

		self.soundStartTime = 0.0
		self.currentFileDuration = 0.0
		pass
	def get_sound_duration(self, file_path):
		info = mediainfo(file_path)
		return float(info['duration'])

	def sound_callback(self, msg):
		print(msg)

		if time.time() - self.soundStartTime > self.currentFileDuration:
			self.soundPlaying = False
			print("sound done")

		if msg.data == "SKIBIDI FORWARD" and not self.soundPlaying:
			#self.stop_sound()
			print("AHOOOOOOOOOOOOOO")
			self.soundPlaying = True
			self.soundStartTime = time.time()
			self.soundhandle.playWave(self.path_follow)
			self.currentFileDuration = self.get_sound_duration(self.path_follow)

		elif msg.data == "SEARCHING" and not self.soundPlaying:
			#self.stop_sound()
			print("WHERE IS IT")
			self.soundPlaying = True
			self.soundStartTime = time.time()
			self.soundhandle.playWave(self.path_search)  
			self.currentFileDuration = self.get_sound_duration(self.path_search)

		pass

	def main(self):
		print("sound player launched")
		rospy.init_node('sound_player', anonymous=True)

		# Create a publisher (does not send data yet)
		pub = rospy.Publisher("/sound_type", String, queue_size=10)

		sub = rospy.Subscriber("/sound_type", String, self.sound_callback) 
		rospy.spin()


if __name__ == '__main__':
    sd = SoundPlayer()
    sd.main()
