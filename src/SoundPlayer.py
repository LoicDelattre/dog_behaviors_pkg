import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
import time
from pydub.utils import mediainfo

stop_sound = rospy.ServiceProxy('/sound_play/stopAll', Empty)

soundhandle = SoundClient(sound_action="/robotsound")
path_follow = '/home/ubuntu/catkin_ws/src/dog_behaviors_pkg/sounds/animals.wav'
path_search = '/home/ubuntu/catkin_ws/src/dog_behaviors_pkg/sounds/searching.wav'
buffer_msg = ""

goingForBallFlag = false
searchingFlag = false

soundStartTime = 0.0
currentFileDuration = 0.0

def get_sound_duration(file_path):
        info = mediainfo(file_path)
        return float(info['duration'])

def sound_callback(msg):
        if buffer_msg != msg:
                soundhandle.stopAll()
        print(msg.data)

        if time.time() - soundStartTime > currentFileDuration:
                print("sound done")

        if msg.data == "SKIBIDI FORWARD" and not goingForBallFlag:
                stop_sound()
                goingForBallFlag = true
                searchingFlag = false
                soundStartTime = time.time()
                currentFileDuration = get_sound_duration(path_follow)
                soundhandle.playWave(path_follow)  
                print("AHOOOOOOOOOOOOOO")

        elif msg.data == "SEARCHING" and not searchingFlag:
                stop_sound()
                searchingFlag = true
                goingForBallFlag = false
                soundStartTime = time.time()
                currentFileDuration = get_sound_duration(path_search)
                soundhandle.playWave(path_search)  
                print("WHERE IS IT")

        pass

def main():
    rospy.init_node('sound_player', anonymous=True)

    # Create a publisher (does not send data yet)
    pub = rospy.Publisher("/sound_type", String, queue_size=10)

    sub = rospy.Subscriber("/sound_type", String, sound_callback)
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
