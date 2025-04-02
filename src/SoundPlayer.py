import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


soundhandle = SoundClient(sound_action="/robotsound")
path_follow = '/home/ubuntu/catkin_ws/src/dog_behaviors_pkg/sounds/animals.wav'
path_search = '/home/ubuntu/catkin_ws/src/dog_behaviors_pkg/sounds/searching.wav'
buffer_msg = ""

def sound_callback(msg):
        if buffer_msg != msg:
                soundhandle.stopAll()
        print(msg.data)
        if msg.data == "SKIBIDI FORWARD":
                soundhandle.playWave(path_follow)  
                print("AHOOOOOOOOOOOOOO")
        elif msg.data == "SEARCHING":
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
