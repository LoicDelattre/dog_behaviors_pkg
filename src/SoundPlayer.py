import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


soundhandle = SoundClient(sound_action="/robotsound")
path = '/home/ubuntu/catkin_ws/src/dog_behaviors_pkg/sounds/animals.wav'

def sound_callback(msg):
        print(msg.data)
        if msg.data == "SKIBIDI FORWARD":
                soundhandle.playWave(path)  
                print("AHOOOOOOOOOOOOOO")
        pass

def main():
    rospy.init_node("sound_publisher", anonymous=True)

    # Create a publisher (does not send data yet)
    pub = rospy.Publisher("/sound_type", String, queue_size=10)

    rospy.init_node('sound_player', anonymous=True)

    
    rospy.spin()
    

if __name__ == '__main__':
    main()
