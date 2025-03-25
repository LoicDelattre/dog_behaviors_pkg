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
    rospy.init_node('sound_player', anonymous=True)

    subscriber = rospy.Subscriber("/sound_type", String, sound_callback)
    
    rospy.spin()
    

if __name__ == '__main__':
    main()