###on dog robot###
First run in seperate terminals:

LIBRAIRIES:

**Turtlebot ROS**
$roslaunch turtlebot3_bringup turtlebot3_robot.launch 


**Camera ROS**
$roslaunch usb_cam usb_cam-test.launch

**Sound ros package**
$rosrun sound_play soundplay_node.py

OWN CODE:
$cd catkin_ws/src/dog_behaviors_pkg/src

**For publishing images to pc (debug)**
$python3 image_publisher.py 

**For playing sounds**
$python3 SoundPlayer.py
#GoForBall publishes to it, and it then publishes to sound_play

**For Obstacle Detection**
$python3 ObstaclesDetection.py

**For exploring**
$python3 Explorer.py

**For going for Ball**
$python3 GoForBall.py

TO INCERASE VOLUME
amixer sset 'Headphone' 80%
