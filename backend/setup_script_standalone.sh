#!/bin/bash

### (change path to match the username
cd /home/pi/catkin_ws/
pip3 install -r requirements.txt

roscore &

sudo chmod u+x src/parking_app_ros_pkg/bin/parking_app_flask_server.py
sudo chmod u+x src/parking_app_ros_pkg/scripts/Service_Server.py
catkin_make
source devel/setup.bash

rosrun parking_app_ros_pkg Service_Server.py &
rosrun parking_app_ros_pkg parking_app_flask_server.py