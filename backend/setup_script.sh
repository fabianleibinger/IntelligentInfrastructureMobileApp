#!/bin/bash

### change path to match the username
cd /home/pi/catkin_ws/
pip3 install -r requirements.txt

export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_HOSTNAME=192.168.0.20
export ROS_IP=192.168.0.20

sudo chmod u+x src/parking_app_ros_pkg/bin/parking_app_flask_server.py
sudo chmod u+x src/parking_app_ros_pkg/scripts/Service_Server.py
catkin_make
source devel/setup.bash

rosrun parking_app_ros_pkg parking_app_flask_server.py
