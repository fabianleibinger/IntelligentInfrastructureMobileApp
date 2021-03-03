#!/bin/bash
pip3 install -r requirements.txt

roscore &

chmod u+x src/parking_app_ros_pkg/bin/parking_app_flask_server.py
chmod u+x src/parking_app_ros_pkg/scripts/Service_Server.py
catkin_make
source devel/setup.bash

rosrun parking_app_ros_pkg Service_Server.py &
rosrun parking_app_ros_pkg parking_app_flask_server.py
