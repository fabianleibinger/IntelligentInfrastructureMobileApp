#!/bin/bash

chmod u+x src/parking_app_ros_pkg/bin/parking_app_flask_server.py
catkin_make
source devel/setup.bash
rosrun parking_app_ros_pkg parking_app_flask_server.py
