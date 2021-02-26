#!/bin/bash
source /opt/ros/%YOUR_ROS_DISTRO%/setup.bash
roscore

source /opt/ros/%YOUR_ROS_DISTRO%/setup.bash
chmod u+x src/parking_app_ros_pkg/bin/parking_app_flask_server.py
catkin_make
source devel/setup.bash
rosrun parking_app_ros_pkg parking_app_flask_server.py

source /opt/ros/%YOUR_ROS_DISTRO%/setup.bash
chmod u+x src/parking_app_ros_pkg/scripts/Service_Server.py
catkin_make
source devel/setup.bash
rosrun parking_app_ros_pkg Service_Server.py
