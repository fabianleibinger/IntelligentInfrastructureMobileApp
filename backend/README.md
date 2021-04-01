# Parking app backend & ROS communication to parking management system

The parking app has a connected backend which can be addressed via HTTP requests. Therefore, the backend opens a python flask server. IP address and port as well as the name of the parking garage can be modified in the config.properties file.

## Installation and usage
To run the flask server on your device, copy parking_app_ros_pkg to your ROS directory with a setted up catkin workspace (e.g. catkin_ws/src). The device must have a working ROS installation and one should be able to build a ROS package. For further questions, follow steps 1-4 of the general [ROS tutorial](http://wiki.ros.org/ROS/Tutorials).

Put parking_app_ros_pkg folder under catkin_ws/src directory (right next to other ROS packages and a self-generated CMakeLists.txt file). Copy setup_script.sh and requirements.txt in the top-level directory catkin_ws.

In a terminal, navigate to catkin_ws/ and run command 'bash setup_script.sh'. The script will install the neccessary requirements (pip3 needed). Afterwards, it starts a ROS master node (roscore), builds your catkin workspace and makes included python scripts executable. Both the flask server as well as the ROS service server will be started and can be accessed using the implemented HTTP routes. If you have an implementation of the ROS service server yourself (e.g. parking management system), modify setup_script.sh so that ROS scripts run on same roscore and dummy implementation in Service_Server.py is not used.

For testing purpose, open a browser and type in a valid address, e.g. http://127.0.0.1:2525/free if your server runs on IP 127.0.0.1 (localhost) and port 2525. If you have the app installed or running on an emulator, use the given buttons to call methods from the API provider. Returns will have JSON format.

## Modification and further development
ROS services are defined in parking_app_ros_pkg/srv, ROS messages in parking_app_ros_pkg/msg. If you add or remove ROS elements from there, modify CMakeLists.txt for your needs.

The backend application is implemented in three different modules. It represents a layered architecture with UI, logic and data management. If you want to modify HTTP routes, JSON fields or data exchange with the frontend in general, go to parking_app_flask_server.py (parking_app_ros_pkg/bin). It also includes the main method and calls methods from parking_communication.py. In parking_communication.py (parking_app_ros_pkg/src/) you can change the logic and the communication with external ROS services. Use class structure and method comments to navigate to your concern. The module database.py (parking_app_ros_pkg/src) provides a data management which primarly is used to map IDs from the frontend with corresponding IDs from the parking management system.

Please be aware of the given folder structure. Otherwise, rosrun could have problems locating your scripts.
