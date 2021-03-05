# Parking app backend & ROS communiaction to parking management system

The parking app has a connected backend which can be addressed via HTTP requests. Therefore, the backend opens a python flask server. IP address and port as well as the name of the parking garage can be modified in the config.properties file.

To run the flask server on your device, copy parking_app_ros_pkg to your ROS directory with a setted up catkin workspace (e.g. catkin_ws/src). The device must have a working ROS installation and one should be able to build a ROS package. For further questions, follow steps 1-4 of the general ROS tutorial ([a link](http://wiki.ros.org/ROS/Tutorials)).

Put parking_app_ros_pkg folder under catkin_ws/src directory (right next to other ROS packages and a self-generated CMakeLists.txt file). Copy setup_script.sh and requirements.txt in the top-level directory catkin_ws.

In a terminal, navigate to catkin_ws/ and run command 'bash setup_script.sh'. The script will install the neccessary requirements (pip3 needed). Afterwards, it starts a ROS master node (roscore), builds your catkin workspace and makes included python scripts executable. Both the flask server as well as the ROS service server will be started and can be accessed using the implemented HTTP routes.

For testing purpose, open a browser and type in a valid address, e.g. http://127.0.0.1:2525/free if your server runs on IP 127.0.0.1 (localhost) and port 2525. If you have the app installed or running on an emulator, use the given buttons to call methods from the API provider. Returns will have JSON format.
