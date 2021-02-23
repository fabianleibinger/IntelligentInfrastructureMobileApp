# IntelligentInfrastructureMobileApp

Python Flask Server & ROS Communication (Backend)

To use the flask server on your device, copy parking_app_ros_pkg to your ROS directory with a setted up catkin workspace (e.g. catkin_ws/src).
Open the command line and navigate to your catkin workspace. Run 'roscore' to initialize a ROS master node.
In a new terminal, make the python file executable by running 'chmod u+x bin/parking_app_flask_server.py'.
Afterwards run 'catkin_make' to build your ROS package. Executable files will be created automatically.
Then, run 'source devel/setup.bash' to set up the environment including new devel folder.
Finally, you can execute the python script and start the flask server with 'rosrun parking_app_ros_pkg parking_app_flask_server.py'.
