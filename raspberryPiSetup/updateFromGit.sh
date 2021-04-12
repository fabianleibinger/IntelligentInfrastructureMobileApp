
sudo openvpn client.conf &
# type in username and password
cd /home/pi/git
sudo git clone https://ids-git.fzi.de/qm821/intelligente-infrastruktur-fzi-autopark.git
sudo cp /home/pi/git/intelligente-infrastruktur-fzi-autopark/backend/setup_script.sh /home/pi/catkin_ws/
sudo cp /home/pi/git/intelligente-infrastruktur-fzi-autopark/backend/setup_script_standalone.sh /home/pi/catkin_ws/
sudo cp /home/pi/git/intelligente-infrastruktur-fzi-autopark/backend/requirements.txt /home/pi/catkin_ws/
sudo cp /home/pi/git/intelligente-infrastruktur-fzi-autopark/backend/shutdown_script.sh /home/pi/catkin_ws/
sudo cp -r /home/pi/git/intelligente-infrastruktur-fzi-autopark/backend/parking_app_ros_pkg /home/pi/catkin_ws/src/
sudo cp -r /home/pi/git/intelligente-infrastruktur-fzi-autopark/backend/ros_parking_management_msgs /home/pi/catkin_ws/src/
sudo systemctl reboot