######################################
# install ros noetic ca. 58:30 min
cd
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y python-rosdep ros-geometry-msgs python-rosinstall python-rosinstall-generator python-wstool build-essential python-pip cmake
sudo pip install -U rosdep
sudo rosdep init
rosdep update
sudo mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
rosinstall_generator ros_comm --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall
wstool init src noetic-ros_comm-wet.rosinstall
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
printf "source /opt/ros/noetic/setup.bash" | sudo tee ~/.bashrc
source ~/.bashrc
cd

######################################
# install flask ca. 1 min
pip install Flask
pip install sqlalchemy


######################################
# install wifi hotspot ca. 1 min
sudo apt install -y hostapd
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo apt install -y dnsmasq
sudo DEBIAN_FRONTEND=noninteractive apt install -y netfilter-persistent iptables-persistent
#printf "\ninterface wlan0 \n    static ip_address=192.168.4.1/24 \n    nohook wpa_supplicant" | sudo tee -a /etc/dhcpcd.conf
printf "# https://www.raspberrypi.org/documentation/configuration/wireless/access-point-routed.md\n# Enable IPv4 routing\nnet.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.d/routed-ap.conf
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo netfilter-persistent save
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
printf "interface=wlan0\ndhcp-range=192.168.4.2,192.168.4.100,255.255.255.0,12h\ndomain=wlan\naddress=/gw.wlan/192.168.4.1" | sudo tee /etc/dnsmasq.conf
sudo apt install -y rfkill
sudo rfkill unblock wlan
printf "country_code=DE\ninterface=wlan0\nssid=Parkhaus\nhw_mode=b\nchannel=7\nmacaddr_acl=0\nauth_algs=1\nignore_broadcast_ssid=0" | sudo tee /etc/hostapd/hostapd.conf


######################################
# install openvpn
sudo apt-get install -y openvpn unzip
export DEBIAN_FRONTEND=noninteractive DEBCONF_NONINTERACTIVE_SEEN=true
printf "tzdata tzdata/Areas select Europe\ntzdata tzdata/Zones/Europe select Berlin" | sudo tee /home/pi/preseed.txt
sudo debconf-set-selections /home/pi/preseed.txt
sudo rm /etc/timezone
sudo rm /etc/localtime
sudo dpkg-reconfigure -f noninteractive tzdata
cd /tmp && wget https://files.ovpn.com/raspbian/ovpn-de-frankfurt.zip && unzip ovpn-de-frankfurt.zip && mkdir -p /etc/openvpn && mv config/* /etc/openvpn && chmod +x /etc/openvpn/update-resolv-conf && rm -rf config && rm -f ovpn-de-frankfurt.zip 
sudo cp /home/pi/client.ovpn /etc/openvpn
sudo openvpn client.ovpn
# type in username and password


######################################
# install git
sudo apt install -y git
mkdir /home/pi/git/
cd git
sudo git clone https://git.scc.kit.edu/teamprojekt-intelligente-infrastruktur-2020-21-fzi/intelligentinfrastructuremobileapp.git
#//https://ids-git.fzi.de/qm821/intelligente-infrastruktur-fzi-autopark.git
sudo cp /home/pi/git/intelligentinfrastructuremobileapp/backend/setup_script.sh /home/pi/catkin_ws/
sudo cp /home/pi/git/intelligentinfrastructuremobileapp/backend/setup_script_standalone.sh /home/pi/catkin_ws/
sudo cp /home/pi/git/intelligentinfrastructuremobileapp/backend/requirements.txt /home/pi/catkin_ws/
sudo cp /home/pi/git/intelligentinfrastructuremobileapp/backend/shutdown_script.sh /home/pi/catkin_ws/
sudo cp -r /home/pi/git/intelligentinfrastructuremobileapp/backend/parking_app_ros_pkg /home/pi/catkin_ws/src/
sudo cp -r /home/pi/git/intelligentinfrastructuremobileapp/backend/ros_parking_management_msgs /home/pi/catkin_ws/src/
cd
cd /home/pi/catkin_ws/src
git clone  https://github.com/ros/common_msgs.git 
sudo systemctl reboot



