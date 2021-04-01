#!/bin/bash
FILE=./src/parking_app_ros_pkg/bin/config.properties
echo "$ipAddress" "$port"
IP=$(grep -i 'ipAddress' $FILE  | cut -f2 -d'=')
Port=$(grep -i 'port' $FILE  | cut -f2 -d'=')
URL="http://$IP:$Port/shutdown"
echo "$URL"
curl -ls $URL | head -1

killall -9 rosmaster
