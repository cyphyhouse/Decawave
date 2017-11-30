#!/bin/bash
sudo systemctl start arducopter
sleep 5
roscore &
sleep 5
rosrun mavros mavros_node _fcu_url:=udp://:14650@ _gcs_url:=udp://@127.0.0.1:14550 &
roslaunch vrpn_client_ros sample.launch server:=192.168.1.3 &
sleep 5
source catkin_ws/devel/setup.bash
sleep 5
rosrun quadcopter fakeGPS_node
