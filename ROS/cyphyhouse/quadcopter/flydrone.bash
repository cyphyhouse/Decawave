#!/bin/bash
sudo systemctl start arducopter
sleep 5
source catkin_ws/devel/setup.bash
roslaunch quadcopter cyphyhouse.launch
