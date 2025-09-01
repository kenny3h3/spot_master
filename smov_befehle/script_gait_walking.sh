#!/bin/bash

echo "Source ROS"
source /opt/ros/jazzy/setup.bash

echo "Source Install"
source /home/winston/spot_ws/install/setup.bash

echo "Run Gait 'Walking'"
cd /home/winston/spot_ws
ros2 run walking_gait state
