#!/bin/bash

echo "Source ROS"
source /opt/ros/jazzy/setup.bash

echo "Source Install"
source /home/winston/smov/install/setup.bash

echo "Run Gait 'Walking'"
cd /home/winston/smov
ros2 run walking_gait state
