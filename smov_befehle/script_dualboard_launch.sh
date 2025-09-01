#!/bin/bash

echo "Source ROS"
source /opt/ros/jazzy/setup.bash

echo "Source Install"
source /home/winston/smov/install/setup.bash

echo "ROS dualboard launch"
cd /home/winston/smov
ros2 launch smov_bringup dual_board.launch.py
