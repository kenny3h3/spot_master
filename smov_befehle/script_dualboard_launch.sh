#!/bin/bash

echo "Source ROS"
source /opt/ros/jazzy/setup.bash

echo "Source Install"
source /home/winston/spot_ws/install/setup.bash

echo "ROS dualboard launch"
cd /home/winston/spot_ws
ros2 launch smov_bringup dual_board.launch.py

echo "IMU launch"
ros2 launch bno085_imu_py imu.launch.py