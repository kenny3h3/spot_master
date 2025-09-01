#!/bin/bash
# ROS 2 Environment für Raspi

# ROS2 Base
source /opt/ros/jazzy/setup.bash

# Workspace smov sourcen
#source ~/spot_ws/install/setup.bash

# Netzwerkeinstellungen
export ROS_DOMAIN_ID=13
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_DISCOVERY_SERVER=udp://192.168.8.174:11811

echo "[ROS2 Raspi] Environment geladen (Domain=$ROS_DOMAIN_ID)"
