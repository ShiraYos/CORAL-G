#!/bin/bash
# Usage: source ~/CORAL-G/source_coral.sh <ROS_DOMAIN_ID>

if [ -z "$1" ]; then
  echo "Usage: source source_coral.sh <ROS_DOMAIN_ID>"
  return 1
fi

source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/CORAL-G/install/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=$1
echo "CORAL-G environment ready. ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
