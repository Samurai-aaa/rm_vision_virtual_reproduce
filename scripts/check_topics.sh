#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/rm_ws/install/setup.bash

echo "========== Topics =========="
ros2 topic list -t

echo ""
echo "========== /image_raw =========="
ros2 topic info /image_raw

echo ""
echo "========== /camera_info =========="
ros2 topic info /camera_info

echo ""
echo "========== detector topics =========="
ros2 topic list | grep detector
