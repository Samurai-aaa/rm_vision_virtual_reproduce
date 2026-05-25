#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source ~/rm_ws/install/setup.bash

echo "========== /image_raw =========="
ros2 topic info /image_raw || true

echo
echo "========== /camera_info =========="
ros2 topic info /camera_info || true

echo
echo "========== simple topics =========="
ros2 topic list | grep simple || true

echo
echo "========== detector topics =========="
ros2 topic list | grep detector || true

echo
echo "========== tracker topics =========="
ros2 topic list | grep tracker || true
