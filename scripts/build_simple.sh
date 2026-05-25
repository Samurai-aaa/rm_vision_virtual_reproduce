#!/bin/bash
set -e

cd ~/rm_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select \
  simple_armor_interfaces \
  simple_armor_detector \
  simple_armor_tracker \
  --symlink-install

source install/setup.bash

echo "Build simple armor pipeline finished."