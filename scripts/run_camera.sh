#!/bin/bash
set -e

cd ~/rm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch my_vision_node virtual_detector.launch.py
