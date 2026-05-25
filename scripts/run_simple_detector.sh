#!/bin/bash
set -e

cd ~/rm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch simple_armor_detector simple_detector.launch.py
