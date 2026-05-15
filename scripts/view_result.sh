#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/rm_ws/install/setup.bash

ros2 run rqt_image_view rqt_image_view /detector/result_img
