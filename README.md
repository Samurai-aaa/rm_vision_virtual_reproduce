# RM Vision Virtual Reproduction

本项目基于 RoboMaster 开源视觉项目 rm_vision / rm_auto_aim，完成了无实体小车、无工业相机条件下的 armor_detector 复现。

## 1. 项目目标

在 Ubuntu 22.04 + ROS 2 Humble 环境下，通过自写虚拟相机节点，将普通摄像头或本地视频转换为 ROS 2 图像话题 `/image_raw`，接入 `auto_aim`复现模块，实现装甲板检测链路的最小复现。

## 2. 项目目录结构

```text
rm_ws/
├── README.md
├── .gitignore
│
├──docs
│   ├── 01_technology_report.md
│   ├── 02_virtual_camera_node.md
│   ├── 03_simple_armor_detector.md.md
│   ├── 04_camera_info_and_pnp.md
│   ├── 05_simple_armor_interfaces.md
│   ├── 06_simple_armor_tracker.md
│   └── image.png
│
│
├──scripts
│   ├── build_simple.sh
│   ├── check_topics.sh
│   ├── echo_simple_armors.sh
│   ├── echo_tracker_target.sh
│   ├── record_bag.sh
│   ├── run_camera.sh
│   ├── run_simple_detector.sh
│   ├── run_simple_tracker.sh
│   └── view_simple_result.sh
│
│
├── src/
│   ├── my_vision_node/                       // 自写摄像头/视频发布模块
│   │   ├── config/
│   │   │   └── virtual_camera.yaml
│   │   ├── launch/
│   │   │   └── virtual_detector.launch.py
│   │   ├── src/
│   │   │   ├── vision_node.cpp          # 摄像头/MP4/转码视频 → /image_raw
│   │   │   └── camera_info_node.cpp     # 虚拟相机内参 → /camera_info
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── simple_armor_interfaces/              // 自写interfaces模块
│   │   ├── msg/
│   │   │   ├── Light.msg                # 单个灯条信息：top、bottom、center、宽高、角度
│   │   │   ├── SimpleArmor.msg          # 单个装甲板信息：type、color、confidence、角点、pose 等
│   │   │   └── SimpleArmors.msg         # 一帧中的所有装甲板数组
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── simple_armor_detector/                // 自写armor_detector模块
│   │   ├── include/
│   │   │   └── simple_armor_detector/
│   │   │       ├── armor.hpp            # Light / SimpleArmor 数据结构
│   │   │       ├── detector.hpp         # OpenCV 检测模块声明
│   │   │       ├── pnp_solver.hpp       # PnP 位姿解算模块声明
│   │   │       └── simple_detector_node.hpp  # ROS 2 节点声明
│   │   ├── src/
│   │   │   ├── detector.cpp             # 二值化、灯条检测、装甲板匹配、绘图
│   │   │   ├── pnp_solver.cpp           # cv::solvePnP 位姿解算
│   │   │   └── simple_detector_node.cpp # 订阅/发布、模块调度
│   │   ├── config/
│   │   │   └── simple_detector.yaml
│   │   ├── launch/
│   │   │   └── simple_detector.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── simple_armor_tracker/                 // 自写tracker模块
│   │   ├── include/
│   │   │   └── simple_armor_tracker/
│   │   │       ├── extended_kalman_filter.hpp   # Kalman Filter 声明
│   │   │       ├── tracker.hpp                  # Tracker 状态机、目标选择、滤波逻辑声明
│   │   │       └── tracker_node.hpp             # ROS 2 节点声明
│   │   ├── src/ 
│   │   │   ├── extended_kalman_filter.cpp       # 6维状态 x,y,z,vx,vy,vz；3维观测 x,y,z
│   │   │   ├── tracker.cpp                      # LOST/DETECTING/TRACKING/TEMP_LOST 状态机
│   │   │   └── tracker_node.cpp                 # 订阅 /simple_detector/armors，发布 /simple_tracker/target 
│   │   ├── config/
│   │   │   └── simple_tracker.yaml
│   │   ├── launch/
│   │   │   └── simple_tracker.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── rm_auto_aim/                         // 官方auto_aim模块（对比用，未上传）
│   │   ├── armor_detector/
│   │   ├── armor_tracker/
│   │   ├── auto_aim_interfaces/
│   │   └── rm_auto_aim/
│   │
│   └── rm_vision/                           // 官方bringup模块与配置参考（未上传）
│       └── rm_vision_bringup/
│
├── build/          # colcon 自动生成，不提交
├── install/        # colcon 自动生成，不提交
└── log/            # colcon 自动生成，不提交
```

## 3.启动测试

### 编译

```bash

cd ~/rm_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select \
  simple_armor_interfaces \
  simple_armor_detector \
  simple_armor_tracker \
  --symlink-install

source install/setup.bash

```

### 终端 1：启动虚拟相机、camera_info 和官方 armor_detector

```bash

cd ~/rm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch my_vision_node virtual_detector.launch.py

```
### 终端 2：启动手写版 simple_armor_detector

```bash

cd ~/rm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch simple_armor_detector simple_detector.launch.py

```
### 终端 3：启动 simple_armor_tracker

```bash

cd ~/rm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch simple_armor_tracker simple_tracker.launch.py

```
### 终端 4：查看检测效果图

```bash

#结果图
ros2 run rqt_image_view rqt_image_view /simple_detector/result_img
#二值图
ros2 run rqt_image_view rqt_image_view /simple_detector/binary_img
#Marker输出
ros2 topic echo /simple_detector/markers --once

```
