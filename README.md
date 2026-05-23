# RM Vision Virtual Reproduction

本项目基于 RoboMaster 开源视觉项目 rm_vision / rm_auto_aim，完成了无实体小车、无工业相机条件下的 armor_detector 复现。

## 1. 项目目标

在 Ubuntu 22.04 + ROS 2 Humble 环境下，通过自写虚拟相机节点，将普通摄像头或本地视频转换为 ROS 2 图像话题 `/image_raw`，接入官方 `armor_detector`，实现装甲板检测链路的最小复现。

## 2. 项目目录结构

## 项目结构

## 项目结构

```text
rm_ws/
├── README.md                         # 项目总说明：目标、环境、运行方式、个人贡献
├── .gitignore                        # 忽略 build/install/log、视频、rosbag 等文件
│
├── docs/                             # 复现文档与分享会材料
│   ├── 01_project_overview.md         # 项目背景、总体目标、工作空间结构
│   ├── 02_virtual_camera_node.md      # my_vision_node 虚拟相机节点说明
│   ├── 03_camera_info_and_pnp.md      # camera_info 与 PnP 解算关系
│   ├── 04_armor_detector_pipeline.md # 官方 armor_detector 源码与算法流程
│   ├── 05_common_errors.md            # 复现过程常见错误与解决方案
│   ├── 06_simple_armor_detector.md   # 手写简化版 detector 的设计与实现
│   └── 07_experiment_result.md        # 实验截图、topic 检查、检测效果记录
│
├── scripts/                          # 辅助脚本
│   ├── check_topics.sh                # 检查 /image_raw、/camera_info、detector 话题
│   ├── view_result.sh                 # 查看官方 /detector/result_img
│   └── view_simple_result.sh          # 查看手写版 /simple_detector/result_img
│
├── src/                              # ROS 2 源码目录
│   ├── my_vision_node/                # 自写虚拟相机与 camera_info 功能包
│   │   ├── config/
│   │   │   └── virtual_camera.yaml     # 摄像头/视频输入、camera_info 参数
│   │   ├── launch/
│   │   │   └── virtual_detector.launch.py
│   │   ├── src/
│   │   │   ├── vision_node.cpp         # 摄像头/MP4/转码视频 → /image_raw
│   │   │   └── camera_info_node.cpp    # 虚拟相机内参 → /camera_info
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── simple_armor_detector/         # 手写简化版装甲板检测器
│   │   ├── include/
│   │   │   └── simple_armor_detector/
│   │   │       ├── armor.hpp           # Light / SimpleArmor 数据结构
│   │   │       ├── detector.hpp        # OpenCV 检测模块声明
│   │   │       ├── pnp_solver.hpp      # PnP 位姿解算模块声明
│   │   │       └── simple_detector_node.hpp # ROS 2 节点声明
│   │   ├── src/
│   │   │   ├── detector.cpp            # 二值化、灯条检测、装甲板匹配、绘图
│   │   │   ├── pnp_solver.cpp          # cv::solvePnP 位姿解算
│   │   │   └── simple_detector_node.cpp # 订阅/发布、模块调度
│   │   ├── config/
│   │   │   └── simple_detector.yaml    # 阈值、灯条筛选、PnP 参数
│   │   ├── launch/
│   │   │   └── simple_detector.launch.py
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── README.md
│   │
│   ├── rm_auto_aim/                   # 官方/叉取的自瞄核心算法模块
│   │   ├── armor_detector/             # 官方装甲板检测模块
│   │   ├── armor_tracker/              # 官方追踪模块
│   │   ├── auto_aim_interfaces/        # 官方自定义消息
│   │   └── rm_auto_aim/                # 自瞄模块入口
│   │
│   └── rm_vision/                     # 官方/叉取的 bringup 与配置参考
│       └── rm_vision_bringup/
│
├── build/                             # colcon 自动生成，不提交
├── install/                           # colcon 自动生成，不提交
└── log/                               # colcon 自动生成，不提交

```

## 3.启动测试

```markdown

### 编译

```bash

cd ~/rm_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install

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
### 终端 3：查看检测效果图

```bash
#结果图
ros2 run rqt_image_view rqt_image_view /simple_detector/result_img
#二值图
ros2 run rqt_image_view rqt_image_view /simple_detector/binary_img
#Marker输出
ros2 topic echo /simple_detector/markers --once

```
