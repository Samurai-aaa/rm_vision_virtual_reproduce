# RM Vision Virtual Reproduction

本项目基于 RoboMaster 开源视觉项目 rm_vision / rm_auto_aim，完成了无实体小车、无工业相机条件下的 armor_detector 复现。

## 1. 项目目标

在 Ubuntu 22.04 + ROS 2 Humble 环境下，通过自写虚拟相机节点，将普通摄像头或本地视频转换为 ROS 2 图像话题 `/image_raw`，接入官方 `armor_detector`，实现装甲板检测链路的最小复现。

## 2. 项目结构

```text
rm_ws/
├── src/
│   ├── my_vision_node/      # 自写虚拟相机与 camera_info 节点
│   ├── rm_auto_aim/
│   └── rm_vision/
├── docs/
├── scripts/
├── build/
├── install/
└── log/