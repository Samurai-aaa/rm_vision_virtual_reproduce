# RM Vision Virtual Reproduction

本项目基于 RoboMaster 开源视觉项目 rm_vision / rm_auto_aim，完成了无实体小车、无工业相机条件下的 armor_detector 复现。

## 1. 项目目标

在 Ubuntu 22.04 + ROS 2 Humble 环境下，通过自写虚拟相机节点，将普通摄像头或本地视频转换为 ROS 2 图像话题 `/image_raw`，接入官方 `armor_detector`，实现装甲板检测链路的最小复现。

## 2. 项目结构

## Project Structure

rm_ws/
├── src/                          # ROS 2 workspace source directory
│   ├── my_vision_node/           # 自写虚拟相机功能包，发布 /image_raw 和 /camera_info
│   ├── rm_auto_aim/              # 官方自瞄核心代码，包含 armor_detector 等模块
│   └── rm_vision/                # 官方 bringup/config 参考工程
├── docs/                         # 复现文档、节点说明、常见问题记录
├── scripts/                      # 快捷脚本，例如 topic 检查和结果图查看
├── build/                        # colcon 自动生成的编译中间目录，不提交
├── install/                      # colcon 自动生成的安装目录，不提交
└── log/                          # colcon 自动生成的编译日志目录，不提交