# 01 Project Overview

## 项目背景

本项目基于 RoboMaster 开源视觉项目 rm_vision / rm_auto_aim，目标是在没有实体小车、没有工业相机的情况下，复现 armor_detector 装甲板检测流程。

当前环境：

- Ubuntu 22.04
- ROS 2 Humble
- OpenCV
- NVIDIA GPU
- 普通 USB 摄像头 / 本地 MP4 视频

## 项目目标

通过自写虚拟相机节点，将普通摄像头或本地视频转换为 ROS 2 图像话题 `/image_raw`，并接入官方 `armor_detector` 节点，实现无硬件条件下的装甲板检测链路复现。

## 当前完成内容

- 编写 `vision_node.cpp`
  - 使用 OpenCV 读取摄像头或视频
  - 使用 cv_bridge 转换为 ROS 2 Image 消息
  - 发布 `/image_raw`

- 编写 `camera_info_node.cpp`
  - 发布 `/camera_info`
  - 提供 PnP 所需相机内参

- 编写 `virtual_detector.launch.py`
  - 一键启动虚拟相机节点
  - 一键启动 camera_info 节点
  - 一键启动官方 armor_detector 节点

- 编写 `virtual_camera.yaml`
  - 集中管理摄像头、分辨率、fps、相机内参等参数

## 数据流

```text
USB Camera / MP4 Video
        ↓
vision_node
        ↓
/image_raw
        ↓
armor_detector
        ↓
/detector/result_img
/detector/binary_img
/detector/armors
