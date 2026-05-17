# 03 Camera Info and PnP

## 文件位置

```text
src/my_vision_node/src/camera_info_node.cpp


camera_info_node
        ↓ 发布 /camera_info
armor_detector
        ↓ 使用 K 和 D 初始化 PnP Solver
        ↓ 输出装甲板 pose
