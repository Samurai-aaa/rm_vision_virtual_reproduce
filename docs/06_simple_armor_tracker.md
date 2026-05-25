
---

### `06_simple_armor_tracker.md`

```markdown
# simple_armor_tracker

`simple_armor_tracker` 是手写简化版目标跟踪器，参考官方 `armor_tracker` 的结构和思想。

## 输入

```text
/simple_detector/armors
simple_armor_interfaces/msg/SimpleArmors

```
## 输出

```text
/simple_tracker/target
geometry_msgs/msg/PoseStamped

/simple_tracker/marker
visualization_msgs/msg/Marker

```
## 核心流程

```text
接收 SimpleArmors
    ↓
过滤低置信度 armor
    ↓
提取有 pose 的目标
    ↓
EKF predict
    ↓
选择距离预测位置最近的观测
    ↓
EKF update
    ↓
发布稳定 target