
---

### `03_simple_armor_detector.md`

这是重点文档。

```markdown
# simple_armor_detector

`simple_armor_detector` 是本项目中手写的简化版装甲板检测器。

## 模块结构

```text
simple_armor_detector/
├── include/simple_armor_detector/
│   ├── armor.hpp
│   ├── detector.hpp
│   ├── pnp_solver.hpp
│   └── simple_detector_node.hpp
├── src/
│   ├── detector.cpp
│   ├── pnp_solver.cpp
│   └── simple_detector_node.cpp

```
## 检测流程

```text
/image_raw
    ↓
B/R 通道差分
    ↓
二值化
    ↓
形态学闭运算 / 膨胀
    ↓
findContours
    ↓
minAreaRect 拟合灯条
    ↓
灯条筛选
    ↓
灯条匹配成装甲板
    ↓
PnP 位姿解算
    ↓
发布 /simple_detector/armors

```
## 输出话题

```text
/simple_detector/binary_img
/simple_detector/result_img
/simple_detector/markers
/simple_detector/poses
/simple_detector/armors

```