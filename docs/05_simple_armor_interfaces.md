
---

### `05_simple_armor_interfaces.md`

```markdown
# simple_armor_interfaces

`simple_armor_interfaces` 是本项目自定义的 ROS 2 消息接口包，用于连接 `simple_armor_detector` 和 `simple_armor_tracker`。

## 作用

`result_img` 是给人看的可视化图像，tracker 不能直接理解图像上的文字和框。

tracker 需要结构化数据，例如：

- 装甲板类型
- 颜色
- 置信度
- 2D 四角点
- 左右灯条信息
- PnP pose

## 消息结构

```text
Light.msg
SimpleArmor.msg
SimpleArmors.msg