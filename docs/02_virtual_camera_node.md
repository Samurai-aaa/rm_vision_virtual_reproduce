
---

### `02_virtual_camera_node.md`

虚拟相机

```markdown
# Virtual Camera Node

`my_vision_node` 用于替代真实工业相机。

## vision_node

输入：

```text
普通摄像头 / MP4 视频

输出：

```text
/image_raw
sensor_msgs/msg/Image

cv::VideoCapture
    ↓
cv::Mat
    ↓
cv_bridge
    ↓
sensor_msgs/msg/Image