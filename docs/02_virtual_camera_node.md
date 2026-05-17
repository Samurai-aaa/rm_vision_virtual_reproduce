# 02 Virtual Camera Node

## 文件位置

```text
src/my_vision_node/src/vision_node.cpp


摄像头 / MP4 视频
        ↓
OpenCV cv::VideoCapture
        ↓
cv::Mat
        ↓
cv_bridge
        ↓
sensor_msgs/msg/Image
        ↓
/image_raw
