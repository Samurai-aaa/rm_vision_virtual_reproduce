#include <cctype>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

// 1. 使用 OpenCV 从本地摄像头或视频文件中读取图像
// 2. 将 OpenCV 的 cv::Mat 图像转换为 ROS 2 的 sensor_msgs/msg/Image
// 3. 发布到 /image_raw，作为 rm_vision armor_detector 的输入
class VirtualCameraNode : public rclcpp::Node
{
public:
  VirtualCameraNode()
  : rclcpp::Node("virtual_camera_node")
  {
    // 声明节点参数，并为每个参数设置默认值
    this->declare_parameter<std::string>("source", "0");               // 摄像头编号或视频文件路径
    this->declare_parameter<std::string>("topic_name", "/image_raw");  // 图像发布话题
    this->declare_parameter<std::string>("frame_id", "camera");        // 图像坐标系名称
    this->declare_parameter<int>("fps", 30);                           // 图像发布帧率
    this->declare_parameter<int>("width", 0);                          // 期望图像宽度
    this->declare_parameter<int>("height", 0);                         // 期望图像高度
    this->declare_parameter<bool>("loop_video", true);                 // 视频文件播放结束后是否循环

    // 从 ROS 2 参数系统读取最终参数值
    source_ = this->get_parameter("source").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    fps_ = this->get_parameter("fps").as_int();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    loop_video_ = this->get_parameter("loop_video").as_bool();

    // 防止 fps 配置为 0 或负数，避免定时器周期非法
    if (fps_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "Invalid fps=%d, reset to 30", fps_);
      fps_ = 30;
    }

    // 打开摄像头或视频文件
    openCapture();

    // 创建图像发布者
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);

    // 根据 fps 创建定时器
    // 每触发一次 timerCallback，就读取并发布一帧图像
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps_));     // 创建一个以毫秒为单位的时间尺度,时长为1000.0/fps_),并转化成整形
    timer_ = this->create_wall_timer(                                                   // 创建一个基于真实时间的定时器,每隔 period 时间，调用一次当前对象的 timerCallback 函数
      period,
      std::bind(&VirtualCameraNode::timerCallback, this)                                //将成员函数yimerCallback与对象this绑定，创建一个可调用对象，当定时器触发时调用该函数
    );

    // 日志输出
    RCLCPP_INFO(this->get_logger(), "Virtual camera node started.");
    RCLCPP_INFO(this->get_logger(), "Source: %s", source_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publish topic: %s", topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "FPS: %d", fps_);
  }

private:
  //打开摄像头或视频文件
  void openCapture()
  {
    if (isInteger(source_)) {
      int camera_id = std::stoi(source_);
      cap_.open(camera_id, cv::CAP_ANY);

      if (width_ > 0) {
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
      }
      if (height_ > 0) {
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
      }

      RCLCPP_INFO(this->get_logger(), "Opening camera device: %d", camera_id);
    } else {
      cap_.open(source_);
      RCLCPP_INFO(this->get_logger(), "Opening video file / stream: %s", source_.c_str());
    }

    if (!cap_.isOpened()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open source: %s", source_.c_str());
      throw std::runtime_error("Failed to open video source");
    }
  }

  //判断是否是整数,如果是整数，说明是摄像头编号；如果不是整数，说明是视频文件路径
  bool isInteger(const std::string & str)
  {
    if (str.empty()) {
      return false;
    }

    size_t start = 0;
    if (str[0] == '-' || str[0] == '+') {
      start = 1;
    }

    if (start >= str.size()) {
      return false;
    }

    for (size_t i = start; i < str.size(); ++i) {
      if (!std::isdigit(static_cast<unsigned char>(str[i]))) {
        return false;
      }
    }

    return true;
  }

  // 定时器回调函数，负责创建并发布 Image 消息
  void timerCallback()
  {
    cv::Mat frame;
    cap_ >> frame;                                 // 从视频流中读取一帧图像，并存储在 frame 变量中

    if (frame.empty()) {
      if (!isInteger(source_) && loop_video_) {    // 如果输入源是视频文件，并且允许循环播放，则跳回第一帧
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Video ended, looping..."
        );
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);      // 表示把视频当前播放位置设置到第 0 帧
        cap_ >> frame;
      }

      if (frame.empty()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Empty frame received."
        );
        return;
      }
    }

    std_msgs::msg::Header header;                  // 创建一个 ROS 2 消息头对象
    header.stamp = this->now();                    // 设置消息头的时间戳为当前时间
    header.frame_id = frame_id_;                   // 设置消息头的坐标系名称为 frame_id_

    sensor_msgs::msg::Image::SharedPtr msg =       // 将 OpenCV 的 cv::Mat 图像转换为 ROS 2 的 sensor_msgs/msg/Image 消息
      cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    publisher_->publish(*msg);                     // 发布图像消息到指定的话题
  }

private:
  std::string source_;
  std::string topic_name_;
  std::string frame_id_;
  int fps_;
  int width_;
  int height_;
  bool loop_video_;

  cv::VideoCapture cap_;                                                 // OpenCV 视频捕获对象，用于从摄像头或视频文件中读取图像

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;      // ROS 2 图像发布者对象，用于将图像消息发布到 ROS 2 网络中  
  rclcpp::TimerBase::SharedPtr timer_;                                   // ROS 2 定时器对象，用于定期调用 timerCallback 函数读取并发布图像
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                                 // 初始化 ROS 2 C++ 客户端库

  try {
    auto node = std::make_shared<VirtualCameraNode>();      // 创建虚拟相机节点对象
    rclcpp::spin(node);                                     // 进入 ROS 2 事件循环
  } catch (const std::exception & e) {                      // 捕获构造或运行过程中抛出的异常
    fprintf(stderr, "Exception: %s\n", e.what());
  }

  rclcpp::shutdown();                                       // 关闭 ROS 2，释放相关资源
  return 0;
}