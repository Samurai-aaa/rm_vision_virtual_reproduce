#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraInfoNode : public rclcpp::Node
{
public:
  CameraInfoNode() : Node("virtual_camera_info_node")
  {  
    // 相机坐标系名称
    this->declare_parameter<std::string>("frame_id", "camera");

    // 图像宽高
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);

    // 相机内参
    this->declare_parameter<double>("fx", 600.0);   // fx, fy 是焦距
    this->declare_parameter<double>("fy", 600.0);
    this->declare_parameter<double>("cx", 320.0);   // cx, cy 是主点坐标，通常是图像中心
    this->declare_parameter<double>("cy", 240.0);

    // 发布频率
    this->declare_parameter<double>("fps", 30.0);

    // 从 ROS 2 参数系统读取最终参数值
    frame_id_ = this->get_parameter("frame_id").as_string();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fx_ = this->get_parameter("fx").as_double();
    fy_ = this->get_parameter("fy").as_double();
    cx_ = this->get_parameter("cx").as_double();
    cy_ = this->get_parameter("cy").as_double();
    fps_ = this->get_parameter("fps").as_double();

    // 创建 /camera_info 发布者
    publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);   // armor_detector 会订阅这个话题，用其中的 K 矩阵和畸变参数初始化 PnP

    // 根据 fps 创建定时器
    // 每触发一次 timerCallback，就读取并发布一帧图像
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps_));     // 创建一个以毫秒为单位的时间尺度,时长为1000.0/fps_),并转化成整形
    timer_ = this->create_wall_timer(                                                   // 创建一个基于真实时间的定时器,每隔 period 时间，调用一次当前对象的 timerCallback 函数
      period,
      std::bind(&CameraInfoNode::timerCallback, this)
    );

    // 日志输出
    RCLCPP_INFO(this->get_logger(), "Virtual camera_info publisher started.");
  }

private:
  // 定时器回调函数，负责创建并发布 CameraInfo 消息
  void timerCallback()
  {
    sensor_msgs::msg::CameraInfo msg;     // 创建一个 CameraInfo 消息对象

    msg.header.stamp = this->now();       // 设置消息的时间戳为当前时间
    msg.header.frame_id = frame_id_;      // 设置消息的坐标系名称

    msg.width = width_;                   // 设置图像宽度
    msg.height = height_;                 // 设置图像高度

    msg.distortion_model = "plumb_bob";   // 设置畸变模型，这里使用常见的 "plumb_bob" 模型，表示使用径向和切向畸变参数
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};    // 设置畸变参数，这里假设没有畸变，所以全部设置为0.0。如果有实际的畸变参数，可以根据相机标定结果进行设置。

    msg.k = {                             // 设置相机内参矩阵 K，包含焦距和主点坐标
      fx_, 0.0, cx_,
      0.0, fy_, cy_,
      0.0, 0.0, 1.0
    };

    msg.r = {                             // 设置相机旋转矩阵 R，这里假设相机没有旋转，所以设置为单位矩阵。如果相机有实际的旋转，可以根据需要进行设置。
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    msg.p = {                             // 设置相机投影矩阵 P，包含内参和外参信息。这里假设相机没有外参偏移，所以设置为内参矩阵的扩展形式。如果有实际的外参偏移，可以根据需要进行设置。 
      fx_, 0.0, cx_, 0.0,
      0.0, fy_, cy_, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    publisher_->publish(msg);             // 发布 CameraInfo 消息到指定的话题
  }

private:
  std::string frame_id_;   // 图像坐标系名称
  int width_;              // 图像宽度
  int height_;             // 图像高度
  double fx_;              // 焦距
  double fy_;
  double cx_;              // 主点坐标
  double cy_;
  double fps_;             // 发布频率

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;  // ROS 2 图像信息发布者对象，用于将相机内参消息发布到 ROS 2 网络中
  rclcpp::TimerBase::SharedPtr timer_;                                    // ROS 2 定时器对象，用于定期调用 timerCallback 函数读取并发布相机内参
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                           // 初始化 ROS 2 C++ 客户端库
  rclcpp::spin(std::make_shared<CameraInfoNode>());   // 创建 CameraInfoNode 节点对象，并进入 ROS 2 事件循环
  rclcpp::shutdown();                                 // 关闭 ROS 2，释放相关资源
  return 0;
}
