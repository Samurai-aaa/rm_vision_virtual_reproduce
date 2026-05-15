#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraInfoNode : public rclcpp::Node
{
public:
  CameraInfoNode() : Node("virtual_camera_info_node")
  {
    this->declare_parameter<std::string>("frame_id", "camera");
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<double>("fx", 600.0);
    this->declare_parameter<double>("fy", 600.0);
    this->declare_parameter<double>("cx", 320.0);
    this->declare_parameter<double>("cy", 240.0);
    this->declare_parameter<double>("fps", 30.0);

    frame_id_ = this->get_parameter("frame_id").as_string();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fx_ = this->get_parameter("fx").as_double();
    fy_ = this->get_parameter("fy").as_double();
    cx_ = this->get_parameter("cx").as_double();
    cy_ = this->get_parameter("cy").as_double();
    fps_ = this->get_parameter("fps").as_double();

    pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);

    auto period_ms = static_cast<int>(1000.0 / fps_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CameraInfoNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Virtual camera_info publisher started.");
  }

private:
  void timerCallback()
  {
    sensor_msgs::msg::CameraInfo msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    msg.width = width_;
    msg.height = height_;

    msg.distortion_model = "plumb_bob";
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    msg.k = {
      fx_, 0.0, cx_,
      0.0, fy_, cy_,
      0.0, 0.0, 1.0
    };

    msg.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    msg.p = {
      fx_, 0.0, cx_, 0.0,
      0.0, fy_, cy_, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    pub_->publish(msg);
  }

private:
  std::string frame_id_;
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double fps_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoNode>());
  rclcpp::shutdown();
  return 0;
}
