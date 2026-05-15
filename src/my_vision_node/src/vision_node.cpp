#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class VirtualCameraNode : public rclcpp::Node
{
public:
  VirtualCameraNode()
  : Node("virtual_camera_node")
  {
    this->declare_parameter<std::string>("source", "0");
    this->declare_parameter<std::string>("topic_name", "/image_raw");
    this->declare_parameter<std::string>("frame_id", "camera");
    this->declare_parameter<int>("fps", 30);
    this->declare_parameter<int>("width", 0);
    this->declare_parameter<int>("height", 0);
    this->declare_parameter<bool>("loop_video", true);

    source_ = this->get_parameter("source").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    fps_ = this->get_parameter("fps").as_int();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    loop_video_ = this->get_parameter("loop_video").as_bool();

    if (fps_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "Invalid fps=%d, reset to 30", fps_);
      fps_ = 30;
    }

    openCapture();

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps_));
    timer_ = this->create_wall_timer(
      period,
      std::bind(&VirtualCameraNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Virtual camera node started.");
    RCLCPP_INFO(this->get_logger(), "Source: %s", source_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publish topic: %s", topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "FPS: %d", fps_);
  }

private:
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

  void timerCallback()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
      if (!isInteger(source_) && loop_video_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Video ended, looping..."
        );

        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
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

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = frame_id_;

    sensor_msgs::msg::Image::SharedPtr msg =
      cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    publisher_->publish(*msg);
  }

private:
  std::string source_;
  std::string topic_name_;
  std::string frame_id_;
  int fps_;
  int width_;
  int height_;
  bool loop_video_;

  cv::VideoCapture cap_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<VirtualCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception: %s\n", e.what());
  }

  rclcpp::shutdown();
  return 0;
}