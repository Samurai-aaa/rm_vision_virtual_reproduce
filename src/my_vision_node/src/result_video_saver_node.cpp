#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ResultVideoSaverNode : public rclcpp::Node
{
public:
  ResultVideoSaverNode() : Node("result_video_saver_node")
  {
    this->declare_parameter<std::string>("image_topic", "/detector/result_img");
    this->declare_parameter<std::string>("output_path", "/home/samurai/Videos/detector_result.mp4");
    this->declare_parameter<int>("fps", 30);

    image_topic_ = this->get_parameter("image_topic").as_string();
    output_path_ = this->get_parameter("output_path").as_string();
    fps_ = this->get_parameter("fps").as_int();

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      10,
      std::bind(&ResultVideoSaverNode::imageCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Saving topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output video: %s", output_path_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;

    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    if (frame.empty()) {
      return;
    }

    if (!writer_.isOpened()) {
      int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
      bool ok = writer_.open(
        output_path_,
        fourcc,
        static_cast<double>(fps_),
        cv::Size(frame.cols, frame.rows)
      );

      if (!ok) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video writer: %s", output_path_.c_str());
        return;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Video writer opened: %s, size=%dx%d, fps=%d",
        output_path_.c_str(),
        frame.cols,
        frame.rows,
        fps_
      );
    }

    writer_.write(frame);
  }

private:
  std::string image_topic_;
  std::string output_path_;
  int fps_;

  cv::VideoWriter writer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ResultVideoSaverNode>());
  rclcpp::shutdown();
  return 0;
}
