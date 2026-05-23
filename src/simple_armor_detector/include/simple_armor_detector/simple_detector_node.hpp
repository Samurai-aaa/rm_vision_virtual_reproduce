#ifndef SIMPLE_ARMOR_DETECTOR__SIMPLE_DETECTOR_NODE_HPP_
#define SIMPLE_ARMOR_DETECTOR__SIMPLE_DETECTOR_NODE_HPP_

#include "simple_armor_detector/detector.hpp"
#include "simple_armor_detector/pnp_solver.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace simple_armor_detector
{
// ROS2节点类，负责订阅图像和相机信息，发布检测结果和位姿信息
class SimpleArmorDetectorNode : public rclcpp::Node
{
public:
  SimpleArmorDetectorNode();

private:
  void declareParams();

  void loadParams();

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  void publishImage(
    image_transport::Publisher & pub,
    const cv::Mat & image,
    const std_msgs::msg::Header & header,
    const std::string & encoding);

  void publishMarkers(
    const std_msgs::msg::Header & header,
    const std::vector<SimpleArmor> & armors);

  void publishPoses(
    const std_msgs::msg::Header & header,
    const std::vector<SimpleArmor> & armors);

private:
  std::string image_topic_;
  std::string camera_info_topic_;

  bool enable_pnp_;

  Detector::Params detector_params_;
  PnPSolver::Params pnp_params_;

  std::unique_ptr<Detector> detector_;
  std::unique_ptr<PnPSolver> pnp_solver_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  image_transport::Publisher binary_pub_;
  image_transport::Publisher result_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
};

}

#endif
