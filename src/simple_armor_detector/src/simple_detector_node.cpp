#include "simple_armor_detector/simple_detector_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace simple_armor_detector
{
// 构造函数
SimpleArmorDetectorNode::SimpleArmorDetectorNode()
: Node("simple_armor_detector")
{
  declareParams();
  loadParams();

  detector_ = std::make_unique<Detector>(detector_params_);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&SimpleArmorDetectorNode::imageCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&SimpleArmorDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

  binary_pub_ = image_transport::create_publisher(this, "/simple_detector/binary_img");
  result_pub_ = image_transport::create_publisher(this, "/simple_detector/result_img");

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/simple_detector/markers", 10);

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/simple_detector/poses", 10);

  RCLCPP_INFO(this->get_logger(), "simple_armor_detector v2 started.");
  RCLCPP_INFO(this->get_logger(), "Subscribe image topic: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Subscribe camera_info topic: %s", camera_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Enemy color: %s", detector_params_.enemy_color.c_str());
  RCLCPP_INFO(this->get_logger(), "PnP enabled: %s", enable_pnp_ ? "true" : "false");
}

// 声明参数
void SimpleArmorDetectorNode::declareParams()
{
  this->declare_parameter<std::string>("image_topic", "/image_raw");
  this->declare_parameter<std::string>("camera_info_topic", "/camera_info");

  this->declare_parameter<bool>("enable_pnp", true);

  this->declare_parameter<std::string>("enemy_color", "red");
  this->declare_parameter<int>("binary_thres", 50);

  this->declare_parameter<double>("min_light_area", 5.0);
  this->declare_parameter<double>("min_light_ratio", 1.2);
  this->declare_parameter<double>("max_light_angle", 60.0);

  this->declare_parameter<double>("max_angle_diff", 15.0);
  this->declare_parameter<double>("max_height_diff_ratio", 0.5);
  this->declare_parameter<double>("max_y_diff_ratio", 0.6);
  this->declare_parameter<double>("min_armor_ratio", 0.6);
  this->declare_parameter<double>("max_armor_ratio", 4.0);
  this->declare_parameter<double>("max_light_x_distance", 250.0);

  this->declare_parameter<int>("morph_kernel_width", 3);
  this->declare_parameter<int>("morph_kernel_height", 9);

  this->declare_parameter<double>("small_armor_width", 135.0);
  this->declare_parameter<double>("small_armor_height", 55.0);
  this->declare_parameter<double>("large_armor_width", 225.0);
  this->declare_parameter<double>("large_armor_height", 55.0);
}

// 加载参数
void SimpleArmorDetectorNode::loadParams()
{
  image_topic_ = this->get_parameter("image_topic").as_string();
  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();

  enable_pnp_ = this->get_parameter("enable_pnp").as_bool();

  detector_params_.enemy_color = this->get_parameter("enemy_color").as_string();
  detector_params_.binary_thres = this->get_parameter("binary_thres").as_int();

  detector_params_.min_light_area = this->get_parameter("min_light_area").as_double();
  detector_params_.min_light_ratio = this->get_parameter("min_light_ratio").as_double();
  detector_params_.max_light_angle = this->get_parameter("max_light_angle").as_double();

  detector_params_.max_angle_diff = this->get_parameter("max_angle_diff").as_double();
  detector_params_.max_height_diff_ratio =
    this->get_parameter("max_height_diff_ratio").as_double();
  detector_params_.max_y_diff_ratio = this->get_parameter("max_y_diff_ratio").as_double();
  detector_params_.min_armor_ratio = this->get_parameter("min_armor_ratio").as_double();
  detector_params_.max_armor_ratio = this->get_parameter("max_armor_ratio").as_double();
  detector_params_.max_light_x_distance =
    this->get_parameter("max_light_x_distance").as_double();

  detector_params_.morph_kernel_width = this->get_parameter("morph_kernel_width").as_int();
  detector_params_.morph_kernel_height = this->get_parameter("morph_kernel_height").as_int();

  pnp_params_.small_armor_width = this->get_parameter("small_armor_width").as_double();
  pnp_params_.small_armor_height = this->get_parameter("small_armor_height").as_double();
  pnp_params_.large_armor_width = this->get_parameter("large_armor_width").as_double();
  pnp_params_.large_armor_height = this->get_parameter("large_armor_height").as_double();
}

// 计算图像点到图像中心的距离
void SimpleArmorDetectorNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  if (!enable_pnp_) {
    return;
  }

  pnp_solver_ = std::make_unique<PnPSolver>(msg->k, msg->d, pnp_params_);

  RCLCPP_INFO(this->get_logger(), "Camera info received. PnP solver initialized.");

  // 收到一次 camera_info 后即可初始化 PnP，后续不需要持续订阅
  camera_info_sub_.reset();
}

// 图像回调函数，处理图像并发布结果
void SimpleArmorDetectorNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv::Mat img;

  try {
    img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (img.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty image.");
    return;
  }

  cv::Mat binary = detector_->preprocessImage(img);
  std::vector<Light> lights = detector_->findLights(binary);
  std::vector<SimpleArmor> armors = detector_->matchArmors(lights);

  for (auto & armor : armors) {
  armor.distance_to_image_center =
    detector_->calculateDistanceToCenter(armor.center, img.size());
  }

  if (enable_pnp_ && pnp_solver_) {
    for (auto & armor : armors) {
      pnp_solver_->solvePnP(armor);
    }
  }

  cv::Mat result = img.clone();
  detector_->drawResults(result, lights, armors);

  publishMarkers(msg->header, armors);
  publishPoses(msg->header, armors);

  publishImage(binary_pub_, binary, msg->header, "mono8");
  publishImage(result_pub_, result, msg->header, "bgr8");
}

void SimpleArmorDetectorNode::publishImage(
  image_transport::Publisher & pub,
  const cv::Mat & image,
  const std_msgs::msg::Header & header,
  const std::string & encoding)
{
  auto msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
  pub.publish(msg);
}

// 发布装甲板位置的可视化标记
void SimpleArmorDetectorNode::publishMarkers(
  const std_msgs::msg::Header & header,
  const std::vector<SimpleArmor> & armors)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.ns = "simple_armors";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.emplace_back(clear_marker);

  int id = 1;

  for (const auto & armor : armors) {
    if (!armor.has_pose || armor.tvec.empty()) {
      continue;
    }
    
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "simple_armors";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = armor.tvec.at<double>(0);
    marker.pose.position.y = armor.tvec.at<double>(1);
    marker.pose.position.z = armor.tvec.at<double>(2);

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.08;              // 设置标记大小，单位为米
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;

    marker.color.r = 1.0;               // 设置标记颜色，红色表示敌方装甲板
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_array.markers.emplace_back(marker);
  }

  marker_pub_->publish(marker_array);
}

// 发布装甲板位姿
void SimpleArmorDetectorNode::publishPoses(
  const std_msgs::msg::Header & header,
  const std::vector<SimpleArmor> & armors)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header = header;

  for (const auto & armor : armors) {
    if (!armor.has_pose || armor.tvec.empty()) {
      continue;
    }

    geometry_msgs::msg::Pose pose;

    pose.position.x = armor.tvec.at<double>(0);     // 直接使用 PnP 求解的 tvec 作为位置
    pose.position.y = armor.tvec.at<double>(1);
    pose.position.z = armor.tvec.at<double>(2);

    pose.orientation.x = 0.0;                       // 构造四元数，假设装甲板面朝前方
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    pose_array.poses.emplace_back(pose);
  }

  pose_pub_->publish(pose_array);
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                         // 初始化 ROS 2
  rclcpp::spin(std::make_shared<simple_armor_detector::SimpleArmorDetectorNode>());   // 创建节点实例并开始处理回调
  rclcpp::shutdown();                               // 关闭 ROS 2
  return 0;
}