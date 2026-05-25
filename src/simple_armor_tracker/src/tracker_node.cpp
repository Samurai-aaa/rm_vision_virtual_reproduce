#include "simple_armor_tracker/tracker_node.hpp"

#include <simple_armor_interfaces/msg/simple_armors.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace simple_armor_tracker
{

SimpleArmorTrackerNode::SimpleArmorTrackerNode()
: Node("simple_armor_tracker")
{
  declareParams();
  loadParams();

  tracker_ = std::make_unique<Tracker>(tracker_params_);

  armors_sub_ = this->create_subscription<simple_armor_interfaces::msg::SimpleArmors>(
    armors_topic_,
    10,
    std::bind(&SimpleArmorTrackerNode::armorsCallback, this, std::placeholders::_1));

  target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/simple_tracker/target", 10);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/simple_tracker/marker", 10);

  RCLCPP_INFO(this->get_logger(), "simple_armor_tracker started.");
  RCLCPP_INFO(this->get_logger(), "Subscribe armors topic: %s", armors_topic_.c_str());
}

void SimpleArmorTrackerNode::declareParams()
{
  this->declare_parameter<std::string>("armors_topic", "/simple_detector/armors");
  this->declare_parameter<double>("min_confidence", 0.3);
  this->declare_parameter<double>("max_match_distance", 0.8);
  this->declare_parameter<int>("tracking_threshold", 3);
  this->declare_parameter<int>("lost_threshold", 10);
}

void SimpleArmorTrackerNode::loadParams()
{
  armors_topic_ = this->get_parameter("armors_topic").as_string();
  min_confidence_ = this->get_parameter("min_confidence").as_double();

  tracker_params_.max_match_distance =
    this->get_parameter("max_match_distance").as_double();

  tracker_params_.tracking_threshold =
    this->get_parameter("tracking_threshold").as_int();

  tracker_params_.lost_threshold =
    this->get_parameter("lost_threshold").as_int();
}

void SimpleArmorTrackerNode::armorsCallback(
  const simple_armor_interfaces::msg::SimpleArmors::ConstSharedPtr msg)
{
  rclcpp::Time current_time = msg->header.stamp;

  double dt = 0.033;

  if (has_last_time_) {
    dt = (current_time - last_time_).seconds();

    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.033;
    }
  }

  last_time_ = current_time;
  has_last_time_ = true;

  std::vector<cv::Point3f> measurements;

  for (const auto & armor : msg->armors) {
    if (!armor.has_pose) {
      continue;
    }

    if (armor.confidence < min_confidence_) {
      continue;
    }

    measurements.emplace_back(
      static_cast<float>(armor.pose.position.x),
      static_cast<float>(armor.pose.position.y),
      static_cast<float>(armor.pose.position.z));
  }

  Tracker::Result result = tracker_->update(measurements, dt);

  publishTarget(msg->header, result);
  publishMarker(msg->header, result);
}

void SimpleArmorTrackerNode::publishTarget(
  const std_msgs::msg::Header & header,
  const Tracker::Result & result)
{
  if (!result.has_target) {
    return;
  }

  geometry_msgs::msg::PoseStamped target;
  target.header = header;

  target.pose.position.x = result.position.x;
  target.pose.position.y = result.position.y;
  target.pose.position.z = result.position.z;

  target.pose.orientation.x = 0.0;
  target.pose.orientation.y = 0.0;
  target.pose.orientation.z = 0.0;
  target.pose.orientation.w = 1.0;

  target_pub_->publish(target);
}

void SimpleArmorTrackerNode::publishMarker(
  const std_msgs::msg::Header & header,
  const Tracker::Result & result)
{
  visualization_msgs::msg::Marker marker;

  marker.header = header;
  marker.ns = "simple_tracker";
  marker.id = 0;

  if (!result.has_target) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(marker);
    return;
  }

  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = result.position.x;
  marker.pose.position.y = result.position.y;
  marker.pose.position.z = result.position.z;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.12;
  marker.scale.y = 0.12;
  marker.scale.z = 0.12;

  if (result.state == Tracker::State::TRACKING) {
    marker.color.g = 1.0;
  } else if (result.state == Tracker::State::DETECTING) {
    marker.color.b = 1.0;
  } else if (result.state == Tracker::State::TEMP_LOST) {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
  } else {
    marker.color.r = 1.0;
  }

  marker.color.a = 1.0;

  marker_pub_->publish(marker);
}

}  // namespace simple_armor_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_armor_tracker::SimpleArmorTrackerNode>());
  rclcpp::shutdown();
  return 0;
}