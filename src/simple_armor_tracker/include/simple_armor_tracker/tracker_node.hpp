#ifndef SIMPLE_ARMOR_TRACKER__TRACKER_NODE_HPP_
#define SIMPLE_ARMOR_TRACKER__TRACKER_NODE_HPP_

#include "simple_armor_tracker/tracker.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <simple_armor_interfaces/msg/simple_armors.hpp>

#include <memory>
#include <string>
#include <vector>

namespace simple_armor_tracker
{

class SimpleArmorTrackerNode : public rclcpp::Node
{
public:
  SimpleArmorTrackerNode();

private:
  void declareParams();

  void loadParams();

  void armorsCallback(
  const simple_armor_interfaces::msg::SimpleArmors::ConstSharedPtr msg);

  void publishTarget(
    const std_msgs::msg::Header & header,
    const Tracker::Result & result);

  void publishMarker(
    const std_msgs::msg::Header & header,
    const Tracker::Result & result);

private:
  std::string armors_topic_;
  double min_confidence_ = 0.3;

  Tracker::Params tracker_params_;

  std::unique_ptr<Tracker> tracker_;

  rclcpp::Subscription<simple_armor_interfaces::msg::SimpleArmors>::SharedPtr armors_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::Time last_time_;
  bool has_last_time_ = false;
};

}  // namespace simple_armor_tracker

#endif  // SIMPLE_ARMOR_TRACKER__TRACKER_NODE_HPP_
