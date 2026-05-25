#ifndef SIMPLE_ARMOR_TRACKER__TRACKER_NODE_HPP_
#define SIMPLE_ARMOR_TRACKER__TRACKER_NODE_HPP_

#include "simple_armor_tracker/tracker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <simple_armor_interfaces/msg/simple_armors.hpp>
#include <simple_armor_interfaces/msg/simple_target.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

namespace simple_armor_tracker
{

class SimpleArmorTrackerNode : public rclcpp::Node
{
public:
  SimpleArmorTrackerNode();

private:
  void armorsCallback(
    const simple_armor_interfaces::msg::SimpleArmors::SharedPtr armors_msg);

  void publishMarkers(
    const simple_armor_interfaces::msg::SimpleTarget & target_msg);

private:
  std::string armors_topic_;
  std::string target_frame_;

  double max_armor_distance_ = 10.0;
  double max_abs_z_ = 1.2;
  double min_confidence_ = 0.3;

  rclcpp::Time last_time_;
  double dt_ = 0.033;
  bool has_last_time_ = false;

  double s2qxyz_ = 20.0;
  double s2qyaw_ = 100.0;
  double s2qr_ = 800.0;
  double r_xyz_factor_ = 0.05;
  double r_yaw_ = 0.02;
  double lost_time_thres_ = 0.3;

  std::unique_ptr<Tracker> tracker_;

  rclcpp::Subscription<simple_armor_interfaces::msg::SimpleArmors>::SharedPtr armors_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  rclcpp::Publisher<simple_armor_interfaces::msg::SimpleTarget>::SharedPtr target_pub_;

  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace simple_armor_tracker

#endif  // SIMPLE_ARMOR_TRACKER__TRACKER_NODE_HPP_
