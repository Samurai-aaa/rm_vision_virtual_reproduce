#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class SimpleArmorTrackerNode : public rclcpp::Node
{
public:
  SimpleArmorTrackerNode() : Node("simple_armor_tracker")
  {
    declareParams();
    loadParams();

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      input_topic_,
      10,
      std::bind(&SimpleArmorTrackerNode::posesCallback, this, std::placeholders::_1));

    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/simple_tracker/target", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/simple_tracker/marker", 10);

    RCLCPP_INFO(this->get_logger(), "simple_armor_tracker started.");
    RCLCPP_INFO(this->get_logger(), "Subscribe: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Filter alpha: %.2f", alpha_);
  }

private:
  void declareParams()
  {
    this->declare_parameter<std::string>("input_topic", "/simple_detector/poses");

    this->declare_parameter<double>("alpha", 0.5);
    this->declare_parameter<int>("max_lost_frames", 10);

    this->declare_parameter<std::string>("select_strategy", "nearest_last");
  }

  void loadParams()
  {
    input_topic_ = this->get_parameter("input_topic").as_string();
    alpha_ = this->get_parameter("alpha").as_double();
    max_lost_frames_ = this->get_parameter("max_lost_frames").as_int();
    select_strategy_ = this->get_parameter("select_strategy").as_string();

    if (alpha_ < 0.0) {
      alpha_ = 0.0;
    }

    if (alpha_ > 1.0) {
      alpha_ = 1.0;
    }
  }

  void posesCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
  {
    if (msg->poses.empty()) {
      handleLost(msg->header);
      return;
    }

    lost_count_ = 0;

    geometry_msgs::msg::Pose selected_pose = selectTarget(msg->poses);

    if (!has_target_) {
      filtered_pose_ = selected_pose;
      has_target_ = true;
    } else {
      filtered_pose_ = lowPassFilter(filtered_pose_, selected_pose);
    }

    publishTarget(msg->header, filtered_pose_);
    publishMarker(msg->header, filtered_pose_, true);
  }

  void handleLost(const std_msgs::msg::Header & header)
  {
    lost_count_++;

    if (has_target_ && lost_count_ <= max_lost_frames_) {
      publishTarget(header, filtered_pose_);
      publishMarker(header, filtered_pose_, true);

      RCLCPP_DEBUG(
        this->get_logger(),
        "Target temporarily lost. Keep last target. lost_count=%d",
        lost_count_);
      return;
    }

    if (has_target_) {
      RCLCPP_INFO(this->get_logger(), "Target lost.");
    }

    has_target_ = false;
    publishMarker(header, filtered_pose_, false);
  }

  geometry_msgs::msg::Pose selectTarget(const std::vector<geometry_msgs::msg::Pose> & poses)
  {
    if (poses.size() == 1) {
      return poses.front();
    }

    if (!has_target_ || select_strategy_ == "nearest_camera") {
      return selectNearestCamera(poses);
    }

    return selectNearestLast(poses);
  }

  geometry_msgs::msg::Pose selectNearestCamera(const std::vector<geometry_msgs::msg::Pose> & poses)
  {
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Pose best_pose = poses.front();

    for (const auto & pose : poses) {
      double distance = std::sqrt(
        pose.position.x * pose.position.x +
        pose.position.y * pose.position.y +
        pose.position.z * pose.position.z);

      if (distance < min_distance) {
        min_distance = distance;
        best_pose = pose;
      }
    }

    return best_pose;
  }

  geometry_msgs::msg::Pose selectNearestLast(const std::vector<geometry_msgs::msg::Pose> & poses)
  {
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Pose best_pose = poses.front();

    for (const auto & pose : poses) {
      double dx = pose.position.x - filtered_pose_.position.x;
      double dy = pose.position.y - filtered_pose_.position.y;
      double dz = pose.position.z - filtered_pose_.position.z;

      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (distance < min_distance) {
        min_distance = distance;
        best_pose = pose;
      }
    }

    return best_pose;
  }

  geometry_msgs::msg::Pose lowPassFilter(
    const geometry_msgs::msg::Pose & last_pose,
    const geometry_msgs::msg::Pose & current_pose)
  {
    geometry_msgs::msg::Pose result;

    result.position.x = alpha_ * current_pose.position.x + (1.0 - alpha_) * last_pose.position.x;
    result.position.y = alpha_ * current_pose.position.y + (1.0 - alpha_) * last_pose.position.y;
    result.position.z = alpha_ * current_pose.position.z + (1.0 - alpha_) * last_pose.position.z;

    result.orientation = current_pose.orientation;

    return result;
  }

  void publishTarget(
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Pose & pose)
  {
    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header = header;
    target_msg.pose = pose;

    target_pub_->publish(target_msg);
  }

  void publishMarker(
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Pose & pose,
    bool visible)
  {
    visualization_msgs::msg::Marker marker;

    marker.header = header;
    marker.ns = "simple_tracker";
    marker.id = 0;

    if (!visible) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(marker);
      return;
    }

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = pose;

    marker.scale.x = 0.12;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub_->publish(marker);
  }

private:
  std::string input_topic_;
  std::string select_strategy_;

  double alpha_;
  int max_lost_frames_;

  int lost_count_ = 0;
  bool has_target_ = false;

  geometry_msgs::msg::Pose filtered_pose_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleArmorTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
