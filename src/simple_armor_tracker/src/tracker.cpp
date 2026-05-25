#include "simple_armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cfloat>
#include <cmath>
#include <memory>
#include <string>

namespace simple_armor_tracker
{
// 初始化 Tracker 状态机、观测量、状态量和目标匹配阈值
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST),
  tracked_id(std::string("")),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff)
{
}

// 在 LOST 状态下，从当前帧 armor 中选择一个目标并初始化 EKF
void Tracker::init(const SimpleArmors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {                           // 没有检测到 armor，无法初始化
    return;
  }

  double min_distance = DBL_MAX;                              // 准备根据 distance_to_image_center 选择最靠近图像中心的 armor
  tracked_armor = armors_msg->armors[0];

  for (const auto & armor : armors_msg->armors) {             // 选择有位姿且最靠近图像中心的 armor 作为初始目标
    if (!armor.has_pose) {
      continue;
    }

    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  if (!tracked_armor.has_pose) {                              // 没有有效 PnP 位姿则不能初始化 EKF
    return;
  }
 
  initEKF(tracked_armor);                                     // 使用选中的 armor 初始化 EKF 状态

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;

  updateArmorsNum(tracked_armor);                             // 根据目标信息更新装甲板数量模型

  // 输出初始化成功日志
  RCLCPP_INFO(rclcpp::get_logger("simple_armor_tracker"), "Init EKF!");
}

// 根据当前帧 armor 观测更新 EKF 和 tracker 状态机
void Tracker::update(const SimpleArmors::SharedPtr & armors_msg)
{
  Eigen::VectorXd ekf_prediction = ekf.predict();            // 先进行 EKF 预测，若本帧无匹配观测则使用预测状态

  bool matched = false;

  target_state = ekf_prediction;

  if (!armors_msg->armors.empty()) {
    SimpleArmor same_id_armor;
    int same_id_armors_count = 0;

    auto predicted_position = getArmorPositionFromState(ekf_prediction);  // 从预测状态中计算当前装甲板应出现的位置

    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;

    for (const auto & armor : armors_msg->armors) {                       // 遍历当前帧 armor，寻找与 tracked_id 相同且最接近预测位置的目标
      if (!armor.has_pose) {
        continue;
      }

      // 当前没有数字识别时，number 大多是 unknown 或 0
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;

        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);

        double position_diff = (predicted_position - position_vec).norm();

        if (position_diff < min_position_diff) {
          min_position_diff = position_diff;
          yaw_diff = std::abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
          tracked_armor = armor;
        }
      }
    }

    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;

    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {    // 位置差和 yaw 差都满足阈值时，认为匹配成功
      matched = true;

      auto p = tracked_armor.pose.position;
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);

      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);

      target_state = ekf.update(measurement);

      RCLCPP_DEBUG(rclcpp::get_logger("simple_armor_tracker"), "EKF update");
    } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      handleArmorJump(same_id_armor);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("simple_armor_tracker"), "No matched armor found!");
    }
  }

  // 防止半径发散
  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    ekf.setState(target_state);
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    ekf.setState(target_state);
  }

  // 状态机，连续多帧匹配成功后进入 TRACKING，失败则回到 LOST
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {       // TRACKING：本帧未匹配到目标时进入 TEMP_LOST
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {      // TEMP_LOST：短时丢失用预测维持，超过阈值则 LOST，重新匹配则恢复 TRACKING
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
}

// 使用当前 armor 的 3D 位姿初始化 EKF 状态，状态量为目标中心、速度、yaw 和半径
void Tracker::initEKF(const SimpleArmor & armor)
{
  double xa = armor.pose.position.x;
  double ya = armor.pose.position.y;
  double za = armor.pose.position.z;

  last_yaw_ = 0.0;
  double yaw = orientationToYaw(armor.pose.orientation);

  target_state = Eigen::VectorXd::Zero(9);

  double r = 0.26;
  double xc = xa + r * std::cos(yaw);
  double yc = ya + r * std::sin(yaw);

  dz = 0.0;
  another_r = r;

  target_state << xc, 0.0, yc, 0.0, za, 0.0, yaw, 0.0, r;

  ekf.setState(target_state);
}

// 根据装甲板类型和编号选择目标模型：2装甲板、3装甲板或4装甲板
void Tracker::updateArmorsNum(const SimpleArmor & armor)
{
  if (armor.type == "large" &&
      (tracked_id == "3" || tracked_id == "4" || tracked_id == "5"))
  {
    tracked_armors_num = BALANCE_2;
  } else if (tracked_id == "outpost") {
    tracked_armors_num = OUTPOST_3;
  } else {
    tracked_armors_num = NORMAL_4;
  }
}

// 处理目标旋转导致的装甲板跳变，必要时重置目标中心和速度状态
void Tracker::handleArmorJump(const SimpleArmor & current_armor)
{
  double yaw = orientationToYaw(current_armor.pose.orientation);

  target_state(6) = yaw;

  updateArmorsNum(current_armor);

  if (tracked_armors_num == NORMAL_4) {
    dz = target_state(4) - current_armor.pose.position.z;
    target_state(4) = current_armor.pose.position.z;
    std::swap(target_state(8), another_r);
  }

  RCLCPP_WARN(rclcpp::get_logger("simple_armor_tracker"), "Armor jump!");

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_) {
    double r = target_state(8);

    target_state(0) = p.x + r * std::cos(yaw);
    target_state(1) = 0.0;
    target_state(2) = p.y + r * std::sin(yaw);
    target_state(3) = 0.0;
    target_state(4) = p.z;
    target_state(5) = 0.0;

    RCLCPP_ERROR(rclcpp::get_logger("simple_armor_tracker"), "Reset State!");
  }

  ekf.setState(target_state);
}

// 将四元数转换为连续 yaw 角，避免 ±pi 边界处角度跳变
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);

  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;

  return yaw;
}

// 根据目标中心、yaw 和半径，推算当前可见装甲板的 3D 位置
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
  double xc = x(0);
  double yc = x(2);
  double za = x(4);
  double yaw = x(6);
  double r = x(8);

  double xa = xc - r * std::cos(yaw);
  double ya = yc - r * std::sin(yaw);

  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace simple_armor_tracker
