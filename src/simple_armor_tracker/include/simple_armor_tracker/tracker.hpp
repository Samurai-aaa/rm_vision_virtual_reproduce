#ifndef SIMPLE_ARMOR_TRACKER__TRACKER_HPP_
#define SIMPLE_ARMOR_TRACKER__TRACKER_HPP_

#include "simple_armor_tracker/extended_kalman_filter.hpp"

#include <simple_armor_interfaces/msg/simple_armor.hpp>
#include <simple_armor_interfaces/msg/simple_armors.hpp>

#include <Eigen/Dense>

#include <geometry_msgs/msg/quaternion.hpp>

#include <memory>
#include <string>

namespace simple_armor_tracker
{

using SimpleArmor = simple_armor_interfaces::msg::SimpleArmor;
using SimpleArmors = simple_armor_interfaces::msg::SimpleArmors;

class Tracker
{
public:
  enum State
  {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST
  };

  enum ArmorsNum
  {
    NORMAL_4 = 4,
    BALANCE_2 = 2,
    OUTPOST_3 = 3
  };

  Tracker(double max_match_distance, double max_match_yaw_diff);

  void init(const SimpleArmors::SharedPtr & armors_msg);

  void update(const SimpleArmors::SharedPtr & armors_msg);

  void initEKF(const SimpleArmor & armor);

  void updateArmorsNum(const SimpleArmor & armor);

  void handleArmorJump(const SimpleArmor & current_armor);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

public:
  ExtendedKalmanFilter ekf;

  State tracker_state = LOST;

  std::string tracked_id;

  SimpleArmor tracked_armor;

  Eigen::VectorXd measurement;

  Eigen::VectorXd target_state;

  int tracking_thres = 5;
  int lost_thres = 10;

  int detect_count_ = 0;
  int lost_count_ = 0;

  double info_position_diff = 0.0;
  double info_yaw_diff = 0.0;

  ArmorsNum tracked_armors_num = NORMAL_4;

  double another_r = 0.26;
  double dz = 0.0;

private:
  double max_match_distance_;
  double max_match_yaw_diff_;

  double last_yaw_ = 0.0;
};

}  // namespace simple_armor_tracker

#endif  // SIMPLE_ARMOR_TRACKER__TRACKER_HPP_
