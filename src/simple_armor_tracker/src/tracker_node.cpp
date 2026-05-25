#include "simple_armor_tracker/tracker_node.hpp"

#include <Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace simple_armor_tracker
{
// 创建 simple_armor_tracker 节点
SimpleArmorTrackerNode::SimpleArmorTrackerNode()
: Node("simple_armor_tracker")
{
  RCLCPP_INFO(this->get_logger(), "Starting SimpleArmorTrackerNode!");

  armors_topic_ = this->declare_parameter<std::string>(         // 声明基础参数：输入话题、输出坐标系、距离过滤和置信度过滤阈值
    "armors_topic", "/simple_detector/armors");

  target_frame_ = this->declare_parameter<std::string>(
    "target_frame", "camera");

  max_armor_distance_ = this->declare_parameter<double>(
    "max_armor_distance", 10.0);

  max_abs_z_ = this->declare_parameter<double>(
    "max_abs_z", 1.2);

  min_confidence_ = this->declare_parameter<double>(
    "min_confidence", 0.3);

  double max_match_distance = this->declare_parameter<double>(      // 读取 tracker 状态机与目标匹配相关参数
    "tracker.max_match_distance", 0.15);

  double max_match_yaw_diff = this->declare_parameter<double>(
    "tracker.max_match_yaw_diff", 1.0);

  int tracking_thres = this->declare_parameter<int>(
    "tracker.tracking_thres", 5);

  lost_time_thres_ = this->declare_parameter<double>(
    "tracker.lost_time_thres", 0.3);

  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);   // 初始化 Tracker，负责目标匹配、状态机和 EKF 更新
  tracker_->tracking_thres = tracking_thres;

  // EKF 状态量：目标中心位置、速度、角度、角速度和半径
  // EKF 观测量：当前检测到的装甲板位置和 yaw
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;

    x_new(0) += x(1) * dt_;                           // EKF 状态预测函数：根据速度预测目标下一时刻的位置和 yaw
    x_new(2) += x(3) * dt_;
    x_new(4) += x(5) * dt_;
    x_new(6) += x(7) * dt_;

    return x_new;
  };

  auto j_f = [this](const Eigen::VectorXd &) {        // 状态转移函数的雅可比矩阵，用于 EKF predict
    Eigen::MatrixXd f(9, 9);

    // clang-format off
    f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   dt_, 0,   0,   0,   0,   0,
          0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   dt_, 0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   dt_, 0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on

    return f;
  };

  auto h = [](const Eigen::VectorXd & x) {            // 观测函数：由目标中心状态推算当前可观测装甲板的位置和 yaw
    Eigen::VectorXd z(4);

    double xc = x(0);
    double yc = x(2);
    double yaw = x(6);
    double r = x(8);

    z(0) = xc - r * std::cos(yaw);
    z(1) = yc - r * std::sin(yaw);
    z(2) = x(4);
    z(3) = x(6);

    return z;
  };

  auto j_h = [](const Eigen::VectorXd & x) {         // 观测函数的雅可比矩阵，用于 EKF update
    Eigen::MatrixXd h(4, 9);

    double yaw = x(6);
    double r = x(8);

    // clang-format off
    h <<  1,   0,   0,   0,   0,   0,   r * std::sin(yaw),  0,  -std::cos(yaw),
          0,   0,   1,   0,   0,   0,  -r * std::cos(yaw),  0,  -std::sin(yaw),
          0,   0,   0,   0,   1,   0,   0,                  0,   0,
          0,   0,   0,   0,   0,   0,   1,                  0,   0;
    // clang-format on

    return h;
  };

  // EKF 过程噪声参数，控制对运动模型的信任程度
  s2qxyz_ = this->declare_parameter<double>("ekf.sigma2_q_xyz", 20.0);       
  s2qyaw_ = this->declare_parameter<double>("ekf.sigma2_q_yaw", 100.0);
  s2qr_ = this->declare_parameter<double>("ekf.sigma2_q_r", 800.0);

  auto u_q = [this]() {                             // 根据 dt 动态计算过程噪声矩阵 Q
    Eigen::MatrixXd q(9, 9);

    double t = dt_;
    double x = s2qxyz_;
    double y = s2qyaw_;
    double r = s2qr_;

    double q_x_x = std::pow(t, 4) / 4.0 * x;
    double q_x_vx = std::pow(t, 3) / 2.0 * x;
    double q_vx_vx = std::pow(t, 2) * x;

    double q_y_y = std::pow(t, 4) / 4.0 * y;
    double q_y_vy = std::pow(t, 3) / 2.0 * y;
    double q_vy_vy = std::pow(t, 2) * y;

    double q_r = std::pow(t, 4) / 4.0 * r;

    // clang-format off
    q <<  q_x_x,   q_x_vx,  0,       0,        0,       0,        0,       0,        0,
          q_x_vx,  q_vx_vx, 0,       0,        0,       0,        0,       0,        0,
          0,       0,       q_x_x,   q_x_vx,   0,       0,        0,       0,        0,
          0,       0,       q_x_vx,  q_vx_vx,  0,       0,        0,       0,        0,
          0,       0,       0,       0,        q_x_x,   q_x_vx,   0,       0,        0,
          0,       0,       0,       0,        q_x_vx,  q_vx_vx,  0,       0,        0,
          0,       0,       0,       0,        0,       0,        q_y_y,   q_y_vy,   0,
          0,       0,       0,       0,        0,       0,        q_y_vy,  q_vy_vy,  0,
          0,       0,       0,       0,        0,       0,        0,       0,        q_r;
    // clang-format on

    return q;
  };

  // EKF 观测噪声参数，控制对 detector 测量值的信任程度
  r_xyz_factor_ = this->declare_parameter<double>("ekf.r_xyz_factor", 0.05);       
  r_yaw_ = this->declare_parameter<double>("ekf.r_yaw", 0.02);

  auto u_r = [this](const Eigen::VectorXd & z) {   // 根据当前观测动态计算观测噪声矩阵 R
    Eigen::DiagonalMatrix<double, 4> r;

    double factor = r_xyz_factor_;

    r.diagonal() <<
      std::abs(factor * z(0)),
      std::abs(factor * z(1)),
      std::abs(factor * z(2)),
      r_yaw_;

    return r;
  };

  Eigen::DiagonalMatrix<double, 9> p0;            // 使用状态模型、观测模型和噪声模型初始化 EKF
  p0.setIdentity();

  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(                       // 创建 tracker 重置服务，便于调试时手动清空跟踪状态
    "/simple_tracker/reset",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->tracker_state = Tracker::LOST;
      response->success = true;
      response->message = "Simple tracker reset";
      RCLCPP_INFO(this->get_logger(), "Simple tracker reset!");
    });

  armors_sub_ = this->create_subscription<simple_armor_interfaces::msg::SimpleArmors>(     // 订阅 detector 发布的装甲板数组，作为 tracker 的观测输入
    armors_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&SimpleArmorTrackerNode::armorsCallback, this, std::placeholders::_1));

  target_pub_ = this->create_publisher<simple_armor_interfaces::msg::SimpleTarget>(        // 发布 tracker 输出的稳定目标状态
    "/simple_tracker/target",
    rclcpp::SensorDataQoS());

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(              // 发布 tracker 可视化 Marker，用于 RViz 调试
    "/simple_tracker/marker", 
    10);

  // 初始化 RViz Marker：目标位置、线速度、角速度和装甲板位置
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = 0.1;
  position_marker_.scale.y = 0.1;
  position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;

  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;

  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;

  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;

  RCLCPP_INFO(this->get_logger(), "Subscribe armors topic: %s", armors_topic_.c_str());
}

// 接收 detector 发布的装甲板数组，完成目标过滤、tracker 更新和目标发布
void SimpleArmorTrackerNode::armorsCallback(
  const simple_armor_interfaces::msg::SimpleArmors::SharedPtr armors_msg)
{
  auto filtered_msg =
    std::make_shared<simple_armor_interfaces::msg::SimpleArmors>(*armors_msg);
 
  filtered_msg->armors.erase(                                     // 过滤无效候选：无位姿、低置信度、z异常或距离过远的 armor 不进入 tracker
    std::remove_if(
      filtered_msg->armors.begin(),
      filtered_msg->armors.end(),
      [this](const simple_armor_interfaces::msg::SimpleArmor & armor) {
        if (!armor.has_pose) {
          return true;
        }

        if (armor.confidence < min_confidence_) {
          return true;
        }

        if (std::abs(armor.pose.position.z) > max_abs_z_) {
          return true;
        }

        double xy_distance = std::hypot(
          armor.pose.position.x,
          armor.pose.position.y);

        if (xy_distance > max_armor_distance_) {
          return true;
        }

        return false;
      }),
    filtered_msg->armors.end());

  simple_armor_interfaces::msg::SimpleTarget target_msg;         // 创建 tracker 输出目标消息

  rclcpp::Time time = filtered_msg->header.stamp;                // 使用输入消息时间戳和坐标系；若为空则使用当前时间和默认坐标系
  if (time.nanoseconds() == 0) {
    time = this->now();
  }

  target_msg.header.stamp = time;
  target_msg.header.frame_id =
    filtered_msg->header.frame_id.empty() ? target_frame_ : filtered_msg->header.frame_id;

  if (tracker_->tracker_state == Tracker::LOST) {                // LOST 状态下尝试用当前观测初始化 tracker，但暂不认为已经稳定跟踪
    tracker_->init(filtered_msg);
    target_msg.tracking = false;
  } else {
    if (has_last_time_) {
      dt_ = (time - last_time_).seconds();
      if (dt_ <= 0.0 || dt_ > 1.0) {                             // 计算帧间隔 dt，异常时使用默认 0.033s
        dt_ = 0.033;
      }
    } else {
      dt_ = 0.033;
      has_last_time_ = true;
    }

    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_); // 将允许丢失时间转换为允许丢失帧数
    tracker_->update(filtered_msg);                                  // 使用当前帧观测更新 tracker 状态机和 EKF

    if (tracker_->tracker_state == Tracker::DETECTING) {             // TRACKING 或 TEMP_LOST 状态下，输出稳定目标状态
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;

      const auto & state = tracker_->target_state;                   // 将 EKF 状态量转换为 SimpleTarget 消息输出

      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);

      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);

      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);

      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);

      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);

      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.dz = tracker_->dz;
    } else {
      target_msg.tracking = false;
    }
  }

  // 更新时间戳，发布目标状态和可视化结果
  last_time_ = time;
  has_last_time_ = true;

  target_pub_->publish(target_msg);

  publishMarkers(target_msg);
}

// 根据 tracker 输出目标生成 RViz 可视化 Marker
void SimpleArmorTrackerNode::publishMarkers(
  const simple_armor_interfaces::msg::SimpleTarget & target_msg)
{
  // 保证所有 Marker 使用同一时间戳和坐标系
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;

  if (target_msg.tracking) {                                                  // 只有处于有效跟踪状态时才显示目标可视化
    double yaw = target_msg.yaw;
    double r1 = target_msg.radius_1;
    double r2 = target_msg.radius_2;

    double xc = target_msg.position.x;
    double yc = target_msg.position.y;
    double za = target_msg.position.z;

    double vx = target_msg.velocity.x;
    double vy = target_msg.velocity.y;
    double vz = target_msg.velocity.z;

    double dz = target_msg.dz;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;           // 显示 tracker 估计的目标中心位置
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2.0;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);

    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;     // 使用箭头显示目标线速度方向
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);    

    arrow_end = position_marker_.pose.position;                               // 使用箭头粗略显示目标 yaw 角速度
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;              // 根据装甲板类型设置可视化宽度
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;

    size_t armors_num = static_cast<size_t>(std::max(1, target_msg.armors_num));
    bool is_current_pair = true;

    geometry_msgs::msg::Point armor_position;

    for (size_t i = 0; i < armors_num; ++i) {                                 // 根据目标中心、yaw 和半径，在圆周模型上推算各装甲板位置
      double tmp_yaw = yaw + static_cast<double>(i) * (2.0 * M_PI / armors_num);

      double r = r1;

      if (armors_num == 4) {                                                  // 四装甲板目标使用双半径和高度差建模
        r = is_current_pair ? r1 : r2;
        armor_position.z = za + (is_current_pair ? 0.0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        armor_position.z = za;
      }

      armor_position.x = xc - r * std::cos(tmp_yaw);
      armor_position.y = yc - r * std::sin(tmp_yaw);

      armor_marker_.id = static_cast<int>(i);
      armor_marker_.pose.position = armor_position;

      tf2::Quaternion q;                                                      // 设置装甲板 Marker 朝向
      q.setRPY(0.0, 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);

      marker_array.markers.emplace_back(armor_marker_);
    }
  } else {                                                                    // 没有有效目标时删除旧 Marker
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;

    marker_array.markers.emplace_back(armor_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);

  marker_pub_->publish(marker_array);                                         // 发布 RViz 可视化结果
}

}  // namespace simple_armor_tracker

// ROS 2 程序入口：初始化、运行 tracker 节点、关闭
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_armor_tracker::SimpleArmorTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
