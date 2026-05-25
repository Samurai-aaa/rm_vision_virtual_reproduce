#include "simple_armor_tracker/extended_kalman_filter.hpp"

namespace simple_armor_tracker
{
// 初始化 EKF：保存状态模型、观测模型、雅可比矩阵和噪声模型
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f,
  const VecVecFunc & h,
  const VecMatFunc & j_f,
  const VecMatFunc & j_h,
  const VoidMatFunc & u_q,
  const VecMatFunc & u_r,
  const Eigen::MatrixXd & P0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_post(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
}

// 设置 EKF 当前状态，用于初始化或状态重置
void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0)
{
  x_post = x0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict()
{
  F = jacobian_f(x_post);
  Q = update_Q();

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

// EKF 预测：根据上一帧状态和运动模型预测当前状态
Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  H = jacobian_h(x_pri);
  R = update_R(z);

  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();

  x_post = x_pri + K * (z - h(x_pri));
  P_post = (I - K * H) * P_pri;

  return x_post;
}

}  // namespace simple_armor_tracker
