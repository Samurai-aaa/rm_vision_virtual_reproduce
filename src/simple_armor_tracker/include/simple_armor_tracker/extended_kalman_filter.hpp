#ifndef SIMPLE_ARMOR_TRACKER__EXTENDED_KALMAN_FILTER_HPP_
#define SIMPLE_ARMOR_TRACKER__EXTENDED_KALMAN_FILTER_HPP_

#include <Eigen/Dense>

#include <functional>

namespace simple_armor_tracker
{

class ExtendedKalmanFilter
{
public:
  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
    const VecVecFunc & f,
    const VecVecFunc & h,
    const VecMatFunc & j_f,
    const VecMatFunc & j_h,
    const VoidMatFunc & u_q,
    const VecMatFunc & u_r,
    const Eigen::MatrixXd & P0);

  void setState(const Eigen::VectorXd & x0);

  Eigen::VectorXd predict();

  Eigen::VectorXd update(const Eigen::VectorXd & z);

private:
  VecVecFunc f;
  VecVecFunc h;
  VecMatFunc jacobian_f;
  VecMatFunc jacobian_h;
  VoidMatFunc update_Q;
  VecMatFunc update_R;

  Eigen::MatrixXd F;
  Eigen::MatrixXd H;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd K;

  Eigen::MatrixXd P_pri;
  Eigen::MatrixXd P_post;

  int n = 0;
  Eigen::MatrixXd I;

  Eigen::VectorXd x_pri;
  Eigen::VectorXd x_post;
};

}  // namespace simple_armor_tracker

#endif  // SIMPLE_ARMOR_TRACKER__EXTENDED_KALMAN_FILTER_HPP_
