#include "simple_armor_detector/pnp_solver.hpp"

#include <opencv2/opencv.hpp>

#include <array>
#include <vector>

namespace simple_armor_detector
{
// 构造函数，初始化相机内参和装甲板3D坐标点
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix,
  const std::vector<double> & dist_coeffs,
  const Params & params)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, static_cast<int>(dist_coeffs.size()), CV_64F,
    const_cast<double *>(dist_coeffs.data())).clone())
{
  double small_half_y = params.small_armor_width / 2.0 / 1000.0;
  double small_half_z = params.small_armor_height / 2.0 / 1000.0;

  double large_half_y = params.large_armor_width / 2.0 / 1000.0;
  double large_half_z = params.large_armor_height / 2.0 / 1000.0;

  // image points 顺序：top_left, top_right, bottom_right, bottom_left
  // object points 也保持同样顺序
  small_armor_points_.emplace_back(cv::Point3f(0.0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0.0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0.0, -small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0.0, small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0.0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0.0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0.0, -large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0.0, large_half_y, -large_half_z));
}

// 解算装甲板位姿。返回是否成功解算
bool PnPSolver::solvePnP(SimpleArmor & armor)
{
  if (armor.points.size() != 4) {
    armor.has_pose = false;
    return false;
  }

  const auto & object_points =
    armor.type == "large" ? large_armor_points_ : small_armor_points_;

  cv::Mat rvec;   // 旋转向量
  cv::Mat tvec;   // 平移向量

  bool success = cv::solvePnP(   //判断解算是否成功
    object_points,
    armor.points,
    camera_matrix_,
    dist_coeffs_,
    rvec,
    tvec,
    false,
    cv::SOLVEPNP_IPPE);

  if (!success) {
    armor.has_pose = false;
    return false;
  }

  armor.rvec = rvec.clone();
  armor.tvec = tvec.clone();
  armor.has_pose = true;

  return true;
}

} 