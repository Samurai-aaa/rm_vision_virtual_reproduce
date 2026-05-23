// PnP位姿态解算器
#ifndef SIMPLE_ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define SIMPLE_ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include "simple_armor_detector/armor.hpp"

#include <opencv2/opencv.hpp>

#include <array>
#include <vector>

namespace simple_armor_detector
{

class PnPSolver
{
public:
  struct Params         // 参数结构体
  {
    double small_armor_width = 135.0;   // mm
    double small_armor_height = 55.0;   // mm
    double large_armor_width = 225.0;   // mm
    double large_armor_height = 55.0;   // mm
  };

  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & dist_coeffs,
    const Params & params);

  bool solvePnP(SimpleArmor & armor);                                // 输入装甲板信息，输出位姿信息，返回是否成功解算
  
  float calculateDistanceToCenter(const cv::Point2f & image_point);  // 计算图像点到图像中心的距离

private:
  cv::Mat camera_matrix_;    // 相机内参矩阵
  cv::Mat dist_coeffs_;      // 相机畸变系数

  std::vector<cv::Point3f> small_armor_points_;     // 小装甲板的3D坐标点
  std::vector<cv::Point3f> large_armor_points_;     // 大装甲板的3D坐标点
};

}

#endif
