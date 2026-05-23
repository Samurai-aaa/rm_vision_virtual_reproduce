#ifndef SIMPLE_ARMOR_DETECTOR__DETECTOR_HPP_
#define SIMPLE_ARMOR_DETECTOR__DETECTOR_HPP_

#include "simple_armor_detector/armor.hpp"

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

namespace simple_armor_detector
{

class Detector
{
public:
  struct Params        // 参数结构体
  {
    // 颜色相关参数
    std::string enemy_color = "red";
    
    // 二值化参数
    int binary_thres = 65;

    // 灯条参数
    double min_light_area = 5.0;
    double min_light_ratio = 2.5;
    double max_light_angle = 40.0;
    
    // 装甲板匹配参数
    double max_angle_diff = 15.0;
    double max_height_diff_ratio = 0.3;
    double max_y_diff_ratio = 0.3;
    double max_armor_tilt_angle = 15.0;
    
    // 装甲板类型判断参数
    double min_armor_ratio = 0.6;
    double max_armor_ratio = 4.0;
    double max_light_x_distance = 250.0;

    // 形态学操作参数
    int morph_kernel_width = 3;
    int morph_kernel_height = 9;
  };

  explicit Detector(const Params & params);

  cv::Mat preprocessImage(const cv::Mat & img);

  std::vector<Light> findLights(const cv::Mat & binary);

  std::vector<SimpleArmor> matchArmors(const std::vector<Light> & lights);

  float calculateDistanceToCenter(
  const cv::Point2f & image_point,
  const cv::Size & image_size);

  void drawResults(
    cv::Mat & img,
    const std::vector<Light> & lights,
    const std::vector<SimpleArmor> & armors);

private:
  Params params_;

  Light makeLight(const cv::RotatedRect & rect);

  bool isValidLight(const Light & light);

  bool isValidArmorPair(const Light & left, const Light & right);

    bool containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights);

  std::vector<cv::Point2f> getArmorPoints(const Light & left, const Light & right);

  std::string judgeArmorType(const Light & left, const Light & right);
};

}

#endif