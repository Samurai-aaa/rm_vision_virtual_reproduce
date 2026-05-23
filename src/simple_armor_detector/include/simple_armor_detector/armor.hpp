#ifndef SIMPLE_ARMOR_DETECTOR__ARMOR_HPP_
#define SIMPLE_ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <algorithm>

namespace simple_armor_detector
{

struct Light                  // 灯条结构体
{
  cv::RotatedRect rect;
  cv::Point2f center;

  float width = 0.0f;
  float height = 0.0f;
  float angle = 0.0f;

  // 灯条上下端点，用于后续匹配装甲板四角点
  cv::Point2f top;
  cv::Point2f bottom;
};

struct SimpleArmor            // 装甲板结构体
{
  Light left_light;
  Light right_light;

  cv::Point2f center;

  // 顺序：top_left, top_right, bottom_right, bottom_left
  std::vector<cv::Point2f> points;

  std::string type = "small";

  float distance_to_image_center = 0.0f;

  cv::Mat rvec;
  cv::Mat tvec;
  bool has_pose = false;
};

}  // namespace simple_armor_detector

#endif  // SIMPLE_ARMOR_DETECTOR__ARMOR_HPP_
