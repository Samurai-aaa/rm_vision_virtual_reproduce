#ifndef SIMPLE_ARMOR_TRACKER__EXTENDED_KALMAN_FILTER_HPP_
#define SIMPLE_ARMOR_TRACKER__EXTENDED_KALMAN_FILTER_HPP_

#include <opencv2/opencv.hpp>

namespace simple_armor_tracker
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter();

  void init(const cv::Point3f & position);

  cv::Point3f predict(double dt);

  cv::Point3f update(const cv::Point3f & measurement);

  cv::Point3f getPosition() const;

private:
  cv::KalmanFilter kf_;
  bool initialized_ = false;
};

}  // namespace simple_armor_tracker

#endif  // SIMPLE_ARMOR_TRACKER__EXTENDED_KALMAN_FILTER_HPP_