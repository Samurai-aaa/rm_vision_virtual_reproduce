#ifndef SIMPLE_ARMOR_TRACKER__TRACKER_HPP_
#define SIMPLE_ARMOR_TRACKER__TRACKER_HPP_

#include "simple_armor_tracker/extended_kalman_filter.hpp"

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

namespace simple_armor_tracker
{

class Tracker
{
public:
  enum class State
  {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST
  };

  struct Params
  {
    double max_match_distance = 0.8;
    int tracking_threshold = 3;
    int lost_threshold = 10;
  };

  struct Result
  {
    bool has_target = false;
    cv::Point3f position;
    State state = State::LOST;
  };

  explicit Tracker(const Params & params);

  Result update(const std::vector<cv::Point3f> & measurements, double dt);

  State getState() const;

  std::string getStateString() const;

private:
  bool selectMeasurement(
    const std::vector<cv::Point3f> & measurements,
    const cv::Point3f & predicted_position,
    cv::Point3f & selected_measurement);

  double distance3D(const cv::Point3f & a, const cv::Point3f & b) const;

private:
  Params params_;

  ExtendedKalmanFilter ekf_;

  State state_ = State::LOST;

  bool initialized_ = false;

  int detect_count_ = 0;
  int lost_count_ = 0;

  cv::Point3f last_position_;
};

}  // namespace simple_armor_tracker

#endif  // SIMPLE_ARMOR_TRACKER__TRACKER_HPP_