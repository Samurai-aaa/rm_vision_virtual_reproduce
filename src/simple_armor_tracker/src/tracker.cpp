#include "simple_armor_tracker/tracker.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace simple_armor_tracker
{

Tracker::Tracker(const Params & params) : params_(params) {}

Tracker::Result Tracker::update(
  const std::vector<cv::Point3f> & measurements,
  double dt)
{
  Result result;
  result.state = state_;

  if (!initialized_) {
    if (measurements.empty()) {
      state_ = State::LOST;
      result.has_target = false;
      result.state = state_;
      return result;
    }

    // 初始目标：选择 z 最小的目标，也就是离相机最近的目标
    cv::Point3f init_measurement = measurements.front();

    for (const auto & p : measurements) {
      if (p.z < init_measurement.z) {
        init_measurement = p;
      }
    }

    ekf_.init(init_measurement);
    last_position_ = init_measurement;

    initialized_ = true;
    detect_count_ = 1;
    lost_count_ = 0;
    state_ = State::DETECTING;

    result.has_target = true;
    result.position = init_measurement;
    result.state = state_;

    return result;
  }

  cv::Point3f predicted_position = ekf_.predict(dt);

  cv::Point3f selected_measurement;
  bool matched = selectMeasurement(measurements, predicted_position, selected_measurement);

  if (matched) {
    cv::Point3f filtered_position = ekf_.update(selected_measurement);

    last_position_ = filtered_position;

    detect_count_++;
    lost_count_ = 0;

    if (detect_count_ >= params_.tracking_threshold) {
      state_ = State::TRACKING;
    } else {
      state_ = State::DETECTING;
    }

    result.has_target = true;
    result.position = filtered_position;
    result.state = state_;

    return result;
  }

  // 没有匹配到观测，进入短暂丢失
  lost_count_++;
  detect_count_ = 0;

  if (lost_count_ > params_.lost_threshold) {
    state_ = State::LOST;
    initialized_ = false;

    result.has_target = false;
    result.state = state_;
    return result;
  }

  state_ = State::TEMP_LOST;
  last_position_ = predicted_position;

  result.has_target = true;
  result.position = predicted_position;
  result.state = state_;

  return result;
}

bool Tracker::selectMeasurement(
  const std::vector<cv::Point3f> & measurements,
  const cv::Point3f & predicted_position,
  cv::Point3f & selected_measurement)
{
  if (measurements.empty()) {
    return false;
  }

  double min_distance = std::numeric_limits<double>::max();
  bool found = false;

  for (const auto & p : measurements) {
    double d = distance3D(p, predicted_position);

    if (d < min_distance) {
      min_distance = d;
      selected_measurement = p;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  if (state_ == State::TRACKING || state_ == State::TEMP_LOST) {
    if (min_distance > params_.max_match_distance) {
      return false;
    }
  }

  return true;
}

double Tracker::distance3D(const cv::Point3f & a, const cv::Point3f & b) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Tracker::State Tracker::getState() const
{
  return state_;
}

std::string Tracker::getStateString() const
{
  switch (state_) {
    case State::LOST:
      return "LOST";
    case State::DETECTING:
      return "DETECTING";
    case State::TRACKING:
      return "TRACKING";
    case State::TEMP_LOST:
      return "TEMP_LOST";
    default:
      return "UNKNOWN";
  }
}

}  // namespace simple_armor_tracker