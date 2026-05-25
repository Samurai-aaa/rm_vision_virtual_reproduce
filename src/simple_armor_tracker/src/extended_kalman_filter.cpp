#include "simple_armor_tracker/extended_kalman_filter.hpp"

namespace simple_armor_tracker
{

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
  // 状态: x, y, z, vx, vy, vz
  // 观测: x, y, z
  kf_ = cv::KalmanFilter(6, 3, 0, CV_32F);

  kf_.transitionMatrix = cv::Mat::eye(6, 6, CV_32F);

  kf_.measurementMatrix = cv::Mat::zeros(3, 6, CV_32F);
  kf_.measurementMatrix.at<float>(0, 0) = 1.0f;
  kf_.measurementMatrix.at<float>(1, 1) = 1.0f;
  kf_.measurementMatrix.at<float>(2, 2) = 1.0f;

  cv::setIdentity(kf_.processNoiseCov, cv::Scalar(1e-2));
  cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(5e-2));
  cv::setIdentity(kf_.errorCovPost, cv::Scalar(1.0));
}

void ExtendedKalmanFilter::init(const cv::Point3f & position)
{
  kf_.statePost = cv::Mat::zeros(6, 1, CV_32F);

  kf_.statePost.at<float>(0) = position.x;
  kf_.statePost.at<float>(1) = position.y;
  kf_.statePost.at<float>(2) = position.z;

  kf_.statePost.at<float>(3) = 0.0f;
  kf_.statePost.at<float>(4) = 0.0f;
  kf_.statePost.at<float>(5) = 0.0f;

  initialized_ = true;
}

cv::Point3f ExtendedKalmanFilter::predict(double dt)
{
  if (!initialized_) {
    return cv::Point3f(0.0f, 0.0f, 0.0f);
  }

  kf_.transitionMatrix.at<float>(0, 3) = static_cast<float>(dt);
  kf_.transitionMatrix.at<float>(1, 4) = static_cast<float>(dt);
  kf_.transitionMatrix.at<float>(2, 5) = static_cast<float>(dt);

  cv::Mat pred = kf_.predict();

  return cv::Point3f(
    pred.at<float>(0),
    pred.at<float>(1),
    pred.at<float>(2));
}

cv::Point3f ExtendedKalmanFilter::update(const cv::Point3f & measurement)
{
  cv::Mat measure = cv::Mat::zeros(3, 1, CV_32F);

  measure.at<float>(0) = measurement.x;
  measure.at<float>(1) = measurement.y;
  measure.at<float>(2) = measurement.z;

  cv::Mat estimated = kf_.correct(measure);

  return cv::Point3f(
    estimated.at<float>(0),
    estimated.at<float>(1),
    estimated.at<float>(2));
}

cv::Point3f ExtendedKalmanFilter::getPosition() const
{
  return cv::Point3f(
    kf_.statePost.at<float>(0),
    kf_.statePost.at<float>(1),
    kf_.statePost.at<float>(2));
}

}  // namespace simple_armor_tracker