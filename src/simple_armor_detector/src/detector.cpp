#include "simple_armor_detector/detector.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace simple_armor_detector
{

Detector::Detector(const Params & params) : params_(params) {}

// 二值化
cv::Mat Detector::preprocessImage(const cv::Mat & img)
{
  std::vector<cv::Mat> channels;
  cv::split(img, channels);

  cv::Mat blue = channels[0];
  cv::Mat red = channels[2];

  cv::Mat diff;

  if (params_.enemy_color == "blue") {                          // 差分图像
    cv::subtract(blue, red, diff);
  } else {
    cv::subtract(red, blue, diff);
  }

  cv::Mat binary;                                              // 根据差分图像进行二值化
  cv::threshold(diff, binary, params_.binary_thres, 255, cv::THRESH_BINARY);

  int kw = std::max(1, params_.morph_kernel_width);            // 构建形态学核
  int kh = std::max(1, params_.morph_kernel_height);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kw, kh));

  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);   // 闭运算，填充灯条内部的空洞
  cv::dilate(binary, binary, kernel);                          // 膨胀，连接断开的灯条

  return binary;
}

// 灯条检测
std::vector<Light> Detector::findLights(const cv::Mat & binary)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<Light> lights;

  for (const auto & contour : contours) {                   // 遍历所有轮廓
    if (contour.size() < 5) {
      continue;
    }

    double area = cv::contourArea(contour);                 // 过滤掉面积过小的轮廓
    if (area < params_.min_light_area) {
      continue;
    }

    cv::RotatedRect rect = cv::minAreaRect(contour);        // 旋转矩形拟合，得到灯条的中心点、宽高、角度等信息
    Light light = makeLight(rect);

    if (isValidLight(light)) {                              // 将符合条件的灯条保存下来
      lights.emplace_back(light);
    }
  }

  std::sort(lights.begin(), lights.end(), [](const Light & a, const Light & b) {     // 按 x 坐标排序，优先选择靠近图像中心的装甲板
    return a.center.x < b.center.x;
  });

  return lights;
}

// 创建灯条
Light Detector::makeLight(const cv::RotatedRect & rect)
{
  cv::Point2f pts[4];
  rect.points(pts);

  // 先按 y 坐标排序，前两个是上方点，后两个是下方点
  std::vector<cv::Point2f> vertices(pts, pts + 4);

  std::sort(vertices.begin(), vertices.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  cv::Point2f top = (vertices[0] + vertices[1]) * 0.5f;
  cv::Point2f bottom = (vertices[2] + vertices[3]) * 0.5f;

  float w = rect.size.width;
  float h = rect.size.height;
  float angle = rect.angle;

  if (w > h) {             // OpenCV 的 RotatedRect 可能会将宽高颠倒，并且角度范围是 [-90, 0)，需要调整
    std::swap(w, h);
    angle += 90.0f;
  }

  if (angle > 90.0f) {     // 将角度调整到 [-90, 90) 范围内
    angle -= 180.0f;
  }

  if (angle < -90.0f) {
    angle += 180.0f;
  }

  Light light;             // 填充 Light 结构体
  light.rect = rect;
  light.center = rect.center;
  light.width = w;
  light.height = h;
  light.angle = angle;
  light.top = top;
  light.bottom = bottom;

  return light;
}

// 灯条筛选
bool Detector::isValidLight(const Light & light)
{
  if (light.width <= 1e-3 || light.height <= 1e-3) {       // 过滤掉宽高过小的灯条
    return false;
  }

  double area = light.width * light.height;
  double ratio = light.height / light.width;

  if (area < params_.min_light_area) {                     // 筛选面积过小的灯条
    return false;
  }

  if (ratio < params_.min_light_ratio) {                   // 筛选长宽比不符合要求的灯条
    return false;
  }

  if (std::abs(light.angle) > params_.max_light_angle) {   // 筛选角度过大的灯条
    return false;
  }

  return true;
}

// 装甲板匹配
std::vector<SimpleArmor> Detector::matchArmors(const std::vector<Light> & lights)
{
  std::vector<SimpleArmor> armors;

  for (size_t i = 0; i < lights.size(); ++i) {                  // 遍历所有灯条组合
    for (size_t j = i + 1; j < lights.size(); ++j) {
      const auto & left = lights[i];
      const auto & right = lights[j];

      if (!isValidArmorPair(left, right)) {                     // 过滤掉不符合条件的装甲板组合
        continue;
      }
      
      // 填充装甲板信息
      SimpleArmor armor;
      armor.left_light = left;
      armor.right_light = right;
      armor.center = (left.center + right.center) * 0.5f;
      armor.points = getArmorPoints(left, right);
      armor.type = judgeArmorType(left, right);

      armors.emplace_back(armor);
    }
  }

  return armors;
}

// 装甲板筛选
bool Detector::isValidArmorPair(const Light & left, const Light & right)
{
  double angle_diff = std::abs(left.angle - right.angle);

  if (angle_diff > params_.max_angle_diff) {                      // 筛选角度差过大的灯条组合
    return false;
  }

  double height_diff = std::abs(left.height - right.height);
  double max_height = std::max(left.height, right.height);

  if (max_height < 1e-3) {                                        // 过滤掉高度过小的灯条组合，避免除零错误
    return false;
  }

  if (height_diff / max_height > params_.max_height_diff_ratio) { // 筛选高度差过大的灯条组合
    return false;
  }

  double avg_height = (left.height + right.height) / 2.0;
  double y_diff = std::abs(left.center.y - right.center.y);

  if (y_diff / avg_height > params_.max_y_diff_ratio) {           // 筛选 y 坐标差过大的灯条组合
    return false;
  }

  double x_diff = std::abs(left.center.x - right.center.x);

  if (x_diff < avg_height * 0.5) {                                // 筛选 x 坐标差过小的灯条组合，避免误匹配
    return false;
  }

  if (x_diff > params_.max_light_x_distance) {                    // 筛选 x 坐标差过大的灯条组合，避免误匹配
    return false;
  }

  double armor_ratio = x_diff / avg_height;

  if (armor_ratio < params_.min_armor_ratio || armor_ratio > params_.max_armor_ratio) {   // 筛选装甲板长宽比不符合要求的灯条组合
    return false;
  }

  return true;
}

// 从两条灯条的端点计算装甲板的四个顶点坐标，顺序为：top_left, top_right, bottom_right, bottom_left
std::vector<cv::Point2f> Detector::getArmorPoints(const Light & left, const Light & right)
{
  std::vector<cv::Point2f> armor_pts;

  // 顺序：top_left, top_right, bottom_right, bottom_left
  armor_pts.emplace_back(left.top);
  armor_pts.emplace_back(right.top);
  armor_pts.emplace_back(right.bottom);
  armor_pts.emplace_back(left.bottom);

  return armor_pts;
}

// 装甲板类型判断
std::string Detector::judgeArmorType(const Light & left, const Light & right)
{
  double avg_height = (left.height + right.height) / 2.0;
  double x_diff = std::abs(left.center.x - right.center.x);
  double ratio = x_diff / avg_height;

  if (ratio > 3.2) {
    return "large";
  }

  return "small";
}

// 计算装甲板中心点到图像中心的距离
float Detector::calculateDistanceToCenter(
  const cv::Point2f & image_point,
  const cv::Size & image_size)
{
  cv::Point2f image_center(
    static_cast<float>(image_size.width) / 2.0f,
    static_cast<float>(image_size.height) / 2.0f);

  float dx = image_point.x - image_center.x;
  float dy = image_point.y - image_center.y;

  return std::sqrt(dx * dx + dy * dy);
}

// 绘制检测结果
void Detector::drawResults(
  cv::Mat & img,
  const std::vector<Light> & lights,
  const std::vector<SimpleArmor> & armors)
{
  // 1. 绘制灯条：绿色中心线 + 端点
  for (const auto & light : lights) {
    cv::line(img, light.top, light.bottom, cv::Scalar(0, 255, 0), 1);

    cv::circle(img, light.top, 2, cv::Scalar(0, 255, 255), -1);
    cv::circle(img, light.bottom, 2, cv::Scalar(0, 255, 255), -1);
    cv::circle(img, light.center, 2, cv::Scalar(255, 0, 0), -1);
  }

  // 2. 绘制装甲板：红色四边形连线
  for (const auto & armor : armors) {
    if (armor.points.size() != 4) {
      continue;
    }

    const auto & pts = armor.points;

    // top_left -> top_right
    cv::line(img, pts[0], pts[1], cv::Scalar(0, 0, 255), 1);

    // top_right -> bottom_right
    cv::line(img, pts[1], pts[2], cv::Scalar(0, 0, 255), 1);

    // bottom_right -> bottom_left
    cv::line(img, pts[2], pts[3], cv::Scalar(0, 0, 255), 1);

    // bottom_left -> top_left
    cv::line(img, pts[3], pts[0], cv::Scalar(0, 0, 255), 1);

    // bottom_left -> top_left
    cv::line(img, pts[0], pts[2], cv::Scalar(0, 0, 255), 1);

    // bottom_left -> top_left
    cv::line(img, pts[1], pts[3], cv::Scalar(0, 0, 255), 1);

    // 装甲板中心点
    cv::circle(img, armor.center, 3, cv::Scalar(255, 0, 0), -1);

    std::string info =
      armor.type + " d=" + std::to_string(static_cast<int>(armor.distance_to_image_center));
      
    if (armor.has_pose && !armor.tvec.empty()) {
      double z = armor.tvec.at<double>(2);
      info += " z=" + std::to_string(z).substr(0, 4) + "m";
    }

    cv::putText(
      img,
      info,
      armor.center + cv::Point2f(5.0f, -5.0f),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(0, 255, 255),
      1);
  }

  // 3. 左上角统计信息
  std::string text =
    "lights: " + std::to_string(lights.size()) +
    "  armors: " + std::to_string(armors.size());

  cv::putText(
    img,
    text,
    cv::Point(20, 40),
    cv::FONT_HERSHEY_SIMPLEX,
    0.8,
    cv::Scalar(0, 255, 255),
    1);
}

}