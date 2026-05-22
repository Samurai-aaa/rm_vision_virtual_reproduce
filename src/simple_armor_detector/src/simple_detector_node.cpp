#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

struct Light                 // 定义灯条结构体
{
  cv::RotatedRect rect;
  cv::Point2f center;
  float width;
  float height;
  float angle;
};

struct SimpleArmor           // 定义装甲板结构体
{
  Light left_light;
  Light right_light;
  cv::Point2f center;
  std::vector<cv::Point2f> points;
};

// 装甲检测节点
class SimpleArmorDetectorNode : public rclcpp::Node
{
public:
  SimpleArmorDetectorNode() : Node("simple_armor_detector")
  {
    declareParams();
    loadParams();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(                         // 订阅图像话题
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&SimpleArmorDetectorNode::imageCallback, this, std::placeholders::_1));

    binary_pub_ = image_transport::create_publisher(this, "/simple_detector/binary_img");    // 发布二值化图像
    result_pub_ = image_transport::create_publisher(this, "/simple_detector/result_img");    // 发布结果图像

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(              // 发布装甲板标记
      "/simple_detector/markers", 10);

    RCLCPP_INFO(this->get_logger(), "simple_armor_detector started.");                       // 输出日志
    RCLCPP_INFO(this->get_logger(), "Subscribe image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Enemy color: %s", enemy_color_.c_str());
  }

private:
  // 声明参数
  void declareParams()
  {
    // 基础参数
    this->declare_parameter<std::string>("image_topic", "/image_raw");
    this->declare_parameter<std::string>("enemy_color", "red");

    // 灯条检测参数
    this->declare_parameter<int>("binary_thres", 60);
    this->declare_parameter<double>("min_light_area", 5.0);
    this->declare_parameter<double>("min_light_ratio", 1.2);
    this->declare_parameter<double>("max_light_angle", 45.0);
    
    // 装甲板匹配参数
    this->declare_parameter<double>("max_angle_diff", 25.0);
    this->declare_parameter<double>("max_height_diff_ratio", 0.7);
    this->declare_parameter<double>("max_y_diff_ratio", 1.0);
    this->declare_parameter<double>("min_armor_ratio", 0.8);
    this->declare_parameter<double>("max_armor_ratio", 6.0);
    this->declare_parameter<double>("max_light_x_distance", 250.0);

    // 形态学核参数
    this->declare_parameter<int>("morph_kernel_width", 3);
    this->declare_parameter<int>("morph_kernel_height", 7);
  }

  // 加载参数
  void loadParams()
  {
    image_topic_ = this->get_parameter("image_topic").as_string();
    enemy_color_ = this->get_parameter("enemy_color").as_string();

    binary_thres_ = this->get_parameter("binary_thres").as_int();
    min_light_area_ = this->get_parameter("min_light_area").as_double();
    min_light_ratio_ = this->get_parameter("min_light_ratio").as_double();
    max_light_angle_ = this->get_parameter("max_light_angle").as_double();

    max_angle_diff_ = this->get_parameter("max_angle_diff").as_double();
    max_height_diff_ratio_ = this->get_parameter("max_height_diff_ratio").as_double();
    max_y_diff_ratio_ = this->get_parameter("max_y_diff_ratio").as_double();
    min_armor_ratio_ = this->get_parameter("min_armor_ratio").as_double();
    max_armor_ratio_ = this->get_parameter("max_armor_ratio").as_double();
    max_light_x_distance_ = this->get_parameter("max_light_x_distance").as_double();


    morph_kernel_width_ = this->get_parameter("morph_kernel_width").as_int();
    morph_kernel_height_ = this->get_parameter("morph_kernel_height").as_int();
  }

  // 图像回调函数
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat img;   // 将ROS图像消息转换为OpenCV图像

    try {
      img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (img.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image.");
      return;
    }

    cv::Mat binary = preprocessImage(img);
    std::vector<Light> lights = findLights(binary);
    std::vector<SimpleArmor> armors = matchArmors(lights);

    cv::Mat result = img.clone();
    drawResults(result, lights, armors);
    publishMarkers(msg->header, armors);

    publishImage(binary_pub_, binary, msg->header, "mono8");
    publishImage(result_pub_, result, msg->header, "bgr8");
  }

  // 图像预处理:将图像转换为二值图像，突出显示敌人颜色的灯条
  cv::Mat preprocessImage(const cv::Mat & img)
  {
    std::vector<cv::Mat> channels;      // 分离颜色通道
    cv::split(img, channels);

    cv::Mat blue = channels[0];
    cv::Mat red = channels[2];

    cv::Mat diff;                       // 计算颜色差分

    if (enemy_color_ == "blue") {       // 根据敌人灯条颜色选择通道
      cv::subtract(blue, red, diff);
    } else {
      cv::subtract(red, blue, diff);
    }

    cv::Mat binary;                     // 二值化
    cv::threshold(diff, binary, binary_thres_, 255, cv::THRESH_BINARY);

    int kw = std::max(1, morph_kernel_width_);                   // 创建形态学核
    int kh = std::max(1, morph_kernel_height_);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kw, kh));

    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);   // 闭运算：先膨胀后腐蚀
    cv::dilate(binary, binary, kernel);

    return binary;
  }

  // 灯条检测
  std::vector<Light> findLights(const cv::Mat & binary)
  {
    std::vector<std::vector<cv::Point>> contours;           // 查找轮廓
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Light> lights;

    for (const auto & contour : contours) {
      if (contour.size() < 5) {                            // 筛选掉点数过少的轮廓
        continue;
      }

      double area = cv::contourArea(contour);              // 筛选掉面积过小的轮廓
      if (area < min_light_area_) {
        continue;
      }

      cv::RotatedRect rect = cv::minAreaRect(contour);     // 旋转矩形拟合轮廓
      Light light = makeLight(rect);

      if (isValidLight(light)) {                           // 将每一个有效的灯条添加到列表中
        lights.emplace_back(light);
      }
    }

    std::sort(lights.begin(), lights.end(), [](const Light & a, const Light & b) {    // 按照灯条中心的x坐标排序
      return a.center.x < b.center.x;
    });

    return lights;
  }

  // 根据旋转矩形创建灯条对象
  Light makeLight(const cv::RotatedRect & rect)
  {
    float w = rect.size.width;
    float h = rect.size.height;
    float angle = rect.angle;

    if (w > h) {
      std::swap(w, h);
      angle += 90.0f;
    }

    if (angle > 90.0f) {
      angle -= 180.0f;
    }

    if (angle < -90.0f) {
      angle += 180.0f;
    }

    Light light;
    light.rect = rect;
    light.center = rect.center;
    light.width = w;
    light.height = h;
    light.angle = angle;

    return light;
  }

  // 检查灯条是否有效
  bool isValidLight(const Light & light)
  {
    if (light.width <= 1e-3 || light.height <= 1e-3) {      // 筛选掉宽高过小的灯条
      return false;
    }

    double area = light.width * light.height;
    double ratio = light.height / light.width;

    if (area < min_light_area_) {                           // 筛选掉面积过小的灯条
      return false;
    }

    if (ratio < min_light_ratio_) {                         // 筛选掉长宽比不符合要求的灯条
      return false;
    }

    if (std::abs(light.angle) > max_light_angle_) {         // 筛选掉角度过大的灯条
      return false;
    }

    return true;
  }

  // 灯条匹配成装甲板
  std::vector<SimpleArmor> matchArmors(const std::vector<Light> & lights)
  {
    std::vector<SimpleArmor> armors;

    for (size_t i = 0; i < lights.size(); ++i) {
      for (size_t j = i + 1; j < lights.size(); ++j) {
        const auto & left = lights[i];
        const auto & right = lights[j];

        if (!isValidArmorPair(left, right)) {                   // 检查灯条对是否可以组成装甲板
          continue;
        }

        SimpleArmor armor;
        armor.left_light = left;
        armor.right_light = right;
        armor.center = (left.center + right.center) * 0.5f;
        armor.points = getArmorPoints(left, right);

        armors.emplace_back(armor);
      }
    }

    return armors;
  }

  // 检查灯条对是否可以组成装甲板
  bool isValidArmorPair(const Light & left, const Light & right)
  {
    double angle_diff = std::abs(left.angle - right.angle);                      // 检查灯条角度差异
    if (angle_diff > max_angle_diff_) {
      return false;
    }

    double height_diff = std::abs(left.height - right.height);                   // 检查灯条高度差异
    double max_height = std::max(left.height, right.height);
    if (max_height < 1e-3) {
      return false;
    }

    if (height_diff / max_height > max_height_diff_ratio_) {                     // 检查灯条高度差异是否超过比例阈值
      return false;
    }

    double y_diff = std::abs(left.center.y - right.center.y);
    double avg_height = (left.height + right.height) / 2.0;

    if (y_diff / avg_height > max_y_diff_ratio_) {                               // 检查灯条中心y坐标差异是否超过比例阈值
      return false;
    }

    double x_diff = std::abs(left.center.x - right.center.x);                    // 检查灯条中心x是否太接近
    if (x_diff < avg_height * 0.5) {
      return false;
    }

    if (x_diff > max_light_x_distance_) {
      return false;
    } 

    double armor_ratio = x_diff / avg_height;                                    // 检查装甲板长宽比是否符合要求

    if (armor_ratio < min_armor_ratio_ || armor_ratio > max_armor_ratio_) {
      return false;
    }

    return true;
  }

  // 根据左右灯条的顶点计算装甲板的四个顶点
  std::vector<cv::Point2f> getArmorPoints(const Light & left, const Light & right)
  {
    cv::Point2f left_pts[4];
    cv::Point2f right_pts[4];

    left.rect.points(left_pts);
    right.rect.points(right_pts);

    std::vector<cv::Point2f> all_pts;                  // 包住两个灯条的八个点

    for (int i = 0; i < 4; ++i) {
      all_pts.emplace_back(left_pts[i]);
      all_pts.emplace_back(right_pts[i]);
    }

    cv::Rect bbox = cv::boundingRect(all_pts);

    std::vector<cv::Point2f> armor_pts;
    armor_pts.emplace_back(cv::Point2f(bbox.x, bbox.y));
    armor_pts.emplace_back(cv::Point2f(bbox.x + bbox.width, bbox.y));
    armor_pts.emplace_back(cv::Point2f(bbox.x + bbox.width, bbox.y + bbox.height));
    armor_pts.emplace_back(cv::Point2f(bbox.x, bbox.y + bbox.height));

    return armor_pts;
  }

  // 在图像上绘制检测结果
  void drawResults(
    cv::Mat & img,
    const std::vector<Light> & lights,
    const std::vector<SimpleArmor> & armors)
  {
    for (const auto & light : lights) {
      cv::Point2f pts[4];
      light.rect.points(pts);

      for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
      }
    }

    for (const auto & armor : armors) {
      const auto & pts = armor.points;

      for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 0, 255), 1);
      }

      cv::circle(img, armor.center, 2, cv::Scalar(255, 0, 0), -1);
    }

    std::string text =
      "lights: " + std::to_string(lights.size()) +
      "  armors: " + std::to_string(armors.size());

    cv::putText(
      img,
      text,
      cv::Point(20, 40),
      cv::FONT_HERSHEY_SIMPLEX,
      1.0,
      cv::Scalar(0, 255, 255),
      2);
  }

  // 发布装甲板标记
  void publishMarkers(
    const std_msgs::msg::Header & header,
    const std::vector<SimpleArmor> & armors)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.ns = "simple_armors";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.emplace_back(clear_marker);

    int id = 1;

    for (const auto & armor : armors) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = "simple_armors";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.scale.x = 0.01;

      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      for (const auto & pt : armor.points) {
        geometry_msgs::msg::Point p;
        p.x = pt.x / 1000.0;
        p.y = pt.y / 1000.0;
        p.z = 0.0;
        marker.points.emplace_back(p);
      }

      if (!armor.points.empty()) {
        geometry_msgs::msg::Point p;
        p.x = armor.points[0].x / 1000.0;
        p.y = armor.points[0].y / 1000.0;
        p.z = 0.0;
        marker.points.emplace_back(p);
      }

      marker_array.markers.emplace_back(marker);
    }

    marker_pub_->publish(marker_array);
  }

  // 发布图像消息
  void publishImage(
    image_transport::Publisher & pub,
    const cv::Mat & image,
    const std_msgs::msg::Header & header,
    const std::string & encoding)
  {
    auto msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    pub.publish(msg);
  }

private:
  std::string image_topic_;
  std::string enemy_color_;

  int binary_thres_;
  double min_light_area_;
  double min_light_ratio_;
  double max_light_angle_;

  double max_angle_diff_;
  double max_height_diff_ratio_;
  double max_y_diff_ratio_;
  double min_armor_ratio_;
  double max_armor_ratio_;
  double max_light_x_distance_;

  int morph_kernel_width_;
  int morph_kernel_height_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  image_transport::Publisher binary_pub_;
  image_transport::Publisher result_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleArmorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
