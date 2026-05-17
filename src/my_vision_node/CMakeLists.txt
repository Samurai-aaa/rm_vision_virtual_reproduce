cmake_minimum_required(VERSION 3.8)

project(my_vision_node)

# 如果使用 GCC 或 Clang 编译器，则开启常用警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准为 C++17。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 ROS 2 构建系统 ament_cmake
# 所有 ament_cmake 类型的 ROS 2 C++ 包都需要它
find_package(ament_cmake REQUIRED)

# 查找 ROS 2 C++ 客户端库。
# rclcpp 用于创建 Node、Publisher、Timer、Parameter 等
find_package(rclcpp REQUIRED)

# 查找 sensor_msgs
find_package(sensor_msgs REQUIRED)

# 查找 std_msgs
find_package(std_msgs REQUIRED)

# 查找 cv_bridge
# cv_bridge 用于 OpenCV cv::Mat 和 ROS 2 Image 消息之间转换
find_package(cv_bridge REQUIRED)

# 查找 OpenCV
# vision_node 使用 OpenCV 的 VideoCapture、Mat 等
find_package(OpenCV REQUIRED)


# 将 src/vision_node.cpp 编译成可执行文件 vision_node
add_executable(vision_node src/vision_node.cpp)

# 将 src/camera_info_node.cpp 编译成可执行文件 camera_info_node
add_executable(camera_info_node src/camera_info_node.cpp)


# 声明 vision_node 依赖的 ROS 2 包
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# 声明 camera_info_node 依赖的 ROS 2 包
# camera_info_node 不需要 OpenCV，也不需要 cv_bridge
ament_target_dependencies(camera_info_node
  rclcpp
  sensor_msgs
)


# 给 vision_node 添加 OpenCV 头文件路径
target_include_directories(vision_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

# 将 OpenCV 库链接到 vision_node
# 因为 vision_node 使用 cv::VideoCapture、cv::Mat 等 OpenCV 功能
target_link_libraries(vision_node
  ${OpenCV_LIBS}
)


# 安装两个可执行文件
install(TARGETS
  vision_node
  camera_info_node
  DESTINATION lib/${PROJECT_NAME}
)


# 安装 launch 和 config 目录
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()