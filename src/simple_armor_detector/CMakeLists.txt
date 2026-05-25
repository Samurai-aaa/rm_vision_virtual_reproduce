# 此文件的作用是定义如何构建和安装simple_armor_detector包。
# 它指定了所需的依赖项、编译选项、源文件以及安装规则。

cmake_minimum_required(VERSION 3.8)
project(simple_armor_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(simple_armor_interfaces REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(simple_detector_node
  src/simple_detector_node.cpp
  src/detector.cpp
  src/pnp_solver.cpp
)

ament_target_dependencies(simple_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  visualization_msgs
  geometry_msgs
  simple_armor_interfaces
)

target_link_libraries(simple_detector_node
  ${OpenCV_LIBS}
)

install(TARGETS
  simple_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
