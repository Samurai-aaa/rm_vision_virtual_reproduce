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

add_executable(simple_detector_node src/simple_detector_node.cpp)

ament_target_dependencies(simple_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  visualization_msgs
  geometry_msgs
)

target_include_directories(simple_detector_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
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

ament_package()
