cmake_minimum_required(VERSION 3.8)
project(simple_armor_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(simple_armor_interfaces REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(simple_tracker_node
  src/tracker_node.cpp
  src/tracker.cpp
  src/extended_kalman_filter.cpp
)

ament_target_dependencies(simple_tracker_node
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
  simple_armor_interfaces
)

target_link_libraries(simple_tracker_node
  ${OpenCV_LIBS}
)

install(TARGETS
  simple_tracker_node
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