cmake_minimum_required(VERSION 3.8)
project(my_vision_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(vision_node src/vision_node.cpp)
add_executable(camera_info_node src/camera_info_node.cpp)

ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

ament_target_dependencies(camera_info_node
  rclcpp
  sensor_msgs
)

target_include_directories(vision_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(vision_node
  ${OpenCV_LIBS}
)

install(TARGETS
  vision_node
  camera_info_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
