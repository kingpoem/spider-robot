cmake_minimum_required(VERSION 3.8)
project(hexapod_agriculture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(agriculture_node src/agriculture_node.cpp)
ament_target_dependencies(agriculture_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

add_executable(monitor_node src/monitor_node.cpp)
ament_target_dependencies(monitor_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

install(TARGETS agriculture_node monitor_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

ament_package()

