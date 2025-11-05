cmake_minimum_required(VERSION 3.8)
project(hexapod_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mock_sensors src/mock_sensors.cpp)
ament_target_dependencies(mock_sensors
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

add_executable(mock_arduino src/mock_arduino.cpp)
ament_target_dependencies(mock_arduino
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS mock_sensors mock_arduino
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

