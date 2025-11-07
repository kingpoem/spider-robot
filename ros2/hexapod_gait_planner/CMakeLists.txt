cmake_minimum_required(VERSION 3.8)
project(hexapod_gait_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 添加源文件
add_executable(gait_planner 
    src/gait_planner.cpp
    src/dwa_planner.cpp
    src/terrain_adaptation.cpp
)
ament_target_dependencies(gait_planner
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS gait_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

