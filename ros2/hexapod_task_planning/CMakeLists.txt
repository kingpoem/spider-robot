cmake_minimum_required(VERSION 3.8)
project(hexapod_task_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(task_planner src/task_planner.cpp)
ament_target_dependencies(task_planner
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS task_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

