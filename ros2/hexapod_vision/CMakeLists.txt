cmake_minimum_required(VERSION 3.8)
project(hexapod_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# 包含目录
include_directories(include)

# 添加可执行文件
add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(vision_node ${OpenCV_LIBS} curl)

# 添加目标识别节点
add_executable(object_detector src/object_detector.cpp)
ament_target_dependencies(object_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(object_detector ${OpenCV_LIBS} curl)

# 安装
install(TARGETS vision_node object_detector
  DESTINATION lib/${PROJECT_NAME})

# 导出
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
)

ament_package()

