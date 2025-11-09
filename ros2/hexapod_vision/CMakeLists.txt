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
find_package(image_transport REQUIRED)

if(DEFINED ENV{CONDA_PREFIX})
    set(OpenCV_DIR "$ENV{CONDA_PREFIX}/lib/cmake/opencv4")
    list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()
find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV REQUIRED)
endif()

include_directories(include)

# 添加可执行文件
add_executable(vision_node 
  src/vision_node.cpp
  src/vit_disease_detector.cpp
)
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  image_transport
)
target_link_libraries(vision_node ${OpenCV_LIBS} curl)
target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 添加目标识别节点
add_executable(object_detector src/object_detector.cpp)
ament_target_dependencies(object_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
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
  image_transport
  OpenCV
)

ament_package()

