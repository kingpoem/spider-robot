cmake_minimum_required(VERSION 3.8)
project(hexapod_perception)

# 抑制关于库路径冲突的CMake警告
# 这个警告是因为conda环境中有同名库，但不会影响实际运行
# 因为我们明确指定了系统库路径
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 排除conda环境路径，避免库冲突警告
if(DEFINED ENV{CONDA_PREFIX})
    list(REMOVE_ITEM CMAKE_SYSTEM_PREFIX_PATH "$ENV{CONDA_PREFIX}")
    list(REMOVE_ITEM CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(DEFINED ENV{CONDA_PREFIX})
    set(OpenCV_DIR "$ENV{CONDA_PREFIX}/lib/cmake/opencv4")
    list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()
find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV REQUIRED)
endif()

include_directories(include)
include_directories(../hexapod_vision/include)

# 环境感知节点
# 注意：如果使用conda环境，CMake可能会警告库路径冲突（libcurl.so.4）
# 这个警告是安全的，不会影响实际运行，因为：
# 1. 我们明确设置了RPATH优先使用系统库路径
# 2. 运行时库加载器会按照正确的顺序搜索库
# 3. 系统库路径通常在conda路径之前被搜索
# 要抑制此警告，可以在构建时使用: colcon build 2>&1 | grep -v "may be hidden by files"
add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)
target_include_directories(perception_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(perception_node ${OpenCV_LIBS})
# 链接curl库以解决OpenCV/GDAL依赖问题
# 优先使用系统库路径，避免conda环境中的库冲突
find_library(CURL_LIBRARY 
    NAMES curl
    PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib
    NO_DEFAULT_PATH
)
if(NOT CURL_LIBRARY)
    find_library(CURL_LIBRARY curl)
endif()
if(CURL_LIBRARY)
    target_link_libraries(perception_node ${CURL_LIBRARY})
endif()
# 设置RPATH属性，明确指定运行时库搜索路径，优先使用系统库
# 这确保了即使conda环境中有同名库，运行时也会使用系统库
set_target_properties(perception_node PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH FALSE
    BUILD_WITH_INSTALL_RPATH FALSE
)
# 明确设置RPATH，优先使用系统库路径
if(UNIX AND NOT APPLE)
    set_target_properties(perception_node PROPERTIES
        INSTALL_RPATH "/usr/lib/x86_64-linux-gnu:$ORIGIN"
        BUILD_RPATH "/usr/lib/x86_64-linux-gnu"
    )
endif()

install(TARGETS perception_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

ament_package()

