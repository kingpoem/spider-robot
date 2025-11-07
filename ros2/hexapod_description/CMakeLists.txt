cmake_minimum_required(VERSION 3.8)
project(hexapod_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装URDF文件
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装world文件
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# 安装mesh文件（如果有）
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# 安装config文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

