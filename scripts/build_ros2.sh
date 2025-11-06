#!/bin/bash
# ROS2构建脚本，自动设置环境并过滤无害的警告

# 设置ROS2环境
export CMAKE_PREFIX_PATH=/opt/ros/humble:$CMAKE_PREFIX_PATH
export AMENT_PREFIX_PATH=/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export PATH=/opt/ros/humble/bin:$PATH
export PKG_CONFIG_PATH=/opt/ros/humble/lib/pkgconfig:$PKG_CONFIG_PATH
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH

# 进入ros2目录
cd "$(dirname "$0")/../ros2" || exit 1

# 执行构建，过滤掉conda库路径冲突的警告（这是无害的）
colcon build "$@" 2>&1 | grep -v "may be hidden by files in:" | grep -v "Cannot generate a safe runtime search path"

# 返回colcon的退出码
exit ${PIPESTATUS[0]}

