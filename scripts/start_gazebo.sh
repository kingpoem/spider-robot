#!/bin/bash

# Gazebo仿真启动脚本

echo "=========================================="
echo "启动Gazebo仿真环境"
echo "=========================================="

# 设置脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# 进入ROS2工作空间
cd "$PROJECT_ROOT/ros2" || exit 1

# 加载ROS2环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "错误: 未找到ROS2 Humble安装"
    echo "请先安装ROS2: sudo apt install ros-humble-desktop"
    exit 1
fi

# 加载工作空间
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "警告: 工作空间未构建，正在编译..."
    colcon build
    source install/setup.bash
fi

# 检查Gazebo是否安装
if ! command -v gazebo &> /dev/null; then
    echo "错误: Gazebo未安装"
    echo "请运行: sudo bash $PROJECT_ROOT/install_gazebo.sh"
    exit 1
fi

# 启动Gazebo仿真
echo "启动Gazebo仿真..."
ros2 launch hexapod_description gazebo.launch.py

