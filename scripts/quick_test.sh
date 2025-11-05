#!/bin/bash

# 快速测试脚本

echo "=== 快速测试 ==="

# 设置ROS2环境
source /opt/ros/humble/setup.bash 2>/dev/null || true

# 进入工作空间
cd "$(dirname "$0")/.."

# 编译
echo "编译项目..."
cd ros2
colcon build --symlink-install

# 加载环境
source install/setup.bash

# 启动最小测试环境
echo "启动测试环境..."
echo "按Ctrl+C停止"

ros2 launch hexapod_simulation minimal_test.launch.py

