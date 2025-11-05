#!/bin/bash

# 系统测试脚本

echo "=== 蜘蛛型多足机器人系统测试 ==="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未配置"
    echo "请运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS2环境: $ROS_DISTRO"

# 检查工作空间
if [ ! -d "ros2" ]; then
    echo "错误: 未找到ros2目录"
    exit 1
fi

# 编译项目
echo "编译项目..."
cd ros2
colcon build

if [ $? -ne 0 ]; then
    echo "错误: 编译失败"
    exit 1
fi

echo "编译成功"

# 加载环境
source install/setup.bash

# 列出可用节点
echo ""
echo "可用节点:"
ros2 pkg list | grep hexapod

# 列出可用话题
echo ""
echo "可用话题:"
ros2 topic list

echo ""
echo "测试完成！"
echo "运行仿真: ros2 launch hexapod_simulation minimal_test.launch.py"

