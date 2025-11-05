# 运行指南

## 运行方式概览

本项目支持多种运行方式：

1. **纯软件仿真** - 无需硬件，使用模拟数据
2. **Gazebo仿真** - 完整的物理仿真环境（需要安装Gazebo）
3. **实际硬件运行** - 在真实硬件上运行

## 方式一：纯软件仿真（最简单）

### 步骤1：编译项目

```bash
# 进入ROS2工作空间
cd ros2

# 编译
colcon build

# 加载环境
source install/setup.bash
```

### 步骤2：启动最小测试环境

```bash
ros2 launch hexapod_simulation minimal_test.launch.py
```

这将启动：
- 模拟传感器节点（发布IMU、LiDAR、力传感器数据）
- 步态规划节点

### 步骤3：测试功能

在另一个终端：

```bash
# 加载环境
source /opt/ros/humble/setup.bash
cd ros2
source install/setup.bash

# 查看可用话题
ros2 topic list

# 发送速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 查看步态命令输出
ros2 topic echo /gait/commands

# 查看传感器数据
ros2 topic echo /imu/data
ros2 topic echo /lidar/scan
```

### 步骤4：启动完整仿真

```bash
ros2 launch hexapod_simulation simulation.launch.py
```

这将启动所有节点：
- 模拟传感器节点
- 模拟Arduino节点
- SLAM节点
- 步态规划节点

## 方式二：Gazebo仿真

### 安装Gazebo

```bash
sudo apt-get install gazebo11
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

### 创建机器人模型

需要创建URDF模型文件（当前版本中未包含，需要后续添加）。

### 启动Gazebo仿真

```bash
# 启动Gazebo环境
gazebo

# 在另一个终端启动ROS2节点
ros2 launch hexapod_simulation simulation.launch.py
```

## 方式三：实际硬件运行

### 步骤1：准备硬件

1. 连接所有硬件
2. 检查电源供应
3. 验证传感器连接

### 步骤2：配置Arduino

1. 修改 `arduino/core/config.h` 中的引脚定义
2. 修改 `arduino/core/ros2_interface.cpp` 中的WiFi配置

### 步骤3：上传Arduino固件

```bash
cd arduino
# 使用Arduino IDE打开 core/hexapod_main.ino
# 点击上传按钮
```

或使用PlatformIO：

```bash
cd arduino
pio run -t upload
```

### 步骤4：配置网络

确保Arduino和ROS2机器在同一WiFi网络中。

### 步骤5：启动ROS2节点

```bash
cd ros2
source install/setup.bash
ros2 launch hexapod_bringup hexapod.launch.py
```

## 使用测试脚本

### 系统测试脚本

```bash
# 运行系统测试
chmod +x scripts/test_system.sh
./scripts/test_system.sh
```

这个脚本会：
- 检查ROS2环境
- 编译项目
- 列出可用节点和话题

### 快速测试脚本

```bash
# 快速启动测试
chmod +x scripts/quick_test.sh
./scripts/quick_test.sh
```

这个脚本会：
- 自动编译项目
- 启动最小测试环境

## 验证系统运行

### 检查节点状态

```bash
# 列出运行中的节点
ros2 node list

# 查看节点信息
ros2 node info /node_name
```

### 检查话题数据

```bash
# 列出所有话题
ros2 topic list

# 查看话题类型
ros2 topic type /topic_name

# 查看话题数据
ros2 topic echo /topic_name

# 查看话题频率
ros2 topic hz /topic_name
```

### 测试通信

```bash
# 测试速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.1}}"

# 测试任务命令
ros2 topic pub /task/command std_msgs/msg/String "data: 'harvest'"

# 测试目标位置
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}"
```

## 常见问题

### Q: 节点无法启动？

1. 检查ROS2环境：
```bash
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash
```

2. 检查编译：
```bash
cd ros2
colcon build
source install/setup.bash
```

3. 检查依赖：
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Q: 话题没有数据？

1. 检查节点是否运行：
```bash
ros2 node list
```

2. 检查话题是否存在：
```bash
ros2 topic list
```

3. 检查话题类型：
```bash
ros2 topic type /topic_name
ros2 topic info /topic_name
```

### Q: Arduino无法连接？

1. 检查串口权限：
```bash
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER
```

2. 检查WiFi连接：
```bash
# 在Arduino代码中检查WiFi状态
# 查看串口输出
```

### Q: 编译错误？

1. 安装缺失依赖：
```bash
rosdep install --from-paths src --ignore-src -r -y
```

2. 清理重建：
```bash
cd ros2
rm -rf build install log
colcon build
```

## 下一步

1. 阅读 [使用指南](USAGE.md) 了解详细功能
2. 查看 [系统架构](ARCHITECTURE.md) 了解系统设计
3. 根据实际需求调整参数
4. 在实际环境中测试

