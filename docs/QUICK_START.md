# 快速开始指南

## 5分钟快速测试

### 步骤1：安装依赖

```bash
# 安装ROS2（如果未安装）
sudo apt update
sudo apt install ros-humble-desktop

# 安装编译工具
sudo apt install python3-colcon-common-extensions
```

### 步骤2：编译项目

```bash
# 进入ROS2工作空间
cd ros2

# 编译
colcon build

# 加载环境
source install/setup.bash
```

### 步骤3：运行仿真

```bash
# 启动最小测试环境
ros2 launch hexapod_simulation minimal_test.launch.py
```

### 步骤4：测试功能

在另一个终端：

```bash
# 加载环境
source /opt/ros/humble/setup.bash
cd ros2
source install/setup.bash

# 查看可用话题
ros2 topic list

# 发送测试命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 查看步态命令
ros2 topic echo /gait/commands

# 查看机器人状态
ros2 topic echo /robot/status
```

## 完整系统测试

### 启动完整仿真

```bash
ros2 launch hexapod_simulation simulation.launch.py
```

这将启动：
- 模拟传感器节点
- 模拟Arduino节点
- SLAM节点
- 步态规划节点

### 测试各个功能

#### 测试传感器数据

```bash
# 查看IMU数据
ros2 topic echo /imu/data

# 查看激光扫描
ros2 topic echo /lidar/scan

# 查看足端力
ros2 topic echo /foot/forces
```

#### 测试步态规划

```bash
# 发送速度命令
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"

# 查看步态命令
ros2 topic echo /gait/commands
```

#### 测试SLAM

```bash
# 查看地图
ros2 topic echo /map

# 查看位姿
ros2 topic echo /pose
```

## 使用测试脚本

### 运行系统测试

```bash
chmod +x scripts/test_system.sh
./scripts/test_system.sh
```

### 运行快速测试

```bash
chmod +x scripts/quick_test.sh
./scripts/quick_test.sh
```

## 常见问题

### Q: 编译失败怎么办？

```bash
# 安装缺失的依赖
rosdep install --from-paths src --ignore-src -r -y

# 重新编译
colcon build
```

### Q: 节点无法启动？

```bash
# 检查环境变量
echo $ROS_DISTRO

# 检查节点是否存在
ros2 pkg list | grep hexapod

# 查看详细错误
ros2 run <package> <node> --ros-args --log-level debug
```

### Q: 话题没有数据？

```bash
# 检查节点是否在运行
ros2 node list

# 检查话题列表
ros2 topic list

# 检查话题频率
ros2 topic hz /topic_name
```

## 下一步

1. 阅读 [使用指南](USAGE.md)
2. 查看 [系统架构](ARCHITECTURE.md)
3. 配置硬件连接
4. 在实际硬件上测试

