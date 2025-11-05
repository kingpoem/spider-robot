# 仿真与运行指南

## 概述

本项目可以通过多种方式进行仿真和运行：
1. **Gazebo仿真** - 完整的物理仿真环境
2. **Arduino仿真** - 使用Arduino模拟器
3. **ROS2仿真** - 仅软件层面仿真
4. **实际硬件运行** - 在真实硬件上运行

## 方法一：Gazebo仿真（推荐）

### 1. 安装Gazebo和ROS2集成

```bash
# 安装Gazebo
sudo apt-get install gazebo11

# 安装ROS2-Gazebo桥接
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-gazebo-ros2-control

# 安装机器人模型依赖
sudo apt-get install ros-humble-xacro
sudo apt-get install ros-humble-joint-state-publisher
sudo apt-get install ros-humble-robot-state-publisher
```

### 2. 创建Gazebo仿真环境

我们需要创建机器人模型和世界文件。

### 3. 启动仿真

```bash
# 启动Gazebo仿真
ros2 launch hexapod_gazebo hexapod_world.launch.py
```

### 4. 运行ROS2节点

```bash
# 在另一个终端
ros2 launch hexapod_bringup hexapod.launch.py
```

## 方法二：Arduino仿真

### 1. 使用Wokwi在线仿真器

Wokwi是一个免费的Arduino在线仿真器。

1. 访问 https://wokwi.com
2. 创建新项目
3. 上传Arduino代码
4. 配置虚拟硬件

### 2. 使用Proteus仿真

Proteus是一个商业仿真软件，支持Arduino仿真。

### 3. 使用Arduino IDE的串口监视器

即使没有硬件，也可以测试代码逻辑：

```bash
# 编译代码（不上传）
# 使用串口监视器查看输出
```

## 方法三：ROS2纯软件仿真

### 1. 使用虚拟传感器

创建模拟传感器数据的ROS2节点。

### 2. 启动仿真节点

```bash
ros2 launch hexapod_simulation simulation.launch.py
```

## 方法四：实际硬件运行

### 1. 准备硬件

- 确保所有硬件连接正确
- 检查电源供应
- 验证传感器连接

### 2. 上传Arduino固件

```bash
cd arduino
# 使用Arduino IDE打开并上传
# 或使用PlatformIO
pio run -t upload
```

### 3. 配置网络

确保Arduino和ROS2机器在同一网络中。

### 4. 启动系统

```bash
# 启动ROS2节点
ros2 launch hexapod_bringup hexapod.launch.py
```

## 快速开始：最小仿真环境

### 步骤1：准备ROS2环境

```bash
# 确保ROS2已安装
source /opt/ros/humble/setup.bash

# 编译项目
cd ros2
colcon build
source install/setup.bash
```

### 步骤2：使用模拟数据

创建一个简单的测试节点来模拟传感器数据：

```bash
# 运行模拟传感器节点
ros2 run hexapod_simulation mock_sensors
```

### 步骤3：测试步态规划

```bash
# 启动步态规划节点
ros2 run hexapod_gait_planner gait_planner

# 发送测试命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 故障排除

### Arduino无法连接

```bash
# 检查串口权限
sudo chmod 666 /dev/ttyUSB0

# 添加到dialout组
sudo usermod -a -G dialout $USER
```

### ROS2节点无法启动

```bash
# 检查依赖
rosdep install --from-paths src --ignore-src -r -y

# 检查环境
echo $ROS_DISTRO
```

### Gazebo无法启动

```bash
# 检查显示设置
export GAZEBO_IP=127.0.0.1

# 检查模型路径
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/your_model_path
```

## 测试清单

- [ ] ROS2环境配置正确
- [ ] 所有节点可以编译
- [ ] 传感器数据可以发布
- [ ] 命令可以接收
- [ ] 步态规划正常工作
- [ ] SLAM节点可以运行
- [ ] 导航节点可以运行

## 下一步

1. 根据实际硬件调整参数
2. 在仿真环境中测试完整流程
3. 逐步迁移到真实硬件
4. 优化性能参数

