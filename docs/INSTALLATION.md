# 安装指南

## 硬件准备

### 必需硬件
- Arduino Portenta H7（双核ARM Cortex-M7/M4，480MHz/240MHz）
- 6-8个高扭矩伺服电机（每个腿3个自由度）
- IMU传感器（MPU6050或BNO085）
- 力传感器（每个足端）
- LiDAR或深度摄像头（每个腿）
- 电源管理系统

### 可选硬件
- 深度摄像头（Intel RealSense或类似）
- 激光雷达（RPLIDAR或类似）
- 额外传感器（温度、湿度等）
- 能量回收模块（用于足端能量回收）

## 软件安装

### 1. Arduino IDE设置

1. 下载并安装 [Arduino IDE](https://www.arduino.cc/en/software) 或 PlatformIO
2. 安装Arduino Portenta H7开发板支持
3. 安装必要的库：
   - Servo库（内置）
   - Wire库（内置）
   - WiFi库（Portenta H7内置支持）

### 2. ROS2安装

#### Ubuntu 22.04 (ROS2 Humble)

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop
```

#### 配置ROS2环境

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 安装依赖

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
```

### 4. 编译项目

```bash
cd ros2
colcon build
source install/setup.bash
```

## 配置

### Arduino配置

1. 打开 `arduino/core/hexapod_main.ino`
2. 根据硬件配置修改 `arduino/core/config.h` 中的引脚定义
3. 配置WiFi信息（在 `ros2_interface.h` 中）：
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
4. 注意：Portenta H7使用双核架构，步态计算在M7核心本地完成

### ROS2配置

1. 配置ROS2节点参数（在各自的launch文件中）
2. 配置TF树（如果需要）
3. 配置地图参数

## 测试

### 测试Arduino固件

```bash
cd arduino
# 使用Arduino IDE上传固件
# 或使用PlatformIO
pio run -t upload
```

### 测试ROS2节点

```bash
# 启动所有节点
ros2 launch hexapod_bringup hexapod.launch.py

# 测试单个节点
ros2 run hexapod_slam slam_node
```

## 故障排除

### Arduino无法连接
- 检查串口权限：`sudo chmod 666 /dev/ttyUSB0`
- 检查USB连接
- 检查Arduino驱动

### ROS2节点无法启动
- 检查ROS2环境：`echo $ROS_DISTRO`
- 检查依赖：`rosdep install --from-paths src --ignore-src -r -y`
- 查看日志：`ros2 run <package> <node> --ros-args --log-level debug`

### WiFi连接失败
- 检查SSID和密码
- 检查网络信号强度
- 检查防火墙设置
