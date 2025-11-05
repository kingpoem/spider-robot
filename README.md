# 蜘蛛型多足农业机器人项目

## 项目概述

本项目是一个基于Arduino和ROS2的蜘蛛型多足机器人系统，设计用于在复杂丘陵雨林地形中执行农作物采摘、收集、监测和灌溉任务。

## 核心特性

- **多足结构**: 6-8条可独立驱动的柔性腿（伺服电机 + 行星减速）
- **被动柔性关节**: 采用弹性材料（PEEK、碳纤维-弹性体混合）或弹簧-阻尼组合
- **主动姿态控制**: IMU + 足端力/位置传感器闭环控制
- **可调步幅/步高**: 步幅 10-30 cm、步高 5-20 cm
- **足端多功能爪**: 软体橡胶爪 + 可伸缩钉
- **分布式感知**: 每条腿装配小型 LiDAR/深度摄像头 + 接触传感器
- **步态规划算法**: 基于动态窗口法（DWA）+ 采样步态库
- **能量回收**: 踏板式发电或弹性元件储能
- **模块化设计**: 快速卡扣式连接，支持更换不同配置

## 系统架构

```
┌─────────────────┐
│   ROS2 节点层   │  (SLAM, 导航, 任务规划)
└────────┬────────┘
         │
┌────────▼────────┐
│  Arduino 核心   │  (实时控制, 传感器读取)
└────────┬────────┘
         │
┌────────▼────────┐
│   FPGA 协处理器 │  (高频率步态计算, 传感器融合)
└────────┬────────┘
         │
┌────────▼────────┐
│   硬件驱动层    │  (伺服电机, 传感器, 执行器)
└─────────────────┘
```

## 目录结构

```
projetct1/
├── arduino/                    # Arduino固件代码
│   ├── core/                   # 核心控制逻辑
│   │   ├── hexapod_main.ino    # 主程序
│   │   ├── config.h            # 配置定义
│   │   ├── fpga_interface.*     # FPGA接口
│   │   └── ros2_interface.*    # ROS2接口
│   ├── drivers/                # 硬件驱动
│   │   └── leg_controller.*    # 腿部控制器
│   ├── sensors/                # 传感器接口
│   │   ├── imu_sensor.*        # IMU传感器
│   │   └── force_sensor.*      # 力传感器
│   └── gait/                   # 基础步态控制
│       └── gait_engine.*       # 步态引擎
├── ros2/                       # ROS2节点
│   ├── hexapod_bringup/        # 启动包
│   ├── hexapod_slam/           # SLAM节点
│   ├── hexapod_navigation/     # 导航节点
│   ├── hexapod_task_planning/  # 任务规划节点
│   ├── hexapod_gait_planner/   # 步态规划节点
│   │   ├── src/
│   │   │   ├── gait_planner.cpp    # 主节点
│   │   │   ├── dwa_planner.*       # DWA算法
│   │   │   └── terrain_adaptation.* # 地形适应性
│   │   └── CMakeLists.txt
│   ├── hexapod_vision/          # 视觉识别模块
│   │   ├── src/
│   │   │   ├── vision_node.cpp      # 视觉处理节点
│   │   │   └── object_detector.cpp  # 目标检测节点
│   ├── hexapod_agriculture/     # 农业任务模块
│   │   ├── src/
│   │   │   ├── agriculture_node.cpp  # 农业任务协调节点
│   │   │   ├── harvest_node.cpp      # 采摘任务节点
│   │   │   ├── monitor_node.cpp      # 监测任务节点
│   │   │   └── irrigate_node.cpp     # 灌溉任务节点
│   └── hexapod_perception/      # 环境感知模块
│       └── src/
│           └── perception_node.cpp   # 环境感知节点
├── fpga/                        # FPGA代码
│   ├── gait_calculator.v        # 步态计算模块
│   └── README.md                # FPGA说明文档
├── config/                      # 配置文件
│   ├── arduino_config.yaml
│   └── ros2_config.yaml
├── docs/                        # 文档
│   ├── INSTALLATION.md         # 安装指南
│   ├── USAGE.md                # 使用指南
│   └── ARCHITECTURE.md         # 架构文档
└── README.md                    # 项目说明
```

## 硬件要求

- **主控制器**: Arduino Mega 2560 或 Arduino Due
- **协处理器**: FPGA开发板（推荐Xilinx或Altera）
- **伺服电机**: 6-8个高扭矩伺服电机（每个腿3个自由度）
- **传感器**: 
  - IMU (MPU6050/BNO085)
  - 力传感器 (每个足端)
  - LiDAR/深度摄像头 (每个腿)
  - 接触传感器
- **通信**: 
  - WiFi模块 (ESP32) 用于ROS2通信
  - SPI/I2C 用于FPGA通信

## 软件依赖

### Arduino
- Arduino IDE 1.8+ 或 PlatformIO
- 库: Servo, Wire, SPI, WiFi, ESP32

### ROS2
- ROS2 Humble 或 ROS2 Foxy
- 依赖包:
  - nav2
  - slam_toolbox
  - sensor_msgs
  - geometry_msgs
  - std_msgs

## 快速开始

### 方法一：快速仿真测试（推荐新手）

```bash
# 1. 编译项目
cd ros2
colcon build
source install/setup.bash

# 2. 启动仿真环境
ros2 launch hexapod_simulation minimal_test.launch.py

# 3. 在另一个终端测试
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

详细步骤请参考 [快速开始指南](docs/QUICK_START.md)

### 方法二：完整系统运行

#### 1. 安装依赖

参考 [安装指南](docs/INSTALLATION.md)

#### 2. 配置硬件（实际运行需要）

根据你的硬件配置修改：
- `arduino/core/config.h` - Arduino引脚定义
- `arduino/core/ros2_interface.cpp` - WiFi配置

#### 3. 上传Arduino固件（实际运行需要）

```bash
cd arduino
# 使用Arduino IDE打开 core/hexapod_main.ino
# 或使用PlatformIO
pio run -t upload
```

#### 4. 编译ROS2节点

```bash
cd ros2
colcon build
source install/setup.bash
```

#### 5. 启动系统

**仿真模式：**
```bash
ros2 launch hexapod_simulation simulation.launch.py
```

**实际硬件：**
```bash
ros2 launch hexapod_bringup hexapod.launch.py
```

### 使用测试脚本

```bash
# 运行系统测试
chmod +x scripts/test_system.sh
./scripts/test_system.sh

# 快速测试
chmod +x scripts/quick_test.sh
./scripts/quick_test.sh
```

## 使用说明

详细的使用说明请参考：
- [快速开始](docs/QUICK_START.md) - 5分钟快速测试
- [运行指南](docs/RUNNING.md) - 详细的运行步骤
- [仿真指南](docs/SIMULATION.md) - 仿真环境设置
- [使用指南](docs/USAGE.md) - 功能使用说明
- [系统架构](docs/ARCHITECTURE.md) - 系统设计文档
- [安装指南](docs/INSTALLATION.md) - 安装步骤
