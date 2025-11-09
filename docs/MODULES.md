# 模块说明文档

本文档详细说明了项目中各个模块的功能和实现。

## 新增模块概览

本次完善为项目添加了以下核心模块：

1. **视觉识别模块** (`hexapod_vision`)
2. **农业任务模块** (`hexapod_agriculture`)
3. **环境感知模块** (`hexapod_perception`)

## 1. 视觉识别模块 (hexapod_vision)

### 功能概述
提供图像处理和目标识别功能，支持农作物识别、障碍物检测等。

### 节点说明

#### vision_node
- **功能**: 基础图像处理
  - 图像预处理（去噪、对比度增强）
  - 边缘检测
  - 颜色分割（识别绿色植物）
- **订阅话题**:
  - `camera/image_raw` - 原始相机图像
- **发布话题**:
  - `vision/processed_image` - 处理后的图像
  - `vision/detections` - 检测结果

#### object_detector
- **功能**: 目标检测
  - 农作物检测（绿色植物、成熟果实）
  - 障碍物检测
  - 支持深度学习模型（可选）
- **订阅话题**:
  - `camera/image_raw` - 原始相机图像
- **发布话题**:
  - `vision/detected_objects` - 检测到的物体信息
  - `vision/detection_result` - 带标注的检测图像

### 使用方法

```bash
# 启动视觉节点
ros2 run hexapod_vision vision_node

# 启动目标检测节点
ros2 run hexapod_vision object_detector
```

## 2. 农业任务模块 (hexapod_agriculture)

### 功能概述
实现农业机器人的核心任务：采摘、收集、监测、灌溉。

### 节点说明

#### agriculture_node
- **功能**: 农业任务协调中心
  - 接收任务命令
  - 协调各个子任务节点
  - 与导航、视觉模块交互
- **订阅话题**:
  - `agriculture/task_command` - 任务命令（harvest/collect/monitor/irrigate）
  - `agriculture/target_pose` - 目标位置
  - `vision/detected_objects` - 视觉检测结果
- **发布话题**:
  - `cmd_vel` - 移动命令
  - `goal_pose` - 目标位置（发送给导航）
  - `agriculture/task_status` - 任务状态
  - `actuator/command` - 执行器控制命令

#### harvest_node
- **功能**: 采摘任务执行
  - 控制机械臂接近作物
  - 力传感器反馈控制
  - 执行采摘动作
- **订阅话题**:
  - `harvest/crop_pose` - 作物位置
  - `sensors/gripper_force` - 夹爪力传感器
- **发布话题**:
  - `harvest/command` - 采摘命令
  - `harvest/status` - 采摘状态

#### monitor_node
- **功能**: 监测任务执行
  - 移动到监测点
  - 采集图像数据
  - 分析作物生长状态
- **订阅话题**:
  - `monitor/point` - 监测点位置
  - `camera/image_raw` - 相机图像
- **发布话题**:
  - `monitor/data` - 监测数据
  - `monitor/status` - 监测状态

#### irrigate_node
- **功能**: 灌溉任务执行
  - 控制水流系统
  - 精确灌溉控制
  - 灌溉时长管理
- **订阅话题**:
  - `irrigate/point` - 灌溉点位置
- **发布话题**:
  - `irrigate/command` - 灌溉命令
  - `irrigate/water_flow` - 水流控制
  - `irrigate/status` - 灌溉状态

### 使用方法

```bash
# 启动农业任务协调节点
ros2 run hexapod_agriculture agriculture_node

# 启动采摘节点
ros2 run hexapod_agriculture harvest_node

# 启动监测节点
ros2 run hexapod_agriculture monitor_node

# 启动灌溉节点
ros2 run hexapod_agriculture irrigate_node
```

## 3. 环境感知模块 (hexapod_perception)

### 功能概述
结合视觉和SLAM数据进行环境理解，提供地形分析、障碍物检测等功能。

### 节点说明

#### perception_node
- **功能**: 环境感知和地形分析
  - 结合SLAM地图和视觉数据进行环境理解
  - 地形特征分析（坡度、粗糙度）
  - 障碍物检测和分类
  - 路径可达性评估
- **订阅话题**:
  - `map` - SLAM地图
  - `camera/image_raw` - 相机图像
  - `lidar/scan` - LiDAR数据
  - `pose` - 机器人位姿
- **发布话题**:
  - `perception/terrain_analysis` - 地形分析结果
  - `perception/obstacles` - 障碍物信息
  - `perception/features` - 环境特征

### 使用方法

```bash
# 启动环境感知节点
ros2 run hexapod_perception perception_node
```

## 系统集成

### 完整系统启动

使用提供的启动文件启动所有模块：

```bash
# 启动完整系统
ros2 launch hexapod_bringup full_system.launch.py
```

该启动文件会启动：
- SLAM节点
- 导航节点
- 步态规划节点
- 任务规划节点
- 视觉节点
- 目标检测节点
- 环境感知节点
- 农业任务节点
- 采摘、监测、灌溉节点

## 数据流

### 感知数据流
```
相机/LiDAR → 视觉节点 → 目标检测 → 农业任务节点
                ↓
           环境感知节点 → 地形分析 → 导航节点
```

### 控制数据流
```
任务规划 → 农业任务 → 导航节点 → 步态规划 → Arduino (Portenta H7) → 执行器
```

## 配置参数

各模块支持通过ROS2参数服务器进行配置，主要参数包括：

- **视觉模块**: 相机话题、检测阈值、模型路径
- **农业模块**: 采摘力阈值、灌溉水流速率、灌溉时长
- **感知模块**: 地图话题、分析频率

## 扩展建议

1. **深度学习集成**: 在目标检测节点中集成YOLO、SSD等模型
2. **传感器融合**: 增强环境感知模块的多传感器融合算法
3. **任务规划优化**: 实现更智能的任务调度和路径优化
4. **ViT模型优化**: 训练和优化ViT病害识别模型，提高识别准确率

## 故障排除

### 常见问题

1. **视觉节点无法接收图像**
   - 检查相机话题是否正确
   - 确认相机驱动已启动

2. **农业任务节点无响应**
   - 检查任务命令话题
   - 确认导航节点已启动

## 参考文献

- ROS2官方文档: https://docs.ros.org/
- OpenCV文档: https://docs.opencv.org/
- Arduino Portenta H7文档: https://docs.arduino.cc/hardware/portenta-h7

