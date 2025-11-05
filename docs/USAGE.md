# 使用指南

## 启动系统

### 1. 启动所有节点

```bash
ros2 launch hexapod_bringup hexapod.launch.py
```

这将启动：
- SLAM节点
- 导航节点
- 任务规划节点
- 步态规划节点
- Arduino通信桥接

### 2. 单独启动节点

```bash
# SLAM节点
ros2 run hexapod_slam slam_node

# 导航节点
ros2 run hexapod_navigation navigation_node

# 任务规划节点
ros2 run hexapod_task_planning task_planner

# 步态规划节点
ros2 run hexapod_gait_planner gait_planner
```

## 基本操作

### 发送速度命令

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 设置目标位置

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 执行任务

```bash
# 采摘任务
ros2 topic pub /task/command std_msgs/msg/String "data: 'harvest'"

# 收集任务
ros2 topic pub /task/command std_msgs/msg/String "data: 'collect'"

# 监测任务
ros2 topic pub /task/command std_msgs/msg/String "data: 'monitor'"

# 灌溉任务
ros2 topic pub /task/command std_msgs/msg/String "data: 'irrigate'"
```

## 监控状态

### 查看机器人状态

```bash
ros2 topic echo /robot/status
```

### 查看地图

```bash
ros2 run rviz2 rviz2
# 添加Map显示
```

### 查看TF树

```bash
ros2 run tf2_tools view_frames
```

## 高级功能

### 调整步态参数

通过参数服务器调整：

```bash
ros2 param set /gait_planner max_linear_vel 0.5
ros2 param set /gait_planner max_step_length 0.3
```

### 自定义步态

修改 `ros2/hexapod_gait_planner/src/gait_planner.cpp` 中的步态生成逻辑。

### 地形适应性

系统会自动根据地形调整步态参数。可以通过订阅 `terrain/score` 话题查看地形评分。

## 安全注意事项

1. **紧急停止**：始终准备紧急停止机制
2. **测试环境**：在安全环境中测试
3. **电源管理**：监控电池电量
4. **传感器检查**：定期检查传感器是否正常工作

## 故障排除

### 机器人不移动
- 检查Arduino连接
- 检查伺服电机电源
- 检查紧急停止状态

### 步态不正常
- 检查IMU数据
- 检查力传感器数据
- 调整步态参数

### SLAM不工作
- 检查LiDAR/摄像头连接
- 检查传感器数据发布
- 检查地图话题

