# 仿真启动指南

## 准备工作

### 1. 确保已构建项目

```bash
# 在项目根目录执行
cd /home/king/app/github/spider-robot/ros2
colcon build
source install/setup.bash
```

## 方式一：最小测试环境（推荐新手，2个终端）

最简单的测试方式，只启动必要的节点。

### 终端1：启动仿真环境

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
# 1. 进入ros2目录
cd /home/king/app/github/spider-robot/ros2

# 2. 加载ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. 启动最小测试环境
ros2 launch hexapod_simulation minimal_test.launch.py
```

**说明：** 这会启动：
- 模拟传感器节点（发布IMU、LiDAR、力传感器数据）
- 步态规划节点

### 终端2：测试控制命令

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
# 1. 进入ros2目录
cd /home/king/app/github/spider-robot/ros2

# 2. 加载ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. 查看可用话题
ros2 topic list

# 4. 发送速度命令（让机器人前进）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 5. 查看步态命令输出
ros2 topic echo /gait/commands

# 6. 查看传感器数据
ros2 topic echo /imu/data
ros2 topic echo /lidar/scan
```

---

## 方式二：完整仿真环境（3个终端）

包含模拟传感器、模拟Arduino、SLAM和步态规划。

### 终端1：启动完整仿真

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hexapod_simulation simulation.launch.py
```

**说明：** 这会启动：
- 模拟传感器节点
- 模拟Arduino节点
- SLAM节点
- 步态规划节点

### 终端2：发送控制命令

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 持续发送速度命令（10Hz频率）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

### 终端3：监控和调试

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看步态命令
ros2 topic echo /gait/commands

# 查看地图
ros2 topic echo /map

# 查看位姿
ros2 topic echo /pose

# 查看节点信息
ros2 node info /gait_planner
```

---

## 方式三：完整系统（4-5个终端）

启动所有模块，包括视觉、农业任务等。

### 终端1：启动完整系统

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hexapod_bringup full_system.launch.py
```

**说明：** 这会启动所有节点：
- SLAM节点
- 导航节点
- 步态规划节点
- 任务规划节点
- 视觉节点
- 目标检测节点
- 环境感知节点
- 农业任务节点
- 采摘节点
- 监测节点
- 灌溉节点

### 终端2：启动仿真传感器（可选）

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 如果full_system没有包含模拟传感器，单独启动
ros2 run hexapod_simulation mock_sensors
```

### 终端3：发送任务命令

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 发送采摘任务命令
ros2 topic pub /agriculture/task_command std_msgs/msg/String "{data: 'harvest'}"

# 发送监测任务命令
ros2 topic pub /agriculture/task_command std_msgs/msg/String "{data: 'monitor'}"

# 发送灌溉任务命令
ros2 topic pub /agriculture/task_command std_msgs/msg/String "{data: 'irrigate'}"

# 发送停止命令
ros2 topic pub /agriculture/task_command std_msgs/msg/String "{data: 'stop'}"
```

### 终端4：发送导航目标

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 发送目标位置
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 终端5：监控和调试

**路径：** `/home/king/app/github/spider-robot/ros2`

**命令：**
```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看任务状态
ros2 topic echo /agriculture/task_status

# 查看视觉检测结果
ros2 topic echo /vision/detected_objects

# 查看环境感知结果
ros2 topic echo /perception/terrain_analysis
```

---

## 快速启动脚本（最简单方式）

### 使用快速测试脚本（1个终端）

**路径：** `/home/king/app/github/spider-robot`

**命令：**
```bash
cd /home/king/app/github/spider-robot
chmod +x scripts/quick_test.sh
./scripts/quick_test.sh
```

这个脚本会自动：
1. 编译项目
2. 加载环境
3. 启动最小测试环境

然后在另一个终端执行测试命令。

---

## 常用测试命令汇总

### 查看系统状态

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /cmd_vel

# 查看话题类型
ros2 topic type /cmd_vel

# 查看节点信息
ros2 node info /gait_planner
```

### 发送控制命令

```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 后退
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# 右转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 查看数据

```bash
# 查看步态命令
ros2 topic echo /gait/commands

# 查看IMU数据
ros2 topic echo /imu/data

# 查看LiDAR数据
ros2 topic echo /lidar/scan

# 查看地图
ros2 topic echo /map

# 查看位姿
ros2 topic echo /pose
```

---

## 停止仿真

在任何启动仿真的终端中按 `Ctrl+C` 即可停止。

---

## 故障排除

### 问题1：找不到launch文件

```bash
# 确保已构建并source环境
cd /home/king/app/github/spider-robot/ros2
colcon build
source install/setup.bash

# 检查launch文件是否存在
ros2 pkg prefix hexapod_simulation
```

### 问题2：节点无法启动

```bash
# 检查节点是否存在
ros2 pkg executables hexapod_simulation

# 查看详细错误信息
ros2 run hexapod_simulation mock_sensors --ros-args --log-level debug
```

### 问题3：话题没有数据

```bash
# 检查节点是否在运行
ros2 node list

# 检查话题是否存在
ros2 topic list

# 检查话题是否有发布者
ros2 topic info /cmd_vel
```

---

## 推荐流程

**新手推荐：**
1. 先使用**方式一（最小测试环境）**，2个终端
2. 熟悉基本操作后，使用**方式二（完整仿真）**，3个终端
3. 最后使用**方式三（完整系统）**，4-5个终端

**快速测试：**
- 直接使用 `scripts/quick_test.sh` 脚本

