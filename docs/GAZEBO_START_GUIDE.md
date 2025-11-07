# Gazebo仿真启动指南

本指南将帮助您在Gazebo中启动和运行蜘蛛机器人仿真。

## 前置要求

### 1. 安装Gazebo和依赖

运行安装脚本（需要sudo权限）：

```bash
cd /home/king/app/github/spider-robot
chmod +x install_gazebo.sh
sudo bash install_gazebo.sh
```

或者手动安装：

```bash
# 安装Gazebo Classic
sudo apt update
sudo apt install -y gazebo
sudo apt install -y ros-humble-gazebo-ros-pkgs

# 安装其他依赖
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    python3-colcon-common-extensions
```

### 2. 编译项目

```bash
cd /home/king/app/github/spider-robot/ros2
colcon build
source install/setup.bash
```

## 启动Gazebo仿真

### 方式一：使用launch文件（推荐）

**在终端1：启动Gazebo仿真**

```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hexapod_description gazebo.launch.py
```

这将启动：
- Gazebo仿真环境
- 机器人模型
- Robot State Publisher
- Joint State Publisher
- ros2_control控制器管理器

### 方式二：手动启动（用于调试）

**终端1：启动Gazebo**

```bash
gazebo --verbose
```

**终端2：启动Robot State Publisher**

```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 发布机器人描述
ros2 run robot_state_publisher robot_state_publisher \
  $(ros2 pkg prefix hexapod_description)/share/hexapod_description/urdf/hexapod.urdf.xacro

# 或者使用xacro
ros2 run xacro xacro $(ros2 pkg prefix hexapod_description)/share/hexapod_description/urdf/hexapod.urdf.xacro > /tmp/hexapod.urdf
ros2 run robot_state_publisher robot_state_publisher /tmp/hexapod.urdf
```

**终端3：在Gazebo中生成机器人**

```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity hexapod_robot \
  -x 0.0 -y 0.0 -z 0.5
```

## 控制机器人

### 查看机器人状态

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看关节状态
ros2 topic echo /joint_states

# 查看TF树
ros2 run tf2_tools view_frames
```

### 控制关节

**启动控制器（如果需要）：**

```bash
# 加载关节状态广播器
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active

# 加载关节轨迹控制器
ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_trajectory_controller active
```

**发送关节命令：**

```bash
# 使用joint_trajectory_controller发送轨迹
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['coxa_1_joint', 'femur_1_joint', 'tibia_1_joint'], \
   points: [{positions: [0.0, 0.5, -0.5], time_from_start: {sec: 1, nanosec: 0}}]}"
```

### 使用RViz可视化

```bash
# 启动RViz
rviz2

# 在RViz中：
# 1. 添加RobotModel显示
# 2. 设置Fixed Frame为 "base_link"
# 3. 添加TF显示
```

## 与ROS2节点集成

### 启动完整系统

**终端1：Gazebo仿真**

```bash
ros2 launch hexapod_description gazebo.launch.py
```

**终端2：启动ROS2节点**

```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动步态规划节点
ros2 launch hexapod_simulation simulation.launch.py
```

**终端3：发送控制命令**

```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 发送速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 故障排除

### 问题1：Gazebo无法启动

```bash
# 检查Gazebo是否安装
gazebo --version

# 检查ROS2环境
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash
```

### 问题2：机器人模型无法加载

```bash
# 检查URDF文件是否存在
ros2 pkg prefix hexapod_description
ls $(ros2 pkg prefix hexapod_description)/share/hexapod_description/urdf/

# 验证URDF文件语法
check_urdf $(ros2 pkg prefix hexapod_description)/share/hexapod_description/urdf/hexapod.urdf.xacro
```

### 问题3：关节无法控制

```bash
# 检查控制器是否加载
ros2 control list_controllers

# 检查ros2_control配置
ros2 param list /controller_manager
```

### 问题4：机器人掉落或穿透地面

```bash
# 检查机器人初始位置（z坐标应该足够高）
# 在launch文件中调整spawn_entity的z参数
```

## 下一步

1. 调整机器人模型参数（在URDF文件中）
2. 添加传感器（IMU、LiDAR等）
3. 创建自定义Gazebo world
4. 集成步态规划算法
5. 添加物理属性（摩擦系数、质量等）

## 参考资源

- [Gazebo官方文档](http://gazebosim.org/)
- [ROS2 Control文档](https://control.ros.org/)
- [URDF/Xacro文档](http://wiki.ros.org/urdf)
