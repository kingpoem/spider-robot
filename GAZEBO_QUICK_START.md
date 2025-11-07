# Gazebo仿真快速启动指南

## 第一步：安装Gazebo和依赖

运行安装脚本（需要sudo权限）：

```bash
cd /home/king/app/github/spider-robot
chmod +x install_gazebo.sh
sudo bash install_gazebo.sh
```

或者手动安装：

```bash
sudo apt update
sudo apt install -y gazebo
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager
```

## 第二步：编译项目

```bash
cd /home/king/app/github/spider-robot/ros2
colcon build
source install/setup.bash
```

## 第三步：启动Gazebo仿真

### 方法1：使用启动脚本（最简单）

```bash
cd /home/king/app/github/spider-robot
./scripts/start_gazebo.sh
```

### 方法2：使用launch文件

```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hexapod_description gazebo.launch.py
```

## 第四步：测试机器人

在另一个终端中：

```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 查看关节状态
ros2 topic echo /joint_states

# 查看所有话题
ros2 topic list

# 查看TF树
ros2 run tf2_tools view_frames
```

## 控制机器人关节

```bash
# 加载控制器
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active

ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_trajectory_controller active

# 发送关节命令（示例：移动第一条腿）
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['coxa_1_joint', 'femur_1_joint', 'tibia_1_joint'], \
   points: [{positions: [0.0, 0.5, -0.5], time_from_start: {sec: 1, nanosec: 0}}]}"
```

## 与ROS2节点集成

启动Gazebo后，在另一个终端启动ROS2节点：

```bash
cd /home/king/app/github/spider-robot/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动步态规划节点
ros2 launch hexapod_simulation simulation.launch.py
```

## 故障排除

如果遇到问题，请查看详细文档：
- [Gazebo启动指南](docs/GAZEBO_START_GUIDE.md)
- [仿真启动指南](docs/SIMULATION_START_GUIDE.md)

## 下一步

1. 调整机器人模型参数
2. 添加传感器
3. 创建自定义Gazebo world
4. 集成步态规划算法

