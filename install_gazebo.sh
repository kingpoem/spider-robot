#!/bin/bash

echo "===================================================================="
echo "蜘蛛机器人 Gazebo 仿真环境安装脚本"
echo "===================================================================="
echo ""

set -e  # Exit on error

# Check if running on Ubuntu
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$ID" != "ubuntu" ]; then
        echo "⚠️  警告: 此脚本针对 Ubuntu 设计，其他发行版可能需要调整"
    fi
fi

echo "📦 更新系统包..."
sudo apt update

echo ""
echo "📦 安装 ROS2 Humble..."
if ! command -v ros2 &> /dev/null; then
    # Add ROS2 repository
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y
    
    # Add ROS2 GPG key
    sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Add repository to sources list
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Update and install ROS2
    sudo apt update
    sudo apt install -y ros-humble-desktop
    sudo apt install -y ros-dev-tools
    
    echo "✓ ROS2 Humble 安装完成"
else
    echo "✓ ROS2 已安装"
fi

echo ""
echo "📦 安装 Gazebo Classic..."
if ! command -v gazebo &> /dev/null; then
    sudo apt install -y ros-humble-gazebo-ros-pkgs
    sudo apt install -y gazebo
    echo "✓ Gazebo 安装完成"
else
    echo "✓ Gazebo 已安装"
fi

echo ""
echo "📦 安装其他依赖..."
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    python3-colcon-common-extensions \
    python3-rosdep

echo ""
echo "📦 初始化 rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo ""
echo "===================================================================="
echo "✓ 安装完成！"
echo "===================================================================="
echo ""
echo "请在新终端或运行以下命令来加载 ROS2 环境:"
echo "    source /opt/ros/humble/setup.bash"
echo ""
echo "要将其添加到您的 shell 配置中:"
echo "    echo 'source /opt/ros/humble/setup.bash' >> ~/.zshrc"
echo ""
echo "编译项目:"
echo "    cd ~/app/github/spider-robot/ros2"
echo "    colcon build"
echo "    source install/setup.bash"
echo ""
echo "启动 Gazebo 仿真:"
echo "    ros2 launch hexapod_description gazebo.launch.py"
echo ""
