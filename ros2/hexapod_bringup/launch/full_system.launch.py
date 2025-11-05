"""
完整系统启动文件
启动所有模块：SLAM、导航、视觉、农业任务、环境感知等
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # 获取配置
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # SLAM节点
    slam_node = Node(
        package='hexapod_slam',
        executable='slam_node',
        name='slam_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 导航节点
    navigation_node = Node(
        package='hexapod_navigation',
        executable='navigation_node',
        name='navigation_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 步态规划节点
    gait_planner_node = Node(
        package='hexapod_gait_planner',
        executable='gait_planner',
        name='gait_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 任务规划节点
    task_planner_node = Node(
        package='hexapod_task_planning',
        executable='task_planner',
        name='task_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 视觉节点
    vision_node = Node(
        package='hexapod_vision',
        executable='vision_node',
        name='vision_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 目标检测节点
    object_detector_node = Node(
        package='hexapod_vision',
        executable='object_detector',
        name='object_detector',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 环境感知节点
    perception_node = Node(
        package='hexapod_perception',
        executable='perception_node',
        name='perception_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 农业任务节点
    agriculture_node = Node(
        package='hexapod_agriculture',
        executable='agriculture_node',
        name='agriculture_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 采摘节点
    harvest_node = Node(
        package='hexapod_agriculture',
        executable='harvest_node',
        name='harvest_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 监测节点
    monitor_node = Node(
        package='hexapod_agriculture',
        executable='monitor_node',
        name='monitor_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 灌溉节点
    irrigate_node = Node(
        package='hexapod_agriculture',
        executable='irrigate_node',
        name='irrigate_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        LogInfo(msg='启动完整系统...'),
        slam_node,
        navigation_node,
        gait_planner_node,
        task_planner_node,
        vision_node,
        object_detector_node,
        perception_node,
        agriculture_node,
        harvest_node,
        monitor_node,
        irrigate_node,
        LogInfo(msg='所有节点已启动')
    ])

