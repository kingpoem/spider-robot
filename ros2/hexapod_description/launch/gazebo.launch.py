#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='hexapod_description').find('hexapod_description')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('hexapod_description'),
            'worlds',
            'empty.world'
        ]),
        description='Gazebo world文件路径'
    )
    
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    
    # 生成URDF
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('hexapod_description'),
            'urdf',
            'hexapod.urdf.xacro'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Joint State Publisher (用于可视化)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Gazebo启动
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # 在Gazebo中生成机器人（延迟5秒，等待Gazebo启动）
    spawn_entity_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_hexapod',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'hexapod_robot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )
    
    # ros2_control控制器管理器（延迟6秒，等待机器人生成）
    controller_manager_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[
                    robot_description,
                    PathJoinSubstitution([
                        FindPackageShare('hexapod_description'),
                        'config',
                        'hexapod_controllers.yaml'
                    ])
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        world_file_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        controller_manager_node,
    ])

