from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    slam_node = Node(
        package='hexapod_slam',
        executable='slam_node',
        name='slam_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    navigation_node = Node(
        package='hexapod_navigation',
        executable='navigation_node',
        name='navigation_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    gait_planner_node = Node(
        package='hexapod_gait_planner',
        executable='gait_planner',
        name='gait_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    task_planner_node = Node(
        package='hexapod_task_planning',
        executable='task_planner',
        name='task_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    vision_node = Node(
        package='hexapod_vision',
        executable='vision_node',
        name='vision_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    object_detector_node = Node(
        package='hexapod_vision',
        executable='object_detector',
        name='object_detector',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    perception_node = Node(
        package='hexapod_perception',
        executable='perception_node',
        name='perception_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    agriculture_node = Node(
        package='hexapod_agriculture',
        executable='agriculture_node',
        name='agriculture_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    monitor_node = Node(
        package='hexapod_agriculture',
        executable='monitor_node',
        name='monitor_node',
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
        monitor_node,
        LogInfo(msg='所有节点已启动')
    ])

