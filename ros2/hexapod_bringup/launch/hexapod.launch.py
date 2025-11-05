from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # SLAM节点
        Node(
            package='hexapod_slam',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'map_update_rate': 5.0,
            }]
        ),
        
        # 导航节点
        Node(
            package='hexapod_navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'planner_frequency': 1.0,
                'controller_frequency': 10.0,
            }]
        ),
        
        # 任务规划节点
        Node(
            package='hexapod_task_planning',
            executable='task_planner',
            name='task_planner',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # 步态规划节点
        Node(
            package='hexapod_gait_planner',
            executable='gait_planner',
            name='gait_planner',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'gait_frequency': 10.0,
            }]
        ),
        
        # 通信节点（与Arduino通信）
        Node(
            package='hexapod_communication',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
            }]
        ),
    ])

