from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 模拟传感器节点
        Node(
            package='hexapod_simulation',
            executable='mock_sensors',
            name='mock_sensors',
            output='screen',
        ),
        
        # 模拟Arduino节点
        Node(
            package='hexapod_simulation',
            executable='mock_arduino',
            name='mock_arduino',
            output='screen',
        ),
        
        # SLAM节点
        Node(
            package='hexapod_slam',
            executable='slam_node',
            name='slam_node',
            output='screen',
        ),
        
        # 步态规划节点
        Node(
            package='hexapod_gait_planner',
            executable='gait_planner',
            name='gait_planner',
            output='screen',
        ),
    ])

