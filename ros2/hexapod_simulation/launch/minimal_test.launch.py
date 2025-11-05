from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    最小测试环境
    只启动必要的节点进行测试
    """
    return LaunchDescription([
        # 模拟传感器节点
        Node(
            package='hexapod_simulation',
            executable='mock_sensors',
            name='mock_sensors',
            output='screen',
        ),
        
        # 步态规划节点
        Node(
            package='hexapod_gait_planner',
            executable='gait_planner',
            name='gait_planner',
            output='screen',
            parameters=[{
                'gait_frequency': 10.0,
                'max_linear_vel': 0.3,
                'max_angular_vel': 1.0,
            }]
        ),
    ])

