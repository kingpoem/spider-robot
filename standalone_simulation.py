#!/usr/bin/env python3
"""
蜘蛛型机器人独立仿真程序
不依赖ROS2，可以直接运行查看机器人运动模拟
"""

import math
import time
import sys

class SpiderRobotSimulation:
    """蜘蛛机器人仿真类"""
    
    def __init__(self, num_legs=6):
        self.num_legs = num_legs
        self.leg_positions = [0.0] * num_legs  # 每条腿的相位
        self.robot_position = [0.0, 0.0]  # 机器人位置 (x, y)
        self.robot_heading = 0.0  # 机器人朝向（弧度）
        
        # 步态参数
        self.gait_frequency = 1.0  # Hz
        self.stride_length = 0.2  # 米
        self.stride_height = 0.1  # 米
        
        # 速度控制
        self.linear_velocity = 0.0  # m/s
        self.angular_velocity = 0.0  # rad/s
        
        # 传感器数据
        self.imu_data = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.foot_forces = [0.0] * num_legs
        
        print(f"✓ 初始化 {num_legs} 足蜘蛛机器人仿真")
    
    def set_velocity(self, linear, angular):
        """设置机器人速度"""
        self.linear_velocity = linear
        self.angular_velocity = angular
        print(f"✓ 设置速度: 线速度={linear:.2f} m/s, 角速度={angular:.2f} rad/s")
    
    def tripod_gait(self, phase):
        """三足步态生成"""
        # 三足步态: 腿 0,2,4 为一组, 腿 1,3,5 为另一组
        leg_phases = []
        for i in range(self.num_legs):
            if i % 2 == 0:
                leg_phases.append(phase)
            else:
                leg_phases.append((phase + math.pi) % (2 * math.pi))
        return leg_phases
    
    def calculate_foot_position(self, phase):
        """计算足端位置"""
        # 简化的足端轨迹计算
        if phase < math.pi:  # 摆动相
            x = self.stride_length * math.sin(phase)
            z = self.stride_height * math.sin(phase)
            ground_contact = False
        else:  # 支撑相
            x = self.stride_length * math.sin(phase)
            z = 0.0
            ground_contact = True
        
        return x, z, ground_contact
    
    def update_sensors(self):
        """更新传感器数据"""
        # 模拟IMU数据（小幅度晃动）
        t = time.time()
        self.imu_data['roll'] = 0.05 * math.sin(t * 2.0)
        self.imu_data['pitch'] = 0.03 * math.sin(t * 1.5)
        self.imu_data['yaw'] = self.robot_heading
        
        # 模拟足端力（支撑腿有力，摆动腿无力）
        for i in range(self.num_legs):
            _, _, ground_contact = self.calculate_foot_position(self.leg_positions[i])
            self.foot_forces[i] = 10.0 if ground_contact else 0.0
    
    def update(self, dt):
        """更新仿真状态"""
        # 更新腿部相位
        phase_increment = 2 * math.pi * self.gait_frequency * dt
        for i in range(self.num_legs):
            self.leg_positions[i] += phase_increment
            self.leg_positions[i] %= (2 * math.pi)
        
        # 更新机器人位置
        self.robot_heading += self.angular_velocity * dt
        self.robot_position[0] += self.linear_velocity * math.cos(self.robot_heading) * dt
        self.robot_position[1] += self.linear_velocity * math.sin(self.robot_heading) * dt
        
        # 更新传感器
        self.update_sensors()
    
    def visualize(self):
        """在终端显示机器人状态"""
        print("\n" + "="*60)
        print(f"🕷️  蜘蛛机器人仿真状态")
        print("="*60)
        
        # 显示位置和姿态
        print(f"位置: X={self.robot_position[0]:6.2f}m  Y={self.robot_position[1]:6.2f}m")
        print(f"朝向: {math.degrees(self.robot_heading):6.1f}°")
        print(f"速度: V={self.linear_velocity:.2f} m/s  ω={self.angular_velocity:.2f} rad/s")
        
        # 显示IMU数据
        print(f"\nIMU: Roll={math.degrees(self.imu_data['roll']):5.1f}°  "
              f"Pitch={math.degrees(self.imu_data['pitch']):5.1f}°  "
              f"Yaw={math.degrees(self.imu_data['yaw']):5.1f}°")
        
        # 显示腿部状态
        print("\n腿部状态:")
        for i in range(self.num_legs):
            x, z, contact = self.calculate_foot_position(self.leg_positions[i])
            phase_deg = math.degrees(self.leg_positions[i])
            status = "■ 支撑" if contact else "○ 摆动"
            force = self.foot_forces[i]
            print(f"  腿 {i}: {status}  相位={phase_deg:6.1f}°  "
                  f"位置=({x:5.2f}, {z:5.2f})m  力={force:4.1f}N")
        
        # ASCII艺术可视化腿的状态
        print("\n俯视图 (■=支撑 ○=摆动):")
        print("        前")
        leg_chars = []
        for i in range(self.num_legs):
            _, _, contact = self.calculate_foot_position(self.leg_positions[i])
            leg_chars.append("■" if contact else "○")
        
        if self.num_legs == 6:
            print(f"     {leg_chars[0]}     {leg_chars[1]}")
            print(f"   {leg_chars[2]}   🕷️    {leg_chars[3]}")
            print(f"     {leg_chars[4]}     {leg_chars[5]}")
        else:
            for i in range(0, self.num_legs, 2):
                if i+1 < self.num_legs:
                    print(f"     {leg_chars[i]}     {leg_chars[i+1]}")
        
        print("        后")
    
    def run_scenario(self, name, duration, linear_vel, angular_vel):
        """运行一个测试场景"""
        print(f"\n{'='*60}")
        print(f"场景: {name}")
        print(f"{'='*60}")
        
        self.set_velocity(linear_vel, angular_vel)
        
        dt = 0.1  # 时间步长
        steps = int(duration / dt)
        
        for step in range(steps):
            self.update(dt)
            
            # 每0.5秒显示一次状态
            if step % 5 == 0:
                self.visualize()
                time.sleep(0.5)  # 减慢显示速度，便于观察


def main():
    """主函数"""
    print("="*60)
    print("🕷️  蜘蛛型多足农业机器人 - 独立仿真")
    print("="*60)
    print("\n本仿真展示机器人的基本运动和传感器数据")
    print("无需安装ROS2即可运行\n")
    
    # 创建仿真实例
    robot = SpiderRobotSimulation(num_legs=6)
    
    # 测试场景
    scenarios = [
        ("直线前进", 3.0, 0.2, 0.0),
        ("原地旋转", 3.0, 0.0, 0.5),
        ("弧线运动", 3.0, 0.15, 0.3),
        ("后退运动", 2.0, -0.1, 0.0),
    ]
    
    try:
        for name, duration, linear, angular in scenarios:
            robot.run_scenario(name, duration, linear, angular)
            time.sleep(1)
        
        print("\n" + "="*60)
        print("✓ 仿真完成!")
        print("="*60)
        print(f"\n最终位置: X={robot.robot_position[0]:.2f}m, Y={robot.robot_position[1]:.2f}m")
        print(f"总行程: {math.sqrt(robot.robot_position[0]**2 + robot.robot_position[1]**2):.2f}m")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  仿真被用户中断")
        sys.exit(0)


if __name__ == "__main__":
    main()
