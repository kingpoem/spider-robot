/**
 * 模拟传感器节点
 * 用于在没有硬件的情况下测试系统
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>

class MockSensorsNode : public rclcpp::Node
{
public:
    MockSensorsNode() : Node("mock_sensors")
    {
        // 发布话题
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("lidar/scan", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("depth/points", 10);
        force_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("foot/forces", 10);
        
        // 定时器
        imu_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz
            std::bind(&MockSensorsNode::publishIMU, this));
        
        lidar_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&MockSensorsNode::publishLidar, this));
        
        force_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz
            std::bind(&MockSensorsNode::publishForces, this));
        
        // 初始化
        time_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "模拟传感器节点已启动");
    }

private:
    void publishIMU()
    {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        
        // 模拟姿态数据（简单的正弦波）
        time_ += 0.01;
        
        // 模拟轻微的倾斜和摆动
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.1 * sin(time_ * 0.5);
        msg.orientation.z = 0.0;
        msg.orientation.w = sqrt(1.0 - msg.orientation.y * msg.orientation.y);
        
        // 模拟角速度
        msg.angular_velocity.x = 0.0;
        msg.angular_velocity.y = 0.05 * cos(time_ * 0.5);
        msg.angular_velocity.z = 0.0;
        
        // 模拟线性加速度
        msg.linear_acceleration.x = 0.0;
        msg.linear_acceleration.y = 0.0;
        msg.linear_acceleration.z = 9.81;
        
        imu_pub_->publish(msg);
    }
    
    void publishLidar()
    {
        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.stamp = this->now();
        msg.header.frame_id = "lidar_link";
        
        msg.angle_min = -M_PI;
        msg.angle_max = M_PI;
        msg.angle_increment = M_PI / 180.0;  // 1度分辨率
        msg.time_increment = 0.0;
        msg.scan_time = 0.1;
        msg.range_min = 0.1;
        msg.range_max = 5.0;
        
        // 生成模拟激光扫描数据
        int num_readings = (msg.angle_max - msg.angle_min) / msg.angle_increment;
        msg.ranges.resize(num_readings);
        
        for (size_t i = 0; i < num_readings; ++i) {
            float angle = msg.angle_min + i * msg.angle_increment;
            // 模拟简单的圆形障碍物
            float range = 2.0 + 0.5 * sin(angle * 3.0);
            msg.ranges[i] = std::min(range, msg.range_max);
        }
        
        msg.intensities.resize(num_readings, 1.0);
        
        lidar_pub_->publish(msg);
    }
    
    void publishForces()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        
        // 模拟6条腿的力数据
        for (int i = 0; i < 6; ++i) {
            // 模拟接触力（简单的正弦波）
            float force = 50.0 + 10.0 * sin(time_ + i * M_PI / 3.0);
            msg.data.push_back(force);
        }
        
        force_pub_->publish(msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr force_pub_;
    
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr force_timer_;
    
    double time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockSensorsNode>());
    rclcpp::shutdown();
    return 0;
}

