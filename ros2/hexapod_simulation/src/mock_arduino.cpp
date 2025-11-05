/**
 * 模拟Arduino节点
 * 模拟Arduino与ROS2的通信
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class MockArduinoNode : public rclcpp::Node
{
public:
    MockArduinoNode() : Node("mock_arduino")
    {
        // 订阅话题
        gait_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "gait/commands", 10,
            std::bind(&MockArduinoNode::gaitCommandCallback, this, std::placeholders::_1));
        
        // 发布话题
        status_pub_ = this->create_publisher<std_msgs::msg::String>("robot/status", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        
        // 定时器
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&MockArduinoNode::publishStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "模拟Arduino节点已启动");
    }

private:
    void gaitCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 5) {
            float linear_vel = msg->data[0];
            float angular_vel = msg->data[1];
            float step_length = msg->data[2];
            float step_height = msg->data[3];
            int gait_type = static_cast<int>(msg->data[4]);
            
            RCLCPP_INFO(this->get_logger(), 
                "收到步态命令: v=%.2f, w=%.2f, step_len=%.2f, step_h=%.2f, type=%d",
                linear_vel, angular_vel, step_length, step_height, gait_type);
        }
    }
    
    void publishStatus()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "RUNNING:OK";
        status_pub_->publish(msg);
    }
    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gait_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockArduinoNode>());
    rclcpp::shutdown();
    return 0;
}

