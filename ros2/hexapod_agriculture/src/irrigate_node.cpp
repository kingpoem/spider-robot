/**
 * 灌溉任务节点
 * 控制灌溉系统进行精确灌溉
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>

class IrrigateNode : public rclcpp::Node
{
public:
    IrrigateNode() : Node("irrigate_node")
    {
        // 声明参数
        this->declare_parameter("water_flow_rate", 0.5);  // L/s
        this->declare_parameter("irrigation_duration", 10.0);  // 秒
        
        // 订阅灌溉点位置
        irrigate_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "irrigate/point", 10,
            std::bind(&IrrigateNode::irrigatePointCallback, this, std::placeholders::_1));
        
        // 发布灌溉命令
        irrigate_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            "irrigate/command", 10);
        
        // 发布水流控制
        water_flow_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "irrigate/water_flow", 10);
        
        // 发布状态
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "irrigate/status", 10);
        
        irrigating_ = false;
        
        RCLCPP_INFO(this->get_logger(), "灌溉节点已启动");
    }

private:
    void irrigatePointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_irrigate_point_ = msg;
        startIrrigation();
    }
    
    void startIrrigation()
    {
        if (irrigating_) {
            return;
        }
        
        irrigating_ = true;
        
        // 获取参数
        double flow_rate = this->get_parameter("water_flow_rate").as_double();
        
        // 发布启动命令
        auto cmd = std_msgs::msg::String();
        cmd.data = "start";
        irrigate_cmd_pub_->publish(cmd);
        
        // 发布水流控制
        auto flow = std_msgs::msg::Float32();
        flow.data = flow_rate;
        water_flow_pub_->publish(flow);
        
        // 发布状态
        auto status = std_msgs::msg::String();
        status.data = "irrigating";
        status_pub_->publish(status);
        
        RCLCPP_INFO(this->get_logger(), "开始灌溉，水流速率: %.2f L/s", flow_rate);
        
        // 设置定时器停止灌溺
        double duration = this->get_parameter("irrigation_duration").as_double();
        stop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(duration * 1000)),
            std::bind(&IrrigateNode::stopIrrigation, this));
        stop_timer_->cancel();  // 立即取消，然后重置
        stop_timer_->reset();  // 重置定时器
    }
    
    void stopIrrigation()
    {
        irrigating_ = false;
        
        // 发布停止命令
        auto cmd = std_msgs::msg::String();
        cmd.data = "stop";
        irrigate_cmd_pub_->publish(cmd);
        
        // 停止水流
        auto flow = std_msgs::msg::Float32();
        flow.data = 0.0;
        water_flow_pub_->publish(flow);
        
        // 发布状态
        auto status = std_msgs::msg::String();
        status.data = "complete";
        status_pub_->publish(status);
        
        RCLCPP_INFO(this->get_logger(), "灌溉完成");
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr irrigate_point_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr irrigate_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr water_flow_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    
    geometry_msgs::msg::PoseStamped::SharedPtr current_irrigate_point_;
    bool irrigating_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IrrigateNode>());
    rclcpp::shutdown();
    return 0;
}

