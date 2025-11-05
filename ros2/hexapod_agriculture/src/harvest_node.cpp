/**
 * 采摘任务节点
 * 专门处理农作物采摘任务
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>

class HarvestNode : public rclcpp::Node
{
public:
    HarvestNode() : Node("harvest_node")
    {
        // 订阅目标作物位置
        crop_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "harvest/crop_pose", 10,
            std::bind(&HarvestNode::cropPoseCallback, this, std::placeholders::_1));
        
        // 订阅力传感器数据
        force_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "sensors/gripper_force", 10,
            std::bind(&HarvestNode::forceCallback, this, std::placeholders::_1));
        
        // 发布采摘命令
        harvest_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            "harvest/command", 10);
        
        // 发布状态
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "harvest/status", 10);
        
        harvest_force_threshold_ = 5.0;  // 采摘力阈值(N)
        current_state_ = "idle";
        
        RCLCPP_INFO(this->get_logger(), "采摘节点已启动");
    }

private:
    void cropPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_crop_pose_ = msg;
        startHarvest();
    }
    
    void forceCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_force_ = msg->data;
        
        if (current_state_ == "grasping" && current_force_ > harvest_force_threshold_) {
            // 达到采摘力阈值，执行采摘
            executeHarvest();
        }
    }
    
    void startHarvest()
    {
        if (current_state_ != "idle") {
            return;
        }
        
        current_state_ = "approaching";
        
        // 发布接近命令
        auto cmd = std_msgs::msg::String();
        cmd.data = "approach";
        harvest_cmd_pub_->publish(cmd);
        
        RCLCPP_INFO(this->get_logger(), "开始接近作物");
    }
    
    void executeHarvest()
    {
        current_state_ = "harvesting";
        
        // 发布采摘命令
        auto cmd = std_msgs::msg::String();
        cmd.data = "harvest";
        harvest_cmd_pub_->publish(cmd);
        
        RCLCPP_INFO(this->get_logger(), "执行采摘动作");
        
        // 等待采摘完成
        // 实际中应该有反馈机制
        
        current_state_ = "complete";
        publishStatus();
    }
    
    void publishStatus()
    {
        auto status = std_msgs::msg::String();
        status.data = current_state_;
        status_pub_->publish(status);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr crop_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr harvest_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    geometry_msgs::msg::PoseStamped::SharedPtr current_crop_pose_;
    float current_force_;
    float harvest_force_threshold_;
    std::string current_state_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HarvestNode>());
    rclcpp::shutdown();
    return 0;
}

