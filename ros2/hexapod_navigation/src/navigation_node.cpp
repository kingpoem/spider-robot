/**
 * 导航节点
 * 使用Nav2进行路径规划和导航
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_node")
    {
        // 订阅目标位置
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&NavigationNode::goalCallback, this, std::placeholders::_1));
        
        // 订阅路径
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10,
            std::bind(&NavigationNode::pathCallback, this, std::placeholders::_1));
        
        // 发布速度命令
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&NavigationNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "导航节点已启动");
    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_goal_ = msg;
        RCLCPP_INFO(this->get_logger(), "收到新目标位置");
    }
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        current_path_ = msg;
    }
    
    void controlLoop()
    {
        if (!current_path_ || current_path_->poses.empty()) {
            return;
        }
        
        // 简单的路径跟踪控制
        // 实际应用中应使用更复杂的控制器
        
        auto cmd = geometry_msgs::msg::Twist();
        
        // 计算到下一个路径点的距离和角度
        // 这里简化处理
        cmd.linear.x = 0.1;
        cmd.angular.z = 0.0;
        
        cmd_vel_pub_->publish(cmd);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::PoseStamped::SharedPtr current_goal_;
    nav_msgs::msg::Path::SharedPtr current_path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}

