#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>

enum AgricultureTask {
    TASK_IDLE,
    TASK_MONITOR
};

class AgricultureNode : public rclcpp::Node
{
public:
    AgricultureNode() : Node("agriculture_node")
    {
        this->declare_parameter("task_mode", "idle");
        
        task_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "agriculture/task_command", 10,
            std::bind(&AgricultureNode::taskCommandCallback, this, std::placeholders::_1));
        
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "agriculture/target_pose", 10,
            std::bind(&AgricultureNode::targetPoseCallback, this, std::placeholders::_1));
        
        detection_sub_ = this->create_subscription<std_msgs::msg::String>(
            "vision/detected_objects", 10,
            std::bind(&AgricultureNode::detectionCallback, this, std::placeholders::_1));
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        task_status_pub_ = this->create_publisher<std_msgs::msg::String>("agriculture/task_status", 10);
        actuator_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("actuator/command", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AgricultureNode::taskLoop, this));
        
        current_task_ = TASK_IDLE;
        task_complete_ = false;
        
        RCLCPP_INFO(this->get_logger(), "农业监测任务节点已启动");
    }

private:
    void taskCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        
        if (cmd == "monitor") {
            current_task_ = TASK_MONITOR;
            RCLCPP_INFO(this->get_logger(), "开始监测任务");
        } else if (cmd == "stop") {
            current_task_ = TASK_IDLE;
            RCLCPP_INFO(this->get_logger(), "停止任务");
        }
        
        task_complete_ = false;
    }
    
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_target_ = msg;
    }
    
    void detectionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        detected_objects_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "检测到物体: %s", detected_objects_.c_str());
    }
    
    void taskLoop()
    {
        if (current_task_ == TASK_IDLE) {
            return;
        }
        
        if (current_task_ == TASK_MONITOR) {
            executeMonitorTask();
        }
        
        publishTaskStatus();
    }
    
    void executeMonitorTask()
    {
        if (current_target_) {
            goal_pub_->publish(*current_target_);
            
            auto cmd = std_msgs::msg::String();
            cmd.data = "monitor_start";
            actuator_cmd_pub_->publish(cmd);
        }
    }
    
    void publishTaskStatus()
    {
        auto status = std_msgs::msg::String();
        std::string task_name = (current_task_ == TASK_MONITOR) ? "monitor" : "idle";
        status.data = task_name + (task_complete_ ? ":complete" : ":running");
        task_status_pub_->publish(status);
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr actuator_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    AgricultureTask current_task_;
    bool task_complete_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_target_;
    std::string detected_objects_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgricultureNode>());
    rclcpp::shutdown();
    return 0;
}

