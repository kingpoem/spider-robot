/**
 * 农业任务执行节点
 * 协调各种农业任务（采摘、监测、灌溉等）
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <string>

enum AgricultureTask {
    TASK_IDLE,
    TASK_HARVEST,
    TASK_COLLECT,
    TASK_MONITOR,
    TASK_IRRIGATE
};

class AgricultureNode : public rclcpp::Node
{
public:
    AgricultureNode() : Node("agriculture_node")
    {
        // 声明参数
        this->declare_parameter("task_mode", "idle");
        this->declare_parameter("harvest_force_threshold", 5.0);
        this->declare_parameter("irrigation_water_flow", 0.5);
        
        // 订阅任务命令
        task_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "agriculture/task_command", 10,
            std::bind(&AgricultureNode::taskCommandCallback, this, std::placeholders::_1));
        
        // 订阅目标位置（从视觉识别或任务规划）
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "agriculture/target_pose", 10,
            std::bind(&AgricultureNode::targetPoseCallback, this, std::placeholders::_1));
        
        // 订阅检测到的物体（从视觉模块）
        detection_sub_ = this->create_subscription<std_msgs::msg::String>(
            "vision/detected_objects", 10,
            std::bind(&AgricultureNode::detectionCallback, this, std::placeholders::_1));
        
        // 发布移动命令
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // 发布目标位置到导航
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        
        // 发布任务状态
        task_status_pub_ = this->create_publisher<std_msgs::msg::String>("agriculture/task_status", 10);
        
        // 发布执行器控制命令
        actuator_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("actuator/command", 10);
        
        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AgricultureNode::taskLoop, this));
        
        current_task_ = TASK_IDLE;
        task_complete_ = false;
        
        RCLCPP_INFO(this->get_logger(), "农业任务节点已启动");
    }

private:
    void taskCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        
        if (cmd == "harvest") {
            current_task_ = TASK_HARVEST;
            RCLCPP_INFO(this->get_logger(), "开始采摘任务");
        } else if (cmd == "collect") {
            current_task_ = TASK_COLLECT;
            RCLCPP_INFO(this->get_logger(), "开始收集任务");
        } else if (cmd == "monitor") {
            current_task_ = TASK_MONITOR;
            RCLCPP_INFO(this->get_logger(), "开始监测任务");
        } else if (cmd == "irrigate") {
            current_task_ = TASK_IRRIGATE;
            RCLCPP_INFO(this->get_logger(), "开始灌溉任务");
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
        // 处理检测到的物体信息
        detected_objects_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "检测到物体: %s", detected_objects_.c_str());
    }
    
    void taskLoop()
    {
        if (current_task_ == TASK_IDLE) {
            return;
        }
        
        switch (current_task_) {
            case TASK_HARVEST:
                executeHarvestTask();
                break;
            case TASK_COLLECT:
                executeCollectTask();
                break;
            case TASK_MONITOR:
                executeMonitorTask();
                break;
            case TASK_IRRIGATE:
                executeIrrigateTask();
                break;
            default:
                break;
        }
        
        // 发布任务状态
        publishTaskStatus();
    }
    
    void executeHarvestTask()
    {
        if (!current_target_) {
            // 等待目标位置
            return;
        }
        
        // 移动到目标位置
        if (!task_complete_) {
            goal_pub_->publish(*current_target_);
            
            // 检查是否到达目标（简化处理）
            // 实际中应该检查当前位置
            
            // 执行采摘动作
            auto cmd = std_msgs::msg::String();
            cmd.data = "harvest";
            actuator_cmd_pub_->publish(cmd);
            
            // 采摘完成
            task_complete_ = true;
        }
    }
    
    void executeCollectTask()
    {
        // 收集任务逻辑
        RCLCPP_DEBUG(this->get_logger(), "执行收集任务");
    }
    
    void executeMonitorTask()
    {
        // 监测任务逻辑
        // 移动到监测点，采集数据
        if (current_target_) {
            goal_pub_->publish(*current_target_);
            
            // 启动监测
            auto cmd = std_msgs::msg::String();
            cmd.data = "monitor_start";
            actuator_cmd_pub_->publish(cmd);
        }
    }
    
    void executeIrrigateTask()
    {
        // 灌溉任务逻辑
        if (current_target_) {
            goal_pub_->publish(*current_target_);
            
            // 启动灌溉
            auto cmd = std_msgs::msg::String();
            cmd.data = "irrigate_start";
            actuator_cmd_pub_->publish(cmd);
            
            // 控制水流
            double flow_rate = this->get_parameter("irrigation_water_flow").as_double();
            // 发布水流控制命令
        }
    }
    
    void publishTaskStatus()
    {
        auto status = std_msgs::msg::String();
        std::string task_name;
        
        switch (current_task_) {
            case TASK_HARVEST:
                task_name = "harvest";
                break;
            case TASK_COLLECT:
                task_name = "collect";
                break;
            case TASK_MONITOR:
                task_name = "monitor";
                break;
            case TASK_IRRIGATE:
                task_name = "irrigate";
                break;
            default:
                task_name = "idle";
                break;
        }
        
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

