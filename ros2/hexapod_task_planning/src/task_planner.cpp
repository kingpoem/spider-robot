/**
 * 任务规划节点
 * 规划采摘、收集、监测、灌溉任务
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

enum TaskType {
    TASK_HARVEST,    // 采摘
    TASK_COLLECT,    // 收集
    TASK_MONITOR,    // 监测
    TASK_IRRIGATE    // 灌溉
};

class TaskPlannerNode : public rclcpp::Node
{
public:
    TaskPlannerNode() : Node("task_planner")
    {
        // 订阅任务命令
        task_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "task/command", 10,
            std::bind(&TaskPlannerNode::taskCommandCallback, this, std::placeholders::_1));
        
        // 发布目标位置
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        
        // 发布任务状态
        task_status_pub_ = this->create_publisher<std_msgs::msg::String>("task/status", 10);
        
        current_task_ = -1;
        
        RCLCPP_INFO(this->get_logger(), "任务规划节点已启动");
    }

private:
    void taskCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        
        if (cmd == "harvest") {
            current_task_ = TASK_HARVEST;
            planHarvestTask();
        } else if (cmd == "collect") {
            current_task_ = TASK_COLLECT;
            planCollectTask();
        } else if (cmd == "monitor") {
            current_task_ = TASK_MONITOR;
            planMonitorTask();
        } else if (cmd == "irrigate") {
            current_task_ = TASK_IRRIGATE;
            planIrrigateTask();
        }
    }
    
    void planHarvestTask()
    {
        RCLCPP_INFO(this->get_logger(), "规划采摘任务");
        
        std::vector<std::pair<double, double>> crop_positions = {
            {5.0, 3.0}, {5.5, 3.5}, {6.0, 3.0}, {6.5, 3.5}
        };
        
        for (const auto& pos : crop_positions) {
            auto goal = geometry_msgs::msg::PoseStamped();
            goal.header.frame_id = "map";
            goal.header.stamp = this->now();
            goal.pose.position.x = pos.first;
            goal.pose.position.y = pos.second;
            goal.pose.orientation.w = 1.0;
            goal_pub_->publish(goal);
        }
    }
    
    void planCollectTask()
    {
        RCLCPP_INFO(this->get_logger(), "规划收集任务");
        
        std::vector<std::pair<double, double>> collection_points = {
            {3.0, 2.0}, {3.5, 2.5}, {4.0, 2.0}
        };
        
        for (size_t i = 0; i < collection_points.size() - 1; ++i) {
            for (size_t j = i + 1; j < collection_points.size(); ++j) {
                double dist1 = sqrt(pow(collection_points[i].first - collection_points[j].first, 2) +
                                   pow(collection_points[i].second - collection_points[j].second, 2));
            }
        }
        
        for (const auto& pos : collection_points) {
            auto goal = geometry_msgs::msg::PoseStamped();
            goal.header.frame_id = "map";
            goal.header.stamp = this->now();
            goal.pose.position.x = pos.first;
            goal.pose.position.y = pos.second;
            goal.pose.orientation.w = 1.0;
            goal_pub_->publish(goal);
        }
    }
    
    void planMonitorTask()
    {
        RCLCPP_INFO(this->get_logger(), "规划监测任务");
        
        std::vector<std::pair<double, double>> monitor_points = {
            {1.0, 1.0}, {1.5, 1.5}, {2.0, 1.0}, {2.5, 1.5}, {3.0, 1.0}
        };
        
        for (const auto& pos : monitor_points) {
            auto goal = geometry_msgs::msg::PoseStamped();
            goal.header.frame_id = "map";
            goal.header.stamp = this->now();
            goal.pose.position.x = pos.first;
            goal.pose.position.y = pos.second;
            goal.pose.orientation.w = 1.0;
            goal_pub_->publish(goal);
        }
    }
    
    void planIrrigateTask()
    {
        RCLCPP_INFO(this->get_logger(), "规划灌溉任务");
        
        std::vector<std::pair<double, double>> irrigate_points;
        for (double x = 4.0; x <= 7.0; x += 0.5) {
            for (double y = 3.0; y <= 5.0; y += 0.5) {
                irrigate_points.push_back({x, y});
            }
        }
        
        for (const auto& pos : irrigate_points) {
            auto goal = geometry_msgs::msg::PoseStamped();
            goal.header.frame_id = "map";
            goal.header.stamp = this->now();
            goal.pose.position.x = pos.first;
            goal.pose.position.y = pos.second;
            goal.pose.orientation.w = 1.0;
            goal_pub_->publish(goal);
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;
    
    int current_task_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskPlannerNode>());
    rclcpp::shutdown();
    return 0;
}

