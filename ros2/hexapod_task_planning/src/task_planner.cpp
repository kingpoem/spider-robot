#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

enum TaskType {
    TASK_MONITOR
};

class TaskPlannerNode : public rclcpp::Node
{
public:
    TaskPlannerNode() : Node("task_planner")
    {
        task_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "task/command", 10,
            std::bind(&TaskPlannerNode::taskCommandCallback, this, std::placeholders::_1));
        
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        task_status_pub_ = this->create_publisher<std_msgs::msg::String>("task/status", 10);
        
        current_task_ = -1;
        
        RCLCPP_INFO(this->get_logger(), "任务规划节点已启动");
    }

private:
    void taskCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        
        if (cmd == "monitor") {
            current_task_ = TASK_MONITOR;
            planMonitorTask();
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

