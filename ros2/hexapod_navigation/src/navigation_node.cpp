/**
 * 导航节点
 * 使用Nav2进行路径规划和导航
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("lookahead_distance", 0.5);
        this->declare_parameter("kp_linear", 0.5);
        this->declare_parameter("kp_angular", 1.0);
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&NavigationNode::goalCallback, this, std::placeholders::_1));
        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10,
            std::bind(&NavigationNode::pathCallback, this, std::placeholders::_1));
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
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
        
        auto cmd = geometry_msgs::msg::Twist();
        
        double current_x = 0.0;
        double current_y = 0.0;
        double current_theta = 0.0;
        
        try {
            std::string base_frame = this->get_parameter("base_frame").as_string();
            std::string map_frame = this->get_parameter("map_frame").as_string();
            
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                map_frame, base_frame, tf2::TimePointZero);
            
            current_x = transform.transform.translation.x;
            current_y = transform.transform.translation.y;
            
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, current_theta);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TF查找失败: %s", ex.what());
            return;
        }
        
        // 找到路径上最近的路径点
        size_t nearest_idx = 0;
        double min_dist = 1000.0;
        
        for (size_t i = 0; i < current_path_->poses.size(); ++i) {
            double dx = current_path_->poses[i].pose.position.x - current_x;
            double dy = current_path_->poses[i].pose.position.y - current_y;
            double dist = sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        double lookahead_dist = this->get_parameter("lookahead_distance").as_double();
        size_t target_idx = nearest_idx;
        
        for (size_t i = nearest_idx; i < current_path_->poses.size(); ++i) {
            double dx = current_path_->poses[i].pose.position.x - current_x;
            double dy = current_path_->poses[i].pose.position.y - current_y;
            double dist = sqrt(dx*dx + dy*dy);
            
            if (dist >= lookahead_dist) {
                target_idx = i;
                break;
            }
        }
        
        if (target_idx >= current_path_->poses.size()) {
            target_idx = current_path_->poses.size() - 1;
        }
        
        // 计算到目标点的角度
        double target_x = current_path_->poses[target_idx].pose.position.x;
        double target_y = current_path_->poses[target_idx].pose.position.y;
        
        double dx = target_x - current_x;
        double dy = target_y - current_y;
        double target_theta = atan2(dy, dx);
        
        // 计算角度误差
        double angle_error = target_theta - current_theta;
        // 归一化到[-π, π]
        while (angle_error > M_PI) angle_error -= 2*M_PI;
        while (angle_error < -M_PI) angle_error += 2*M_PI;
        
        double distance = sqrt(dx*dx + dy*dy);
        
        double kp_linear = this->get_parameter("kp_linear").as_double();
        double kp_angular = this->get_parameter("kp_angular").as_double();
        
        cmd.linear.x = kp_linear * distance;
        cmd.angular.z = kp_angular * angle_error;
        
        if (cmd.linear.x > 0.3) cmd.linear.x = 0.3;
        if (cmd.linear.x < -0.3) cmd.linear.x = -0.3;
        if (cmd.angular.z > 1.0) cmd.angular.z = 1.0;
        if (cmd.angular.z < -1.0) cmd.angular.z = -1.0;
        
        cmd_vel_pub_->publish(cmd);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
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

