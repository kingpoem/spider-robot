/**
 * 步态规划节点
 * 集成DWA算法和地形适应性评分
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "dwa_planner.hpp"
#include "terrain_adaptation.hpp"

class GaitPlannerNode : public rclcpp::Node
{
public:
    GaitPlannerNode() : Node("gait_planner")
    {
        // 声明参数
        this->declare_parameter("gait_frequency", 10.0);
        this->declare_parameter("max_linear_vel", 0.3);
        this->declare_parameter("max_angular_vel", 1.0);
        this->declare_parameter("min_step_length", 0.1);
        this->declare_parameter("max_step_length", 0.3);
        this->declare_parameter("min_step_height", 0.05);
        this->declare_parameter("max_step_height", 0.2);
        
        // 订阅话题
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&GaitPlannerNode::cmdVelCallback, this, std::placeholders::_1));
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10,
            std::bind(&GaitPlannerNode::mapCallback, this, std::placeholders::_1));
        
        terrain_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "terrain/points", 10,
            std::bind(&GaitPlannerNode::terrainCallback, this, std::placeholders::_1));
        
        // 发布话题
        gait_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("gait/commands", 10);
        
        // 初始化规划器
        double max_linear = this->get_parameter("max_linear_vel").as_double();
        double max_angular = this->get_parameter("max_angular_vel").as_double();
        dwa_planner_ = std::make_unique<DWAPlanner>(max_linear, max_angular);
        terrain_adaptation_ = std::make_unique<TerrainAdaptation>();
        
        // 定时器
        double freq = this->get_parameter("gait_frequency").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / freq)),
            std::bind(&GaitPlannerNode::planGait, this));
        
        RCLCPP_INFO(this->get_logger(), "步态规划节点已启动");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_velocity_ = msg;
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = msg;
    }
    
    void terrainCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        terrain_data_ = msg;
        // 分析地形
        if (terrain_adaptation_) {
            terrain_adaptation_->updateTerrain(*msg);
        }
    }
    
    void planGait()
    {
        if (!target_velocity_) return;
        
        // 使用DWA规划步态
        GaitCommand cmd;
        
        if (current_map_ && target_velocity_) {
            // 使用地图进行避障
            cmd = dwa_planner_->plan(target_velocity_, current_map_);
        } else if (target_velocity_) {
            // 直接使用速度命令
            cmd.linear_velocity = target_velocity_->linear.x;
            cmd.angular_velocity = target_velocity_->angular.z;
        }
        
        // 地形适应性调整
        if (terrain_adaptation_) {
            TerrainScore score = terrain_adaptation_->getTerrainScore();
            adjustGaitForTerrain(cmd, score);
        }
        
        // 发布步态命令
        publishGaitCommand(cmd);
    }
    
    void adjustGaitForTerrain(GaitCommand& cmd, const TerrainScore& score)
    {
        // 根据地形评分调整步态参数
        // 坡度越大，步高越大
        if (score.slope > 0.3) {
            cmd.step_height = 0.15;  // 增加步高
            cmd.step_length = 0.1;   // 减小步幅
        } else if (score.slope > 0.1) {
            cmd.step_height = 0.1;
            cmd.step_length = 0.15;
        } else {
            cmd.step_height = 0.05;
            cmd.step_length = 0.2;
        }
        
        // 粗糙度越大，步高越大
        if (score.roughness > 0.5) {
            cmd.step_height = std::max(cmd.step_height, 0.12f);
        }
        
        // 可达性检查
        if (score.reachability < 0.5) {
            // 降低速度
            cmd.linear_velocity *= 0.5;
            cmd.angular_velocity *= 0.5;
        }
    }
    
    void publishGaitCommand(const GaitCommand& cmd)
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {
            static_cast<float>(cmd.linear_velocity),
            static_cast<float>(cmd.angular_velocity),
            cmd.step_length,
            cmd.step_height,
            static_cast<float>(cmd.gait_type)
        };
        gait_cmd_pub_->publish(msg);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gait_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::unique_ptr<DWAPlanner> dwa_planner_;
    std::unique_ptr<TerrainAdaptation> terrain_adaptation_;
    
    geometry_msgs::msg::Twist::SharedPtr target_velocity_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    sensor_msgs::msg::PointCloud2::SharedPtr terrain_data_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitPlannerNode>());
    rclcpp::shutdown();
    return 0;
}

