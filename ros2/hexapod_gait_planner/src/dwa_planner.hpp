/**
 * 动态窗口法（DWA）规划器
 * 用于实时路径规划和避障
 */

#ifndef DWA_PLANNER_HPP
#define DWA_PLANNER_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>

struct GaitCommand {
    double linear_velocity;
    double angular_velocity;
    float step_length;
    float step_height;
    int gait_type;  // 0=tripod, 1=wave, 2=ripple
};

class DWAPlanner {
public:
    DWAPlanner(double max_linear_vel, double max_angular_vel)
        : max_linear_vel_(max_linear_vel)
        , max_angular_vel_(max_angular_vel)
        , max_accel_(0.5)
        , max_angular_accel_(1.0)
        , dt_(0.1)
        , v_resolution_(0.05)
        , w_resolution_(0.1)
    {
    }
    
    GaitCommand plan(const geometry_msgs::msg::Twist::SharedPtr& target_vel,
                     const nav_msgs::msg::OccupancyGrid::SharedPtr& map);
    
private:
    struct Velocity {
        double v;
        double w;
    };
    
    std::vector<Velocity> generateVelocities(double v, double w);
    double evaluateTrajectory(const Velocity& vel, 
                              const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
                              const geometry_msgs::msg::Twist& target_vel);
    bool isTrajectoryValid(const Velocity& vel,
                          const nav_msgs::msg::OccupancyGrid::SharedPtr& map);
    double calculateObstacleCost(const Velocity& vel,
                                 const nav_msgs::msg::OccupancyGrid::SharedPtr& map);
    double calculateGoalCost(const Velocity& vel, const geometry_msgs::msg::Twist& target);
    
    double max_linear_vel_;
    double max_angular_vel_;
    double max_accel_;
    double max_angular_accel_;
    double dt_;
    double v_resolution_;
    double w_resolution_;
    
    // 当前位置（应从TF获取）
    double current_x_;
    double current_y_;
    double current_theta_;
};

#endif // DWA_PLANNER_HPP

