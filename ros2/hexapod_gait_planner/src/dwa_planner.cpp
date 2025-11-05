#include "dwa_planner.hpp"
#include <algorithm>

GaitCommand DWAPlanner::plan(const geometry_msgs::msg::Twist::SharedPtr& target_vel,
                             const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
{
    GaitCommand best_cmd;
    best_cmd.linear_velocity = 0;
    best_cmd.angular_velocity = 0;
    best_cmd.step_length = 0.15;
    best_cmd.step_height = 0.1;
    best_cmd.gait_type = 0;
    
    if (!target_vel || !map) {
        return best_cmd;
    }
    
    // 获取当前位置（从TF或参数）
    // 这里简化处理，实际应从TF获取
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_theta_ = 0.0;
    
    // 生成速度窗口
    std::vector<Velocity> velocities = generateVelocities(
        target_vel->linear.x, target_vel->angular.z);
    
    // 评估每个轨迹
    double best_score = -1.0;
    Velocity best_vel;
    
    for (const auto& vel : velocities) {
        double score = evaluateTrajectory(vel, map, *target_vel);
        if (score > best_score) {
            best_score = score;
            best_vel = vel;
        }
    }
    
    // 转换为步态命令
    best_cmd.linear_velocity = best_vel.v;
    best_cmd.angular_velocity = best_vel.w;
    
    // 根据速度调整步态参数
    if (std::abs(best_cmd.linear_velocity) > 0.2) {
        best_cmd.step_length = 0.2;
        best_cmd.gait_type = 0;  // 三脚架步态（快速）
    } else if (std::abs(best_cmd.linear_velocity) > 0.1) {
        best_cmd.step_length = 0.15;
        best_cmd.gait_type = 1;  // 波浪步态（中等）
    } else {
        best_cmd.step_length = 0.1;
        best_cmd.gait_type = 2;  // 涟漪步态（慢速）
    }
    
    return best_cmd;
}

std::vector<DWAPlanner::Velocity> DWAPlanner::generateVelocities(double v, double w)
{
    std::vector<Velocity> velocities;
    
    // 计算动态窗口
    double v_min = std::max(0.0, v - max_accel_ * dt_);
    double v_max = std::min(max_linear_vel_, v + max_accel_ * dt_);
    double w_min = std::max(-max_angular_vel_, w - max_angular_accel_ * dt_);
    double w_max = std::min(max_angular_vel_, w + max_angular_accel_ * dt_);
    
    // 生成速度采样
    for (double v_sample = v_min; v_sample <= v_max; v_sample += v_resolution_) {
        for (double w_sample = w_min; w_sample <= w_max; w_sample += w_resolution_) {
            velocities.push_back({v_sample, w_sample});
        }
    }
    
    return velocities;
}

double DWAPlanner::evaluateTrajectory(const Velocity& vel,
                                      const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
                                      const geometry_msgs::msg::Twist& target_vel)
{
    // 检查轨迹是否有效
    if (!isTrajectoryValid(vel, map)) {
        return -1.0;
    }
    
    // 计算综合评分
    double obstacle_cost = calculateObstacleCost(vel, map);
    double goal_cost = calculateGoalCost(vel, target_vel);
    
    // 综合评分（越小越好，所以取负值）
    double score = -obstacle_cost + goal_cost;
    
    return score;
}

bool DWAPlanner::isTrajectoryValid(const Velocity& vel,
                                   const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
{
    // 模拟轨迹
    double x = current_x_;
    double y = current_y_;
    double theta = current_theta_;
    
    int steps = static_cast<int>(dt_ / 0.01);  // 每10ms一步
    
    for (int i = 0; i < steps; ++i) {
        // 更新位置
        x += vel.v * cos(theta) * 0.01;
        y += vel.v * sin(theta) * 0.01;
        theta += vel.w * 0.01;
        
        // 检查是否碰撞
        int map_x = static_cast<int>((x - map->info.origin.position.x) / map->info.resolution);
        int map_y = static_cast<int>((y - map->info.origin.position.y) / map->info.resolution);
        
        if (map_x >= 0 && map_x < static_cast<int>(map->info.width) &&
            map_y >= 0 && map_y < static_cast<int>(map->info.height)) {
            int index = map_y * map->info.width + map_x;
            if (map->data[index] > 50) {  // 占用
                return false;
            }
        }
    }
    
    return true;
}

double DWAPlanner::calculateObstacleCost(const Velocity& vel,
                                         const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
{
    // 计算到最近障碍物的距离
    double min_dist = 10.0;  // 最大距离
    
    double x = current_x_;
    double y = current_y_;
    double theta = current_theta_;
    
    // 向前预测
    for (double t = 0; t < dt_; t += 0.01) {
        x += vel.v * cos(theta) * 0.01;
        y += vel.v * sin(theta) * 0.01;
        theta += vel.w * 0.01;
        
        // 检查周围障碍物
        for (double dx = -0.2; dx <= 0.2; dx += 0.05) {
            for (double dy = -0.2; dy <= 0.2; dy += 0.05) {
                double check_x = x + dx;
                double check_y = y + dy;
                
                int map_x = static_cast<int>((check_x - map->info.origin.position.x) / map->info.resolution);
                int map_y = static_cast<int>((check_y - map->info.origin.position.y) / map->info.resolution);
                
                if (map_x >= 0 && map_x < static_cast<int>(map->info.width) &&
                    map_y >= 0 && map_y < static_cast<int>(map->info.height)) {
                    int index = map_y * map->info.width + map_x;
                    if (map->data[index] > 50) {
                        double dist = sqrt(dx*dx + dy*dy);
                        if (dist < min_dist) {
                            min_dist = dist;
                        }
                    }
                }
            }
        }
    }
    
    // 距离越近，代价越大
    return 1.0 / (min_dist + 0.1);
}

double DWAPlanner::calculateGoalCost(const Velocity& vel, const geometry_msgs::msg::Twist& target)
{
    // 计算速度误差
    double v_error = std::abs(vel.v - target.linear.x);
    double w_error = std::abs(vel.w - target.angular.z);
    
    // 误差越小，评分越高
    return 1.0 / (v_error + w_error + 0.1);
}

