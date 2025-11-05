/**
 * SLAM节点
 * 使用LiDAR和深度摄像头数据进行SLAM建图和定位
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

class SLAMNode : public rclcpp::Node
{
public:
    SLAMNode() : Node("slam_node")
    {
        // 声明参数
        this->declare_parameter("map_update_rate", 5.0);
        
        // 订阅话题
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "lidar/scan", 10,
            std::bind(&SLAMNode::lidarCallback, this, std::placeholders::_1));
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "depth/points", 10,
            std::bind(&SLAMNode::depthCallback, this, std::placeholders::_1));
        
        // 发布话题
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
        
        // TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 定时器
        double update_rate = this->get_parameter("map_update_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate)),
            std::bind(&SLAMNode::updateMap, this));
        
        // 初始化地图
        initializeMap();
        
        RCLCPP_INFO(this->get_logger(), "SLAM节点已启动");
    }

private:
    void initializeMap()
    {
        map_.info.width = 400;
        map_.info.height = 400;
        map_.info.resolution = 0.05;  // 5cm/pixel
        map_.info.origin.position.x = -10.0;
        map_.info.origin.position.y = -10.0;
        map_.info.origin.orientation.w = 1.0;
        map_.data.resize(map_.info.width * map_.info.height, -1);  // 未知区域
        
        current_pose_.pose.pose.position.x = 0.0;
        current_pose_.pose.pose.position.y = 0.0;
        current_pose_.pose.pose.orientation.w = 1.0;
    }
    
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 处理LiDAR数据
        lidar_data_ = msg;
        processLidarData();
    }
    
    void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 处理深度摄像头数据
        depth_data_ = msg;
        processDepthData();
    }
    
    void processLidarData()
    {
        if (!lidar_data_) return;
        
        float robot_x = current_pose_.pose.pose.position.x;
        float robot_y = current_pose_.pose.pose.position.y;
        
        tf2::Quaternion q(
            current_pose_.pose.pose.orientation.x,
            current_pose_.pose.pose.orientation.y,
            current_pose_.pose.pose.orientation.z,
            current_pose_.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        float angle = lidar_data_->angle_min + yaw;
        for (size_t i = 0; i < lidar_data_->ranges.size(); ++i) {
            float range = lidar_data_->ranges[i];
            
            if (range > lidar_data_->range_min && range < lidar_data_->range_max) {
                float obs_x = robot_x + range * cos(angle);
                float obs_y = robot_y + range * sin(angle);
                
                updateMapCell(obs_x, obs_y, 100);
                
                for (float r = 0.1; r < range - 0.1; r += map_.info.resolution) {
                    float px = robot_x + r * cos(angle);
                    float py = robot_y + r * sin(angle);
                    updateMapCell(px, py, 0);
                }
            }
            
            angle += lidar_data_->angle_increment;
        }
    }
    
    void processDepthData()
    {
        if (!depth_data_) return;
        
        const uint8_t* data_ptr = depth_data_->data.data();
        int point_step = depth_data_->point_step;
        int row_step = depth_data_->row_step;
        
        for (uint32_t y = 0; y < depth_data_->height; y += 4) {
            for (uint32_t x = 0; x < depth_data_->width; x += 4) {
                const uint8_t* point_ptr = data_ptr + y * row_step + x * point_step;
                
                float point_x = *reinterpret_cast<const float*>(point_ptr);
                float point_y = *reinterpret_cast<const float*>(point_ptr + 4);
                float point_z = *reinterpret_cast<const float*>(point_ptr + 8);
                
                if (std::isfinite(point_x) && std::isfinite(point_y) && std::isfinite(point_z)) {
                    if (point_z > 0.1 && point_z < 5.0) {
                        updateMapCell(point_x, point_y, 100);
                        
                        float robot_x = current_pose_.pose.pose.position.x;
                        float robot_y = current_pose_.pose.pose.position.y;
                        
                        float dx = point_x - robot_x;
                        float dy = point_y - robot_y;
                        float dist = sqrt(dx*dx + dy*dy);
                        
                        int steps = static_cast<int>(dist / map_.info.resolution);
                        for (int i = 0; i < steps; ++i) {
                            float px = robot_x + (dx * i / steps);
                            float py = robot_y + (dy * i / steps);
                            updateMapCell(px, py, 0);
                        }
                    }
                }
            }
        }
    }
    
    void updateMapCell(float x, float y, int8_t value)
    {
        int map_x = static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
        int map_y = static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);
        
        if (map_x >= 0 && map_x < static_cast<int>(map_.info.width) &&
            map_y >= 0 && map_y < static_cast<int>(map_.info.height)) {
            int index = map_y * map_.info.width + map_x;
            
            if (value == 0) {
                if (map_.data[index] == -1) {
                    map_.data[index] = 0;
                } else if (map_.data[index] < 100) {
                    map_.data[index] = std::max(0, map_.data[index] - 1);
                }
            } else if (value == 100) {
                if (map_.data[index] == -1) {
                    map_.data[index] = 100;
                } else if (map_.data[index] < 100) {
                    map_.data[index] = std::min(100, map_.data[index] + 10);
                }
            }
        }
    }
    
    void updateMap()
    {
        // 更新时间戳
        map_.header.stamp = this->now();
        map_.header.frame_id = "map";
        
        // 发布地图
        map_pub_->publish(map_);
        
        // 发布位姿
        current_pose_.header.stamp = this->now();
        current_pose_.header.frame_id = "map";
        pose_pub_->publish(current_pose_);
        
        // 发布TF变换
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = current_pose_.pose.pose.position.x;
        transform.transform.translation.y = current_pose_.pose.pose.position.y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = current_pose_.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    sensor_msgs::msg::LaserScan::SharedPtr lidar_data_;
    sensor_msgs::msg::PointCloud2::SharedPtr depth_data_;
    nav_msgs::msg::OccupancyGrid map_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMNode>());
    rclcpp::shutdown();
    return 0;
}

