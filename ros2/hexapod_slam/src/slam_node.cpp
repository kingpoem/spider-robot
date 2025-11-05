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
#include <memory>
#include <vector>

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
        
        // 简单的SLAM实现
        // 实际应用中应使用slam_toolbox或cartographer
        
        // 将激光扫描数据转换为地图占用
        float angle = lidar_data_->angle_min;
        for (size_t i = 0; i < lidar_data_->ranges.size(); ++i) {
            float range = lidar_data_->ranges[i];
            
            if (range > lidar_data_->range_min && range < lidar_data_->range_max) {
                // 计算障碍物位置
                float x = range * cos(angle);
                float y = range * sin(angle);
                
                // 更新地图
                updateMapCell(x, y, 100);  // 占用
                
                // 更新路径（占用到传感器之间为空闲）
                for (float r = 0; r < range; r += map_.info.resolution) {
                    float px = r * cos(angle);
                    float py = r * sin(angle);
                    updateMapCell(px, py, 0);  // 空闲
                }
            }
            
            angle += lidar_data_->angle_increment;
        }
    }
    
    void processDepthData()
    {
        // 处理深度摄像头点云数据
        // 这里可以添加点云处理逻辑
    }
    
    void updateMapCell(float x, float y, int8_t value)
    {
        // 将世界坐标转换为地图坐标
        int map_x = static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
        int map_y = static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);
        
        if (map_x >= 0 && map_x < static_cast<int>(map_.info.width) &&
            map_y >= 0 && map_y < static_cast<int>(map_.info.height)) {
            int index = map_y * map_.info.width + map_x;
            map_.data[index] = value;
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

