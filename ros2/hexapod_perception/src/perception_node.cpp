/**
 * 环境感知节点
 * 结合视觉和SLAM数据进行环境理解
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>

struct TerrainFeature {
    float slope;           // 坡度
    float roughness;       // 粗糙度
    bool traversable;      // 是否可通过
    geometry_msgs::msg::Point position;
};

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode() : Node("perception_node")
    {
        // 声明参数
        this->declare_parameter("map_topic", "map");
        this->declare_parameter("camera_topic", "camera/image_raw");
        this->declare_parameter("lidar_topic", "lidar/scan");
        
        std::string map_topic = this->get_parameter("map_topic").as_string();
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        
        // 订阅SLAM地图
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, 10,
            std::bind(&PerceptionNode::mapCallback, this, std::placeholders::_1));
        
        // 订阅相机图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10,
            std::bind(&PerceptionNode::imageCallback, this, std::placeholders::_1));
        
        // 订阅LiDAR数据
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10,
            std::bind(&PerceptionNode::lidarCallback, this, std::placeholders::_1));
        
        // 订阅机器人位姿
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 10,
            std::bind(&PerceptionNode::poseCallback, this, std::placeholders::_1));
        
        // 发布地形分析结果
        terrain_pub_ = this->create_publisher<std_msgs::msg::String>(
            "perception/terrain_analysis", 10);
        
        // 发布障碍物信息
        obstacles_pub_ = this->create_publisher<std_msgs::msg::String>(
            "perception/obstacles", 10);
        
        // 发布环境特征
        features_pub_ = this->create_publisher<std_msgs::msg::String>(
            "perception/features", 10);
        
        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PerceptionNode::perceptionLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "环境感知节点已启动");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = msg;
        analyzeTerrain();
    }
    
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image;
            analyzeVisualFeatures();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        }
    }
    
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        current_lidar_ = msg;
        analyzeObstacles();
    }
    
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = msg;
    }
    
    void analyzeTerrain()
    {
        if (!current_map_) {
            return;
        }
        
        int free_cells = 0;
        int occupied_cells = 0;
        int unknown_cells = 0;
        
        for (int y = 1; y < static_cast<int>(current_map_->info.height) - 1; ++y) {
            for (int x = 1; x < static_cast<int>(current_map_->info.width) - 1; ++x) {
                int idx = y * current_map_->info.width + x;
                int8_t cell = current_map_->data[idx];
                
                if (cell == 0) {
                    free_cells++;
                } else if (cell > 50) {
                    occupied_cells++;
                } else {
                    unknown_cells++;
                }
            }
        }
        
        float slope_estimate = 0.0;
        for (int y = 1; y < static_cast<int>(current_map_->info.height) - 1; ++y) {
            for (int x = 1; x < static_cast<int>(current_map_->info.width) - 1; ++x) {
                int idx = y * current_map_->info.width + x;
                int idx_up = (y - 1) * current_map_->info.width + x;
                int idx_down = (y + 1) * current_map_->info.width + x;
                
                if (current_map_->data[idx] == 0 && 
                    current_map_->data[idx_up] > 50 && 
                    current_map_->data[idx_down] > 50) {
                    slope_estimate += 0.1;
                }
            }
        }
        
        auto terrain_msg = std_msgs::msg::String();
        terrain_msg.data = "地形分析: 自由=" + std::to_string(free_cells) +
                          ", 占用=" + std::to_string(occupied_cells) +
                          ", 未知=" + std::to_string(unknown_cells) +
                          ", 坡度估计=" + std::to_string(slope_estimate);
        terrain_pub_->publish(terrain_msg);
    }
    
    void analyzeVisualFeatures()
    {
        if (current_image_.empty()) {
            return;
        }
        
        cv::Mat gray;
        cv::cvtColor(current_image_, gray, cv::COLOR_BGR2GRAY);
        
        cv::Mat edges;
        cv::Canny(gray, edges, 50, 150);
        
        int edge_count = cv::countNonZero(edges);
        float roughness = static_cast<float>(edge_count) / (edges.rows * edges.cols);
        
        cv::Mat grad_x, grad_y;
        cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3);
        cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3);
        
        cv::Mat magnitude;
        cv::magnitude(grad_x, grad_y, magnitude);
        
        cv::Scalar mean_val = cv::mean(magnitude);
        float texture_variance = static_cast<float>(mean_val[0]);
        
        auto features_msg = std_msgs::msg::String();
        features_msg.data = "视觉特征: 粗糙度=" + std::to_string(roughness) +
                           ", 纹理方差=" + std::to_string(texture_variance);
        features_pub_->publish(features_msg);
    }
    
    void analyzeObstacles()
    {
        if (!current_lidar_) {
            return;
        }
        
        // 分析障碍物
        std::vector<float> obstacles;
        
        float angle = current_lidar_->angle_min;
        for (size_t i = 0; i < current_lidar_->ranges.size(); ++i) {
            float range = current_lidar_->ranges[i];
            
            if (range > current_lidar_->range_min && 
                range < current_lidar_->range_max &&
                range < 2.0) {  // 2米内的障碍物
                obstacles.push_back(range);
            }
            
            angle += current_lidar_->angle_increment;
        }
        
        // 发布障碍物信息
        auto obstacles_msg = std_msgs::msg::String();
        obstacles_msg.data = "检测到 " + std::to_string(obstacles.size()) + " 个近距离障碍物";
        obstacles_pub_->publish(obstacles_msg);
    }
    
    void perceptionLoop()
    {
        if (current_map_ && current_pose_) {
            float robot_x = current_pose_->pose.pose.position.x;
            float robot_y = current_pose_->pose.pose.position.y;
            
            int map_x = static_cast<int>((robot_x - current_map_->info.origin.position.x) / current_map_->info.resolution);
            int map_y = static_cast<int>((robot_y - current_map_->info.origin.position.y) / current_map_->info.resolution);
            
            int reachable_count = 0;
            int obstacle_count = 0;
            
            for (int dy = -5; dy <= 5; ++dy) {
                for (int dx = -5; dx <= 5; ++dx) {
                    int check_x = map_x + dx;
                    int check_y = map_y + dy;
                    
                    if (check_x >= 0 && check_x < static_cast<int>(current_map_->info.width) &&
                        check_y >= 0 && check_y < static_cast<int>(current_map_->info.height)) {
                        int idx = check_y * current_map_->info.width + check_x;
                        if (current_map_->data[idx] == 0) {
                            reachable_count++;
                        } else if (current_map_->data[idx] > 50) {
                            obstacle_count++;
                        }
                    }
                }
            }
            
            float safety_score = static_cast<float>(reachable_count) / (reachable_count + obstacle_count + 1);
            
            auto features_msg = std_msgs::msg::String();
            features_msg.data = "环境感知: 可达性=" + std::to_string(reachable_count) +
                              ", 障碍物=" + std::to_string(obstacle_count) +
                              ", 安全评分=" + std::to_string(safety_score);
            features_pub_->publish(features_msg);
        }
    }
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr terrain_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr features_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    cv::Mat current_image_;
    sensor_msgs::msg::LaserScan::SharedPtr current_lidar_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}

