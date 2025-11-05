/**
 * 监测任务节点
 * 用于监测作物生长、土壤状况等
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class MonitorNode : public rclcpp::Node
{
public:
    MonitorNode() : Node("monitor_node")
    {
        // 订阅监测点位置
        monitor_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "monitor/point", 10,
            std::bind(&MonitorNode::monitorPointCallback, this, std::placeholders::_1));
        
        // 订阅相机图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&MonitorNode::imageCallback, this, std::placeholders::_1));
        
        // 发布监测数据
        monitor_data_pub_ = this->create_publisher<std_msgs::msg::String>(
            "monitor/data", 10);
        
        // 发布监测状态
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "monitor/status", 10);
        
        monitoring_ = false;
        
        RCLCPP_INFO(this->get_logger(), "监测节点已启动");
    }

private:
    void monitorPointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_monitor_point_ = msg;
        startMonitoring();
    }
    
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (!monitoring_) {
            return;
        }
        
        // 处理图像数据，提取监测信息
        processMonitoringData(msg);
    }
    
    void startMonitoring()
    {
        monitoring_ = true;
        
        auto status = std_msgs::msg::String();
        status.data = "monitoring";
        status_pub_->publish(status);
        
        RCLCPP_INFO(this->get_logger(), "开始监测");
    }
    
    void processMonitoringData(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        // 这里可以添加图像分析逻辑
        // 例如：分析作物颜色、大小、健康状态等
        
        auto data = std_msgs::msg::String();
        data.data = "监测数据: 图像大小=" + std::to_string(msg->width) + "x" + std::to_string(msg->height);
        monitor_data_pub_->publish(data);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr monitor_point_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr monitor_data_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    geometry_msgs::msg::PoseStamped::SharedPtr current_monitor_point_;
    bool monitoring_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitorNode>());
    rclcpp::shutdown();
    return 0;
}

