/**
 * 视觉节点
 * 处理相机图像，进行基础图像处理
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include "hexapod_vision/image_converter.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <cstring>

class VisionNode : public rclcpp::Node
{
public:
    VisionNode() : Node("vision_node")
    {
        // 声明参数
        this->declare_parameter("camera_topic", "camera/image_raw");
        this->declare_parameter("output_topic", "vision/processed_image");
        this->declare_parameter("enable_edge_detection", true);
        this->declare_parameter("enable_color_segmentation", true);
        
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        
        // 图像传输
        image_transport::ImageTransport it(shared_from_this());
        
        // 订阅相机图像
        image_sub_ = it.subscribe(
            camera_topic, 1,
            std::bind(&VisionNode::imageCallback, this, std::placeholders::_1));
        
        // 发布处理后的图像
        processed_pub_ = it.advertise(
            this->get_parameter("output_topic").as_string(), 1);
        
        // 发布检测结果
        detection_pub_ = this->create_publisher<std_msgs::msg::String>(
            "vision/detections", 10);
        
        RCLCPP_INFO(this->get_logger(), "视觉节点已启动");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            cv::Mat image = hexapod_vision::imageToMat(msg);
            cv::Mat processed = image.clone();
            
            // 图像预处理
            preprocessImage(image, processed);
            
            // 边缘检测
            if (this->get_parameter("enable_edge_detection").as_bool()) {
                detectEdges(processed);
            }
            
            // 颜色分割（用于识别作物）
            if (this->get_parameter("enable_color_segmentation").as_bool()) {
                segmentColors(processed);
            }
            
            // 检测物体
            std::vector<cv::Rect> detections = detectObjects(processed);
            
            // 在图像上绘制检测结果
            for (const auto& rect : detections) {
                cv::rectangle(processed, rect, cv::Scalar(0, 255, 0), 2);
            }
            
            sensor_msgs::msg::Image::SharedPtr processed_msg = 
                hexapod_vision::matToImage(processed, msg->header);
            processed_pub_.publish(processed_msg);
            
            // 发布检测结果
            if (!detections.empty()) {
                auto detection_msg = std_msgs::msg::String();
                detection_msg.data = "检测到 " + std::to_string(detections.size()) + " 个物体";
                detection_pub_->publish(detection_msg);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "图像处理异常: %s", e.what());
        }
    }
    
    void preprocessImage(const cv::Mat& input, cv::Mat& output)
    {
        // 高斯模糊去噪
        cv::GaussianBlur(input, output, cv::Size(5, 5), 0);
        
        // 对比度增强
        output.convertTo(output, -1, 1.2, 10);
    }
    
    void detectEdges(cv::Mat& image)
    {
        cv::Mat gray, edges;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::Canny(gray, edges, 50, 150);
        cv::cvtColor(edges, gray, cv::COLOR_GRAY2BGR);
        
        // 叠加边缘到原图
        cv::addWeighted(image, 0.7, gray, 0.3, 0, image);
    }
    
    void segmentColors(cv::Mat& image)
    {
        // 转换到HSV颜色空间
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        // 定义绿色范围（用于识别植物）
        cv::Scalar lower_green(40, 50, 50);
        cv::Scalar upper_green(80, 255, 255);
        
        // 创建掩码
        cv::Mat mask;
        cv::inRange(hsv, lower_green, upper_green, mask);
        
        // 应用掩码
        cv::Mat result;
        image.copyTo(result, mask);
        
        // 叠加结果
        cv::addWeighted(image, 0.6, result, 0.4, 0, image);
    }
    
    std::vector<cv::Rect> detectObjects(const cv::Mat& image)
    {
        std::vector<cv::Rect> detections;
        
        // 转换为灰度图
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        // 使用轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        
        cv::threshold(gray, gray, 127, 255, cv::THRESH_BINARY);
        cv::findContours(gray, contours, hierarchy, 
                        cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 过滤轮廓
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 500) {  // 最小面积阈值
                cv::Rect rect = cv::boundingRect(contour);
                detections.push_back(rect);
            }
        }
        
        return detections;
    }
    
    image_transport::Subscriber image_sub_;
    image_transport::Publisher processed_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}

