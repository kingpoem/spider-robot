/**
 * 目标检测节点
 * 使用深度学习或传统方法检测农作物、障碍物等
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <memory>
#include <vector>

struct Detection {
    std::string label;
    float confidence;
    cv::Rect bbox;
    geometry_msgs::msg::Point position;
};

class ObjectDetectorNode : public rclcpp::Node
{
public:
    ObjectDetectorNode() : Node("object_detector")
    {
        // 声明参数
        this->declare_parameter("camera_topic", "camera/image_raw");
        this->declare_parameter("detection_model_path", "");
        this->declare_parameter("confidence_threshold", 0.5);
        this->declare_parameter("detect_crops", true);
        this->declare_parameter("detect_obstacles", true);
        
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        
        // 图像传输
        image_transport::ImageTransport it(shared_from_this());
        
        // 订阅图像
        image_sub_ = it.subscribe(
            camera_topic, 1,
            std::bind(&ObjectDetectorNode::imageCallback, this, std::placeholders::_1));
        
        // 发布检测结果
        detection_pub_ = this->create_publisher<std_msgs::msg::String>(
            "vision/detected_objects", 10);
        
        // 发布检测图像
        result_pub_ = it.advertise("vision/detection_result", 1);
        
        // 初始化检测器
        initializeDetector();
        
        RCLCPP_INFO(this->get_logger(), "目标检测节点已启动");
    }

private:
    void initializeDetector()
    {
        std::string model_path = this->get_parameter("detection_model_path").as_string();
        
        if (!model_path.empty()) {
            // 加载深度学习模型（如果提供）
            try {
                // 这里可以加载YOLO、SSD等模型
                // net_ = cv::dnn::readNetFromDarknet(config_path, weights_path);
                RCLCPP_INFO(this->get_logger(), "尝试加载检测模型: %s", model_path.c_str());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "无法加载模型，使用传统方法: %s", e.what());
            }
        }
        
        // 初始化颜色检测器（用于识别绿色作物）
        setupColorDetector();
    }
    
    void setupColorDetector()
    {
        // 绿色范围（HSV）- 用于识别植物
        green_lower_ = cv::Scalar(40, 50, 50);
        green_upper_ = cv::Scalar(80, 255, 255);
        
        // 红色范围 - 用于识别成熟果实
        red_lower1_ = cv::Scalar(0, 50, 50);
        red_upper1_ = cv::Scalar(10, 255, 255);
        red_lower2_ = cv::Scalar(170, 50, 50);
        red_upper2_ = cv::Scalar(180, 255, 255);
    }
    
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                msg, sensor_msgs::image_encodings::BGR8);
            
            cv::Mat image = cv_ptr->image;
            std::vector<Detection> detections;
            
            // 执行检测
            if (this->get_parameter("detect_crops").as_bool()) {
                detectCrops(image, detections);
            }
            
            if (this->get_parameter("detect_obstacles").as_bool()) {
                detectObstacles(image, detections);
            }
            
            // 绘制检测结果
            cv::Mat result = image.clone();
            drawDetections(result, detections);
            
            // 发布结果图像
            sensor_msgs::msg::Image::SharedPtr result_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", result).toImageMsg();
            result_pub_.publish(result_msg);
            
            // 发布检测信息
            if (!detections.empty()) {
                std::string detection_info = formatDetections(detections);
                auto msg_str = std_msgs::msg::String();
                msg_str.data = detection_info;
                detection_pub_->publish(msg_str);
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        }
    }
    
    void detectCrops(const cv::Mat& image, std::vector<Detection>& detections)
    {
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        // 检测绿色植物
        cv::Mat green_mask;
        cv::inRange(hsv, green_lower_, green_upper_, green_mask);
        
        // 形态学操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, kernel);
        
        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 1000) {  // 最小面积
                cv::Rect bbox = cv::boundingRect(contour);
                
                Detection det;
                det.label = "crop";
                det.confidence = std::min(1.0f, static_cast<float>(area / 10000.0));
                det.bbox = bbox;
                det.position.x = bbox.x + bbox.width / 2.0;
                det.position.y = bbox.y + bbox.height / 2.0;
                det.position.z = 0.0;
                
                detections.push_back(det);
            }
        }
        
        // 检测红色成熟果实
        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv, red_lower1_, red_upper1_, red_mask1);
        cv::inRange(hsv, red_lower2_, red_upper2_, red_mask2);
        cv::bitwise_or(red_mask1, red_mask2, red_mask);
        
        cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
        cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 500) {
                cv::Rect bbox = cv::boundingRect(contour);
                
                Detection det;
                det.label = "ripe_fruit";
                det.confidence = std::min(1.0f, static_cast<float>(area / 5000.0));
                det.bbox = bbox;
                det.position.x = bbox.x + bbox.width / 2.0;
                det.position.y = bbox.y + bbox.height / 2.0;
                det.position.z = 0.0;
                
                detections.push_back(det);
            }
        }
    }
    
    void detectObstacles(const cv::Mat& image, std::vector<Detection>& detections)
    {
        // 转换为灰度图
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        // 使用边缘检测
        cv::Mat edges;
        cv::Canny(gray, edges, 50, 150);
        
        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 2000) {
                cv::Rect bbox = cv::boundingRect(contour);
                
                // 简单的几何特征判断
                double extent = area / (bbox.width * bbox.height);
                if (extent > 0.3) {
                    Detection det;
                    det.label = "obstacle";
                    det.confidence = 0.7f;
                    det.bbox = bbox;
                    det.position.x = bbox.x + bbox.width / 2.0;
                    det.position.y = bbox.y + bbox.height / 2.0;
                    det.position.z = 0.0;
                    
                    detections.push_back(det);
                }
            }
        }
    }
    
    void drawDetections(cv::Mat& image, const std::vector<Detection>& detections)
    {
        for (const auto& det : detections) {
            // 绘制边界框
            cv::Scalar color = (det.label == "crop") ? cv::Scalar(0, 255, 0) :
                              (det.label == "ripe_fruit") ? cv::Scalar(0, 0, 255) :
                              cv::Scalar(255, 0, 0);
            
            cv::rectangle(image, det.bbox, color, 2);
            
            // 绘制标签
            std::string label_text = det.label + ": " + 
                                    std::to_string(static_cast<int>(det.confidence * 100)) + "%";
            cv::putText(image, label_text, 
                       cv::Point(det.bbox.x, det.bbox.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
    }
    
    std::string formatDetections(const std::vector<Detection>& detections)
    {
        std::string result = "检测到 " + std::to_string(detections.size()) + " 个物体: ";
        for (size_t i = 0; i < detections.size(); ++i) {
            result += detections[i].label;
            if (i < detections.size() - 1) result += ", ";
        }
        return result;
    }
    
    image_transport::Subscriber image_sub_;
    image_transport::Publisher result_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_pub_;
    
    cv::Scalar green_lower_, green_upper_;
    cv::Scalar red_lower1_, red_upper1_, red_lower2_, red_upper2_;
    cv::dnn::Net net_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectorNode>());
    rclcpp::shutdown();
    return 0;
}

