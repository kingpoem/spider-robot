#ifndef HEXAPOD_VISION_IMAGE_CONVERTER_HPP
#define HEXAPOD_VISION_IMAGE_CONVERTER_HPP

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <cstring>

namespace hexapod_vision {

cv::Mat imageToMat(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (msg->encoding != "bgr8" && msg->encoding != "rgb8" && 
        msg->encoding != "mono8" && msg->encoding != "8UC3" && 
        msg->encoding != "8UC1") {
        throw std::runtime_error("不支持的图像编码: " + msg->encoding);
    }
    
    int cv_type = CV_8UC1;
    
    if (msg->encoding == "bgr8" || msg->encoding == "rgb8" || msg->encoding == "8UC3") {
        cv_type = CV_8UC3;
    }
    
    cv::Mat mat(msg->height, msg->width, cv_type);
    std::memcpy(mat.data, msg->data.data(), msg->data.size());
    
    if (msg->encoding == "rgb8") {
        cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    }
    
    return mat;
}

sensor_msgs::msg::Image::SharedPtr matToImage(const cv::Mat& mat, const std_msgs::msg::Header& header) {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header = header;
    msg->height = mat.rows;
    msg->width = mat.cols;
    
    if (mat.type() == CV_8UC1) {
        msg->encoding = "mono8";
        msg->is_bigendian = false;
        msg->step = mat.cols;
    } else if (mat.type() == CV_8UC3) {
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step = mat.cols * 3;
    } else {
        throw std::runtime_error("不支持的Mat类型");
    }
    
    msg->data.resize(msg->step * msg->height);
    std::memcpy(msg->data.data(), mat.data, msg->data.size());
    
    return msg;
}

} // namespace hexapod_vision

#endif // HEXAPOD_VISION_IMAGE_CONVERTER_HPP

