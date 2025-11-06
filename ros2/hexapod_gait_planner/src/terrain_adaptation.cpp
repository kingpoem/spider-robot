#include "terrain_adaptation.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstring>

// 手动实现点云转换（ROS2中pcl_conversions可能不可用）
// 这是一个简化版本，仅支持标准的PointXYZ格式
namespace pcl {
    inline void fromROSMsg(const sensor_msgs::msg::PointCloud2& cloud, pcl::PointCloud<pcl::PointXYZ>& pcl_cloud) {
        pcl_cloud.width = cloud.width;
        pcl_cloud.height = cloud.height;
        pcl_cloud.is_dense = cloud.is_dense;
        pcl_cloud.points.clear();
        pcl_cloud.points.reserve(cloud.width * cloud.height);
        
        // 查找x, y, z字段的偏移量
        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (const auto& field : cloud.fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
        }
        
        // 如果找不到标准字段，使用默认偏移量
        if (x_offset < 0) x_offset = 0;
        if (y_offset < 0) y_offset = 4;
        if (z_offset < 0) z_offset = 8;
        
        // 解析点云数据
        for (size_t i = 0; i < cloud.data.size(); i += cloud.point_step) {
            if (i + cloud.point_step > cloud.data.size()) break;
            
            pcl::PointXYZ point;
            if (i + z_offset + sizeof(float) <= cloud.data.size()) {
                std::memcpy(&point.x, &cloud.data[i + x_offset], sizeof(float));
                std::memcpy(&point.y, &cloud.data[i + y_offset], sizeof(float));
                std::memcpy(&point.z, &cloud.data[i + z_offset], sizeof(float));
                pcl_cloud.points.push_back(point);
            }
        }
        
        if (pcl_cloud.height == 0) pcl_cloud.height = 1;
        if (pcl_cloud.width == 0) pcl_cloud.width = pcl_cloud.points.size();
    }
}

TerrainAdaptation::TerrainAdaptation()
    : analysis_radius_(0.5)
    , min_points_(10)
{
    terrain_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    current_score_ = {0.0, 0.0, 1.0, 1.0};
}

void TerrainAdaptation::updateTerrain(const sensor_msgs::msg::PointCloud2& cloud)
{
    // 转换点云
    pcl::fromROSMsg(cloud, *terrain_cloud_);
    
    // 分析地形
    analyzeSlope();
    analyzeRoughness();
    analyzeReachability();
    calculateSafety();
}

void TerrainAdaptation::analyzeSlope()
{
    if (terrain_cloud_->points.size() < static_cast<size_t>(min_points_)) {
        current_score_.slope = 0.0;
        return;
    }
    
    // 计算点云的法向量（使用PCA）
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*terrain_cloud_, centroid);
    
    // 计算协方差矩阵
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrix(*terrain_cloud_, centroid, covariance);
    
    // 计算特征值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();
    
    // 最小特征值对应的特征向量是法向量（第一列对应最小特征值）
    Eigen::Vector3f normal = eigenvectors.col(0);
    
    // 计算坡度（法向量与z轴的夹角）
    float cos_angle = std::abs(normal.z());
    float slope_angle = acos(cos_angle);
    
    // 归一化到0-1
    current_score_.slope = std::min(1.0f, static_cast<float>(slope_angle / (M_PI / 2.0)));
}

void TerrainAdaptation::analyzeRoughness()
{
    if (terrain_cloud_->points.size() < static_cast<size_t>(min_points_)) {
        current_score_.roughness = 0.0;
        return;
    }
    
    // 计算点云的标准差
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*terrain_cloud_, centroid);
    
    float variance = 0.0;
    for (const auto& point : terrain_cloud_->points) {
        float dist = (point.x - centroid.x()) * (point.x - centroid.x()) +
                     (point.y - centroid.y()) * (point.y - centroid.y()) +
                     (point.z - centroid.z()) * (point.z - centroid.z());
        variance += dist;
    }
    variance /= terrain_cloud_->points.size();
    
    float std_dev = sqrt(variance);
    
    // 归一化到0-1（假设标准差大于0.1m为很粗糙）
    current_score_.roughness = std::min(1.0f, std_dev / 0.1f);
}

void TerrainAdaptation::analyzeReachability()
{
    // 检查是否有足够的支撑点
    // 简化实现：检查点云密度
    
    if (terrain_cloud_->points.size() < static_cast<size_t>(min_points_)) {
        current_score_.reachability = 0.0;
        return;
    }
    
    // 计算点云密度
    float density = static_cast<float>(terrain_cloud_->points.size()) / 
                    (analysis_radius_ * analysis_radius_ * M_PI);
    
    // 归一化到0-1
    current_score_.reachability = std::min(1.0f, density / 100.0f);
}

void TerrainAdaptation::calculateSafety()
{
    // 综合评分
    // 坡度越大、粗糙度越大，安全性越低
    current_score_.safety = 1.0 - (current_score_.slope * 0.4 + current_score_.roughness * 0.6);
    current_score_.safety = std::max(0.0f, std::min(1.0f, current_score_.safety));
}

TerrainScore TerrainAdaptation::getTerrainScore() const
{
    return current_score_;
}

