#include "terrain_adaptation.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <algorithm>
#include <cmath>

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
    if (terrain_cloud_->points.size() < min_points_) {
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
    Eigen::Vector3f eigenvalues = solver.eigenvalues();
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();
    
    // 最小特征值对应的特征向量是法向量
    Eigen::Vector3f normal = eigenvectors.col(0);
    
    // 计算坡度（法向量与z轴的夹角）
    float cos_angle = std::abs(normal.z());
    float slope_angle = acos(cos_angle);
    
    // 归一化到0-1
    current_score_.slope = std::min(1.0f, slope_angle / (M_PI / 2));
}

void TerrainAdaptation::analyzeRoughness()
{
    if (terrain_cloud_->points.size() < min_points_) {
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
    
    if (terrain_cloud_->points.size() < min_points_) {
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

