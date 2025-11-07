#include "terrain_adaptation.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

TerrainAdaptation::TerrainAdaptation()
    : analysis_radius_(0.5)
    , min_points_(10)
{
    terrain_points_.clear();
    current_score_ = {0.0, 0.0, 1.0, 1.0};
}

void TerrainAdaptation::updateTerrain(const sensor_msgs::msg::PointCloud2& cloud)
{
    terrain_points_.clear();
    
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : cloud.fields) {
        if (field.name == "x") x_offset = field.offset;
        else if (field.name == "y") y_offset = field.offset;
        else if (field.name == "z") z_offset = field.offset;
    }
    
    if (x_offset < 0) x_offset = 0;
    if (y_offset < 0) y_offset = 4;
    if (z_offset < 0) z_offset = 8;
    
    for (size_t i = 0; i < cloud.data.size(); i += cloud.point_step) {
        if (i + cloud.point_step > cloud.data.size()) break;
        
        if (i + z_offset + sizeof(float) <= cloud.data.size()) {
            std::array<float, 3> point;
            std::memcpy(&point[0], &cloud.data[i + x_offset], sizeof(float));
            std::memcpy(&point[1], &cloud.data[i + y_offset], sizeof(float));
            std::memcpy(&point[2], &cloud.data[i + z_offset], sizeof(float));
            terrain_points_.push_back(point);
        }
    }
    
    analyzeSlope();
    analyzeRoughness();
    analyzeReachability();
    calculateSafety();
}

void TerrainAdaptation::analyzeSlope()
{
    if (terrain_points_.size() < static_cast<size_t>(min_points_)) {
        current_score_.slope = 0.0;
        return;
    }
    
    float cx = 0.0f, cy = 0.0f, cz = 0.0f;
    for (const auto& p : terrain_points_) {
        cx += p[0];
        cy += p[1];
        cz += p[2];
    }
    cx /= terrain_points_.size();
    cy /= terrain_points_.size();
    cz /= terrain_points_.size();
    
    float max_z = cz, min_z = cz;
    for (const auto& p : terrain_points_) {
        max_z = std::max(max_z, p[2]);
        min_z = std::min(min_z, p[2]);
    }
    
    float height_diff = max_z - min_z;
    current_score_.slope = std::min(1.0f, height_diff / analysis_radius_);
}

void TerrainAdaptation::analyzeRoughness()
{
    if (terrain_points_.size() < static_cast<size_t>(min_points_)) {
        current_score_.roughness = 0.0;
        return;
    }
    
    float cx = 0.0f, cy = 0.0f, cz = 0.0f;
    for (const auto& p : terrain_points_) {
        cx += p[0];
        cy += p[1];
        cz += p[2];
    }
    cx /= terrain_points_.size();
    cy /= terrain_points_.size();
    cz /= terrain_points_.size();
    
    float variance = 0.0f;
    for (const auto& p : terrain_points_) {
        float dist = (p[0] - cx) * (p[0] - cx) +
                     (p[1] - cy) * (p[1] - cy) +
                     (p[2] - cz) * (p[2] - cz);
        variance += dist;
    }
    variance /= terrain_points_.size();
    
    float std_dev = std::sqrt(variance);
    current_score_.roughness = std::min(1.0f, std_dev / 0.1f);
}

void TerrainAdaptation::analyzeReachability()
{
    if (terrain_points_.size() < static_cast<size_t>(min_points_)) {
        current_score_.reachability = 0.0;
        return;
    }
    
    float density = static_cast<float>(terrain_points_.size()) / 
                    (analysis_radius_ * analysis_radius_ * M_PI);
    
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

