/**
 * 地形适应性模块
 * 分析地形并计算适应性评分
 */

#ifndef TERRAIN_ADAPTATION_HPP
#define TERRAIN_ADAPTATION_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <array>

struct TerrainScore {
    float slope;        // 坡度 (0-1)
    float roughness;    // 粗糙度 (0-1)
    float reachability; // 可达性 (0-1)
    float safety;       // 安全性 (0-1)
};

class TerrainAdaptation {
public:
    TerrainAdaptation();
    
    void updateTerrain(const sensor_msgs::msg::PointCloud2& cloud);
    TerrainScore getTerrainScore() const;
    
private:
    void analyzeSlope();
    void analyzeRoughness();
    void analyzeReachability();
    void calculateSafety();
    
    std::vector<std::array<float, 3>> terrain_points_;
    TerrainScore current_score_;
    
    // 分析参数
    float analysis_radius_;
    int min_points_;
};

#endif // TERRAIN_ADAPTATION_HPP

