#pragma once
#include "algo/lidar_geometry_quality.h"
#include <array>
#include <map>
#include <set>
namespace zjloc {
using CoverageBin=std::array<int,6>;
inline CoverageBin CorrespondenceBin(const LidarCorrespondence& c) {
    Vec3d n=c.normal_world;Eigen::Index dominant;n.cwiseAbs().maxCoeff(&dominant);
    if(n[dominant]<0)n=-n; // n and -n are the same plane orientation
    CoverageBin bin;
    for(int k=0;k<3;++k){bin[k]=static_cast<int>(std::round(2*n[k]));
        // Bounded spatial octants around the IMU reference origin.
        bin[k+3]=c.end_imu[k]<0?-1:1;}
    return bin;
}
struct SamplingCoverage {size_t candidates=0,retained=0,normal_bins=0,spatial_bins=0;Vec3d dominant_normal_counts=Vec3d::Zero();};
inline std::vector<size_t> SelectGeometricCoverage(const std::vector<LidarCorrespondence>& candidates,
                                                 size_t budget,SamplingCoverage& coverage) {
    std::map<CoverageBin,std::vector<size_t>> bins;
    for(size_t i=0;i<candidates.size();++i)bins[CorrespondenceBin(candidates[i])].push_back(i);
    std::vector<size_t> selected;selected.reserve(std::min(budget,candidates.size()));
    for(size_t round=0;selected.size()<budget;++round) {
        bool added=false;
        for(const auto& entry:bins){if(round>=entry.second.size())continue;
            selected.push_back(entry.second[round]);added=true;if(selected.size()==budget)break;}
        if(!added)break;
    }
    coverage=SamplingCoverage{};coverage.candidates=candidates.size();coverage.retained=selected.size();
    std::set<std::array<int,3>> normals,spatial;
    for(size_t i:selected){auto b=CorrespondenceBin(candidates[i]);
        normals.insert({b[0],b[1],b[2]});spatial.insert({b[3],b[4],b[5]});
        Eigen::Index axis;candidates[i].normal_world.cwiseAbs().maxCoeff(&axis);++coverage.dominant_normal_counts[axis];}
    coverage.normal_bins=normals.size();coverage.spatial_bins=spatial.size();return selected;
}
} // namespace zjloc
