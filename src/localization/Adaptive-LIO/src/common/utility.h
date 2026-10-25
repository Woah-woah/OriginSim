#ifndef UTILITY_H_
#define UTILITY_H_
// c++
#include <iostream>
#include <string>
#include <tr1/unordered_map>

#include "cloudMap.hpp"

#include "sophus/so3.hpp"

double AngularDistance(const Eigen::Vector3d &qa, const Eigen::Vector3d &qb);

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b);

void sub_sample_frame(std::vector<point3D> &frame, double size_voxel);

void grid_sampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling);

void subSampleFrame(std::vector<point3D> &frame, double size_voxel);

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling);

// end_point is already in the IMU frame at frame end; no alpha interpolation here.
void transformPoint(point3D& point, const Eigen::Quaterniond& rotation,
                    const Eigen::Vector3d& translation);

#if __cplusplus < 201402L
// provide make_unique for pre-C++14 compilers (but don't inject into namespace std)
namespace zjloc {
     template <typename T, typename... Args>
     std::unique_ptr<T> make_unique(Args &&...args)
     {
          return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
     }
}
#define zjloc_make_unique zjloc::make_unique
#endif
#endif
