#include "utility.h"

#include <algorithm>
#include <cmath>

double AngularDistance(const Eigen::Matrix3d &rota, const Eigen::Matrix3d &rotb)
{
     double cos_angle = ((rota * rotb.transpose()).trace() - 1) / 2;
     cos_angle = std::clamp(cos_angle, -1.0, 1.0);
     return std::acos(cos_angle) * 180 / M_PI;
}

double AngularDistance(const Eigen::Vector3d &qa, const Eigen::Vector3d &qb)
{
     Eigen::Quaterniond q_a = Eigen::Quaterniond(Sophus::SO3d::exp(qa).matrix());
     Eigen::Quaterniond q_b = Eigen::Quaterniond(Sophus::SO3d::exp(qb).matrix());
     q_a.normalize(), q_b.normalize();

     Eigen::Matrix3d rota = q_a.toRotationMatrix();
     Eigen::Matrix3d rotb = q_b.toRotationMatrix();

     double cos_angle = ((rota * rotb.transpose()).trace() - 1) / 2;
     cos_angle = std::clamp(cos_angle, -1.0, 1.0);
     return std::acos(cos_angle) * 180 / M_PI;
}

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b)
{
     Eigen::Quaterniond qa = q_a;
     Eigen::Quaterniond qb = q_b;
     if (qa.norm() < 1e-12 || qb.norm() < 1e-12)
     {
          return 0.0;
     }
     qa.normalize();
     qb.normalize();

     Eigen::Matrix3d rota = qa.toRotationMatrix();
     Eigen::Matrix3d rotb = qb.toRotationMatrix();

     double cos_angle = ((rota * rotb.transpose()).trace() - 1) / 2;
     cos_angle = std::clamp(cos_angle, -1.0, 1.0);
     return std::acos(cos_angle) * 180 / M_PI;
}

void sub_sample_frame(std::vector<point3D> &frame, double size_voxel)
{
     tsl::robin_map<voxel, point3D> grid;
     grid.reserve(size_t(frame.size() / 4.));
     voxel vox;
     for (int i = 0; i < (int)frame.size(); i++)
     {
          vox.x = static_cast<short>(frame[i].point[0] / size_voxel);
          vox.y = static_cast<short>(frame[i].point[1] / size_voxel);
          vox.z = static_cast<short>(frame[i].point[2] / size_voxel);
          if (grid.find(vox) == grid.end())
               grid[vox] = frame[i];
     }
     frame.resize(0);
     frame.reserve(grid.size());
     for (const auto &[_, point] : grid)
     {
          frame.push_back(point);
     }
}

void grid_sampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling)
{
     keypoints.clear();
     if (frame.empty())
     {
          return;
     }

     // Direct voxel sampling avoids copying the whole input frame before downsampling.
     // Semantics stay the same as sub_sample_frame(): one representative point per voxel.
     tsl::robin_map<voxel, size_t> selected;
     selected.reserve(std::max<size_t>(frame.size() / 4, 1));

     voxel vox;
     for (size_t i = 0; i < frame.size(); ++i)
     {
          vox.x = static_cast<short>(frame[i].point[0] / size_voxel_subsampling);
          vox.y = static_cast<short>(frame[i].point[1] / size_voxel_subsampling);
          vox.z = static_cast<short>(frame[i].point[2] / size_voxel_subsampling);
          if (selected.find(vox) == selected.end())
          {
               selected[vox] = i;
          }
     }

     keypoints.reserve(selected.size());
     for (const auto &[_, idx] : selected)
     {
          keypoints.push_back(frame[idx]);
     }
}


void subSampleFrame(std::vector<point3D> &frame, double size_voxel)
{
     std::tr1::unordered_map<voxel, std::vector<point3D>, std::hash<voxel>> grid;
     grid.rehash(frame.size());
     for (int i = 0; i < (int)frame.size(); i++)
     {
          auto kx = static_cast<short>(frame[i].point[0] / size_voxel);
          auto ky = static_cast<short>(frame[i].point[1] / size_voxel);
          auto kz = static_cast<short>(frame[i].point[2] / size_voxel);
          grid[voxel(kx, ky, kz)].push_back(frame[i]);
     }
     frame.clear();
     frame.reserve(grid.size());
     int step = 0;
     for (const auto &n : grid)
     {
          if (n.second.size() > 0)
          {
               frame.push_back(n.second[0]);
               step++;
          }
     }
}

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling)
{
     keypoints.clear();
     std::vector<point3D> frame_sub(frame);
     subSampleFrame(frame_sub, size_voxel_subsampling);
     keypoints = std::move(frame_sub);
}

void transformPoint(point3D& point, const Eigen::Quaterniond& rotation,
                    const Eigen::Vector3d& translation)
{
     point.point = rotation * point.end_point + translation;
}
