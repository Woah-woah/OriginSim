#pragma once

#include <string>
#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "map_server/dynamic_memory.hpp"

namespace map_server
{
using DynamicCloud = pcl::PointCloud<pcl::PointXYZ>;
struct DynamicFilterOptions
{
  double roi_min_radius{0.4}, roi_max_radius{8.0};
  double roi_min_z{-0.45}, roi_max_z{0.8};
  double voxel_size{0.03}, static_voxel_size{0.05}, static_distance{0.2};
  int sor_neighbors{12};
  double sor_stddev{2.0}, min_density{400.0}, min_component_area{0.0075};
  double component_connection_distance{0.55};
};
struct DynamicDetection
{
  std::vector<DynamicCell> cells;
  DynamicCloud::Ptr accumulated;
  DynamicCloud::Ptr filtered;
};
class DynamicCloudFilter
{
public:
  DynamicCloudFilter(const DynamicFilterOptions & options, const std::string & static_pcd);
  DynamicCloud::Ptr prepare(const DynamicCloud & cloud,
    const Eigen::Isometry3d & tracking_from_cloud, const Eigen::Vector3d & robot) const;
  DynamicDetection detect(const DynamicCloud & accumulated, const DynamicCloud & fresh,
    const Eigen::Isometry3d & static_from_tracking, const Eigen::Vector3d & robot,
    const DynamicGrid & grid);
  size_t staticSize() const {return static_cloud_->size();}
private:
  bool inRoi(const pcl::PointXYZ & point, const Eigen::Vector3d & robot) const;
  DynamicCloud::Ptr voxel(const DynamicCloud & cloud, double leaf) const;
  DynamicFilterOptions options_;
  DynamicCloud::Ptr static_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> static_tree_;
};
}  // namespace map_server
