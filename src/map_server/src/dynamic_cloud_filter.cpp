#include "map_server/dynamic_cloud_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <pcl/common/point_tests.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/imgproc.hpp>

namespace map_server
{
DynamicCloudFilter::DynamicCloudFilter(
  const DynamicFilterOptions & options, const std::string & static_pcd) : options_(options)
{
  for (double value : {options.roi_min_radius, options.roi_max_radius, options.voxel_size,
    options.static_voxel_size, options.static_distance, options.sor_stddev,
    options.min_density, options.min_component_area, options.component_connection_distance})
  {if (!std::isfinite(value) || value < 0) {throw std::invalid_argument("Invalid dynamic filter setting");}}
  if (options.roi_max_radius <= options.roi_min_radius || options.roi_max_radius > 100 ||
    !std::isfinite(options.roi_min_z) || !std::isfinite(options.roi_max_z) ||
    options.roi_max_z <= options.roi_min_z || options.voxel_size <= 0 ||
    options.static_voxel_size <= 0 || options.static_distance <= 0 || options.sor_neighbors < 2 ||
    options.sor_neighbors > 1000 || options.min_density <= 0 ||
    options.component_connection_distance > 2.0 || static_pcd.empty())
  {throw std::invalid_argument("Invalid ROI, voxel, static PCD or SOR configuration");}
  DynamicCloud loaded;
  if (pcl::io::loadPCDFile(static_pcd, loaded) < 0)
  {throw std::runtime_error("Cannot load static PCD: " + static_pcd);}
  DynamicCloud finite;
  finite.reserve(loaded.size());
  for (const auto & point : loaded) {if (pcl::isFinite(point)) {finite.push_back(point);}}
  if (finite.empty()) {throw std::runtime_error("Static PCD contains no finite XYZ points");}
  static_cloud_ = voxel(finite, options_.static_voxel_size);
  if (static_cloud_->empty()) {throw std::runtime_error("Empty static PCD after Voxel filtering");}
  static_tree_.setInputCloud(static_cloud_);
}
bool DynamicCloudFilter::inRoi(const pcl::PointXYZ & point, const Eigen::Vector3d & robot) const
{
  if (!pcl::isFinite(point)) {return false;}
  const double radius = std::hypot(point.x - robot.x(), point.y - robot.y());
  const double z = point.z - robot.z();
  return radius >= options_.roi_min_radius && radius <= options_.roi_max_radius &&
         z >= options_.roi_min_z && z <= options_.roi_max_z;
}
DynamicCloud::Ptr DynamicCloudFilter::voxel(const DynamicCloud & cloud, double leaf) const
{
  auto result = std::make_shared<DynamicCloud>();
  if (cloud.empty()) {return result;}
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud.makeShared());
  filter.setLeafSize(leaf, leaf, leaf); filter.filter(*result);
  return result;
}
DynamicCloud::Ptr DynamicCloudFilter::prepare(const DynamicCloud & cloud,
  const Eigen::Isometry3d & tracking_from_cloud, const Eigen::Vector3d & robot) const
{
  auto cropped = std::make_shared<DynamicCloud>();
  cropped->reserve(cloud.size());
  for (const auto & point : cloud) {
    if (!pcl::isFinite(point)) {continue;}
    const Eigen::Vector3d value = tracking_from_cloud * point.getVector3fMap().cast<double>();
    pcl::PointXYZ transformed(value.x(), value.y(), value.z());
    if (inRoi(transformed, robot)) {cropped->push_back(transformed);}
  }
  return voxel(*cropped, options_.voxel_size);
}
DynamicDetection DynamicCloudFilter::detect(const DynamicCloud & accumulated,
  const DynamicCloud & fresh, const Eigen::Isometry3d & static_from_tracking,
  const Eigen::Vector3d & robot, const DynamicGrid & grid)
{
  DynamicCloud cropped;
  cropped.reserve(accumulated.size());
  for (const auto & point : accumulated) {if (inRoi(point, robot)) {cropped.push_back(point);}}
  DynamicDetection result;
  result.accumulated = voxel(cropped, options_.voxel_size);
  auto difference = std::make_shared<DynamicCloud>();
  std::vector<int> nearest(1);
  std::vector<float> distance(1);
  const auto is_dynamic = [&](const pcl::PointXYZ & point) {
      const Eigen::Vector3d value = static_from_tracking * point.getVector3fMap().cast<double>();
      const pcl::PointXYZ query(value.x(), value.y(), value.z());
      if (!pcl::isFinite(query)) {throw std::runtime_error("Nonfinite static PCD transform");}
      return static_tree_.nearestKSearch(query, 1, nearest, distance) != 1 ||
             distance[0] > options_.static_distance * options_.static_distance;
    };
  for (const auto & point : *result.accumulated) {
    if (is_dynamic(point)) {difference->push_back(point);}
  }
  result.filtered = std::make_shared<DynamicCloud>();
  if (difference->size() >= 3) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(difference);
    sor.setMeanK(std::min(options_.sor_neighbors, static_cast<int>(difference->size()) - 1));
    sor.setStddevMulThresh(options_.sor_stddev); sor.filter(*result.filtered);
  } else {
    // Fewer than three points cannot establish a statistical neighborhood.
    // Reject them rather than silently bypassing SOR.
    return result;
  }
  cv::Mat counts(grid.height, grid.width, CV_32SC1, cv::Scalar(0));
  cv::Mat mask(grid.height, grid.width, CV_8UC1, cv::Scalar(0));
  const auto cell_at = [&grid](const pcl::PointXYZ & point) {
      return DynamicCell{static_cast<int>(std::floor(point.x / grid.resolution)),
        static_cast<int>(std::floor(point.y / grid.resolution))};
    };
  for (const auto & point : *result.filtered) {
    const auto cell = cell_at(point);
    if (grid.contains(cell)) {++counts.at<int>(cell.y - grid.y, cell.x - grid.x);}
  }
  const double density_count = std::ceil(options_.min_density * grid.resolution * grid.resolution);
  for (int y = 0; y < counts.rows; ++y) {
    for (int x = 0; x < counts.cols; ++x) {
      if (counts.at<int>(y, x) >= density_count) {mask.at<uint8_t>(y, x) = 255;}
    }
  }
  cv::Mat connectivity;
  if (options_.component_connection_distance > 0) {
    // Registration may use a coarser voxel than the navigation grid. Connect
    // nearby measured cells for component filtering, but never render the
    // dilation as obstacle geometry or count its unmeasured area.
    const int radius = std::ceil(options_.component_connection_distance / (2 * grid.resolution));
    cv::dilate(mask, connectivity, cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * radius + 1, 2 * radius + 1)));
  } else {connectivity = mask;}
  cv::Mat labels, stats, centers;
  cv::connectedComponentsWithStats(connectivity, labels, stats, centers, 8, CV_32S);
  std::vector<unsigned int> measured_cells(stats.rows, 0);
  for (int y = 0; y < mask.rows; ++y) {
    for (int x = 0; x < mask.cols; ++x) {
      if (mask.at<uint8_t>(y, x)) {++measured_cells[labels.at<int>(y, x)];}
    }
  }
  const double min_cells = std::ceil(options_.min_component_area / (grid.resolution * grid.resolution));
  // Accumulated old samples help shape/density but never refresh a cell's memory
  // unless there is independently measured dynamic support in this newest frame.
  std::unordered_map<DynamicCell, bool, DynamicCellHash> supported;
  for (const auto & point : fresh) {
    if (!inRoi(point, robot)) {continue;}
    const auto cell = cell_at(point);
    if (!grid.contains(cell) || !mask.at<uint8_t>(cell.y - grid.y, cell.x - grid.x)) {continue;}
    const int label = labels.at<int>(cell.y - grid.y, cell.x - grid.x);
    if (label <= 0 || measured_cells[label] < min_cells || supported.count(cell))
    {continue;}
    // Only cells that survived every filter need fresh support queries; one
    // matching fresh point suffices, so avoid repeated searches in that cell.
    if (is_dynamic(point)) {supported[cell] = true;}
  }
  result.cells.reserve(supported.size());
  for (const auto & cell : supported) {result.cells.push_back(cell.first);}
  return result;
}
}  // namespace map_server
