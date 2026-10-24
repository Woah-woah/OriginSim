#include "map_server/static_map.hpp"
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <set>
#include <yaml-cpp/yaml.h>
namespace map_server
{
namespace
{
struct Image
{
  GridGeometry geometry;
  cv::Mat pixels;
  YAML::Node yaml;
};
Image read(const std::string &file)
{
  auto y = YAML::LoadFile(file);
  const auto origin = y["origin"].as<std::vector<double>>();
  if (origin.size() != 3 || !std::isfinite(origin[2]) || std::abs(origin[2]) > 1e-9)
    throw std::invalid_argument("Only axis-aligned map layers are supported: " + file);
  std::filesystem::path path = y["image"].as<std::string>();
  if (path.is_relative())
    path = std::filesystem::path(file).parent_path() / path;
  auto image = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
  if (image.empty() || image.channels() != 1)
    throw std::invalid_argument("Invalid single-channel layer: " + path.string());
  GridGeometry g{static_cast<uint32_t>(image.cols), static_cast<uint32_t>(image.rows),
                 y["resolution"].as<double>(), origin[0], origin[1]};
  g.validate();
  return {g, image, y};
}
size_t index(const cv::Mat &m, int x, int y)
{
  return static_cast<size_t>(m.rows - 1 - y) * m.cols + x;
}
} // namespace
StaticMap loadStaticMapFiles(const StaticMapFiles &files)
{
  auto in = read(files.occupancy_yaml);
  if (in.pixels.type() != CV_8UC1)
    throw std::invalid_argument("Occupancy image must be uint8");
  if (in.yaml["mode"] && in.yaml["mode"].as<std::string>() != "trinary")
    throw std::invalid_argument("Occupancy requires trinary mode");
  const double free = in.yaml["free_thresh"].as<double>(),
               occupied = in.yaml["occupied_thresh"].as<double>();
  const int negate = in.yaml["negate"].as<int>();
  if (!(free >= 0 && free < occupied && occupied <= 1) || (negate != 0 && negate != 1))
    throw std::invalid_argument("Invalid occupancy thresholds");
  StaticMap out;
  out.geometry = in.geometry;
  out.occupancy = {in.geometry, std::vector<uint8_t>(in.geometry.size())};
  out.region_id = {in.geometry, std::vector<RegionId>(in.geometry.size(), INVALID_REGION)};
  for (int y = 0; y < in.pixels.rows; ++y)
    for (int x = 0; x < in.pixels.cols; ++x)
    {
      double p = in.pixels.at<uint8_t>(y, x) / 255.0;
      if (!negate)
        p = 1 - p;
      // The planning grid is binary: ROS unknown/ambiguous cells remain blocked.
      out.occupancy.data[index(in.pixels, x, y)] = p < free ? 0 : 1;
    }
  if (!files.region_yaml.empty())
  {
    auto layer = read(files.region_yaml);
    if (!(layer.geometry == out.geometry) || layer.pixels.type() != CV_16UC1 ||
        !layer.yaml["labels"].IsSequence())
      throw std::invalid_argument("Region layer needs matching geometry, uint16 image and labels");
    std::set<RegionId> labels{0}, seen{0};
    for (const auto &label : layer.yaml["labels"])
    {
      const int id = label.as<int>();
      if (id <= 0 || id > 65535 || !labels.insert(static_cast<RegionId>(id)).second)
        throw std::invalid_argument("Invalid/duplicate region label");
    }
    for (int y = 0; y < layer.pixels.rows; ++y)
      for (int x = 0; x < layer.pixels.cols; ++x)
      {
        auto id = layer.pixels.at<uint16_t>(y, x);
        seen.insert(id);
        out.region_id.data[index(layer.pixels, x, y)] = id;
      }
    if (seen != labels)
      throw std::invalid_argument("Region labels do not match image labels");
  }
  const auto scalar = [&](const std::string &file) -> std::optional<Grid<float>>
  {
    if (file.empty())
      return {};
    auto layer = read(file);
    if (!(layer.geometry == out.geometry) || layer.pixels.type() != CV_32FC1)
      throw std::invalid_argument("Height/depth needs matching geometry and float32 image");
    Grid<float> result{out.geometry, std::vector<float>(out.geometry.size())};
    for (int y = 0; y < layer.pixels.rows; ++y)
      for (int x = 0; x < layer.pixels.cols; ++x)
      {
        float v = layer.pixels.at<float>(y, x);
        if (!std::isfinite(v))
          throw std::invalid_argument("Nonfinite height/depth");
        result.data[index(layer.pixels, x, y)] = v;
      }
    return result;
  };
  out.height = scalar(files.height_yaml);
  out.depth = scalar(files.depth_yaml);
  return out;
}
} // namespace map_server
