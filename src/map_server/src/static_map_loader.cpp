#include "map_server/static_map_loader.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

namespace map_server
{
nav_msgs::msg::OccupancyGrid loadStaticMap(const std::string & filename)
{
  const auto yaml = YAML::LoadFile(filename);
  const double resolution = yaml["resolution"].as<double>();
  const auto origin = yaml["origin"].as<std::vector<double>>();
  const double occupied = yaml["occupied_thresh"].as<double>();
  const double free = yaml["free_thresh"].as<double>();
  const int negate = yaml["negate"].as<int>();
  const std::string mode = yaml["mode"] ? yaml["mode"].as<std::string>() : "trinary";
  if (!std::isfinite(resolution) || resolution <= 0 || origin.size() != 3 ||
    !std::isfinite(origin[0]) || !std::isfinite(origin[1]) || !std::isfinite(origin[2]) ||
    !(free >= 0 && free < occupied && occupied <= 1) || (negate != 0 && negate != 1) ||
    (mode != "trinary" && mode != "scale" && mode != "raw"))
  {throw std::invalid_argument("Invalid map YAML geometry, thresholds, negate or mode");}
  auto path = std::filesystem::path(yaml["image"].as<std::string>());
  if (path.is_relative()) {path = std::filesystem::path(filename).parent_path() / path;}
  const auto image = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
  if (image.empty() || image.depth() != CV_8U ||
    (image.channels() != 1 && image.channels() != 3 && image.channels() != 4) ||
    static_cast<size_t>(image.rows) * image.cols > 100000000)
  {throw std::runtime_error("Cannot load 8-bit map image: " + path.string());}
  nav_msgs::msg::OccupancyGrid map;
  map.info.resolution = resolution;
  map.info.width = image.cols; map.info.height = image.rows;
  map.info.origin.position.x = origin[0]; map.info.origin.position.y = origin[1];
  map.info.origin.orientation.z = std::sin(origin[2] / 2);
  map.info.origin.orientation.w = std::cos(origin[2] / 2);
  map.data.resize(static_cast<size_t>(image.rows) * image.cols);
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      const auto * pixel = image.ptr<uint8_t>(y) + x * image.channels();
      const int channels = std::min(image.channels(), 3);
      double gray = 0;
      for (int c = 0; c < channels; ++c) {gray += pixel[c];}
      gray /= channels;
      const double probability = (negate ? gray : 255.0 - gray) / 255.0;
      int value = -1;
      if (mode == "raw") {value = gray <= 100 ? static_cast<int>(gray) : -1;}
      else if (probability > occupied) {value = 100;}
      else if (probability < free) {value = 0;}
      else if (mode == "scale") {
        value = static_cast<int>(std::round(100 * (probability - free) / (occupied - free)));
      }
      if (image.channels() == 4 && pixel[3] < 255) {value = -1;}
      map.data[static_cast<size_t>(image.rows - y - 1) * image.cols + x] = value;
    }
  }
  return map;
}
}  // namespace map_server
