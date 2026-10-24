#include "map_server/edt_2d.hpp"
#include <algorithm>
#include <limits>
#include <opencv2/imgproc.hpp>
namespace map_server
{
DistanceGrid preciseEdt(const BinaryGrid &in)
{
  in.validate();
  DistanceGrid out{in.geometry, std::vector<float>(in.data.size(), std::hypot(in.geometry.width,
                                                                              in.geometry.height) *
                                                                       in.geometry.resolution)};
  if (std::any_of(in.data.begin(), in.data.end(), [](uint8_t v) { return v > 1; }))
    throw std::invalid_argument("Nonbinary obstacles");
  if (std::none_of(in.data.begin(), in.data.end(), [](uint8_t v) { return v == 1; }))
    return out;
  cv::Mat source(in.geometry.height, in.geometry.width, CV_8UC1);
  for (size_t i = 0; i < in.data.size(); ++i)
    source.data[i] = in.data[i] ? 0 : 255;
  cv::Mat distance;
  cv::distanceTransform(source, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  for (size_t i = 0; i < out.data.size(); ++i)
    out.data[i] = distance.ptr<float>()[i] * in.geometry.resolution;
  return out;
}
} // namespace map_server
