// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0.
#pragma once
#include <cmath>
#include <iterator>

namespace mppi::path_geometry
{
template<typename Pose>
double euclidean_distance(const Pose & a, const Pose & b)
{
  const auto & p = a.pose.position; const auto & q = b.pose.position;
  return std::hypot(p.x - q.x, p.y - q.y);
}
template<typename Iterator>
Iterator first_after_integrated_distance(Iterator begin, Iterator end, double distance)
{
  if (begin == end) {return end;}
  double accumulated = 0.0;
  for (auto previous = begin, current = std::next(begin); current != end; ++previous, ++current) {
    accumulated += euclidean_distance(*previous, *current);
    if (accumulated > distance) {return current;}
  }
  return end;
}
template<typename Iterator, typename Score>
Iterator min_by(Iterator begin, Iterator end, Score score)
{
  if (begin == end) {return end;}
  auto best = begin; auto minimum = score(*best);
  for (auto current = std::next(begin); current != end; ++current) {
    const auto value = score(*current);
    if (value <= minimum) {minimum = value; best = current;}
  }
  return best;
}
}  // namespace mppi::path_geometry
