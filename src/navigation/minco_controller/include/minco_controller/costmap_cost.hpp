#pragma once

#include "minco_controller/trajectory_safety.hpp"

namespace minco_controller
{
// 兼容现有调用；碰撞几何只有 TrajectorySafety 一份实现。
using CostmapCost = TrajectorySafety;
}
