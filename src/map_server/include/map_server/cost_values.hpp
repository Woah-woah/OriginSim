#pragma once
#include <cstdint>

namespace map_server
{
inline constexpr uint8_t FREE_SPACE = 0;
inline constexpr uint8_t INSCRIBED_INFLATED_OBSTACLE = 253;
inline constexpr uint8_t LETHAL_OBSTACLE = 254;
inline constexpr uint8_t NO_INFORMATION = 255;
inline constexpr double NO_SPEED_LIMIT = 0.0;
}  // namespace map_server
