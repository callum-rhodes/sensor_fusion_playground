#pragma once
#include <cmath>
#include <numbers>

namespace sfsim {

// Wrap an angle to [-pi, pi]
inline double wrapAngle(double a) {
  return std::remainder(a, 2.0 * std::numbers::pi);
}

const float f_rad2deg = 180.0f / (float)M_PI;
const float f_deg2rad = (float)M_PI / 180.0f;

}  // namespace sfsim