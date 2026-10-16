#pragma once
#include <cmath>
#include <numbers>

namespace sfsim {

// Wrap an angle to [-pi, pi]
inline double wrapAngle(double a) {
  return std::remainder(a, 2.0 * std::numbers::pi);
}

}  // namespace sfsim