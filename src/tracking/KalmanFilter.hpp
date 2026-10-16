#pragma once
#include "core/Types.hpp"

namespace sfsim {

// Linear constant-velocity Kalman filter, state [x y z vx vy vz]. Position-only measurements.
struct KalmanCV {
  Vec6 x = Vec6::Zero();
  Mat6 P = Mat6::Identity();

  void predict(double dt, double q_accel);
  // Returns the Mahalanobis distance squared of the innovation
  double gateDistance(const Vec3& z, const Mat3& R) const;
  void update(const Vec3& z, const Mat3& R);
};

}  // namespace sfsim
