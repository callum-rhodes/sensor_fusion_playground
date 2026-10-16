#include "tracking/KalmanFilter.hpp"

namespace sfsim {

static Eigen::Matrix<double, 3, 6> H() {
  Eigen::Matrix<double, 3, 6> h = Eigen::Matrix<double, 3, 6>::Zero();
  h.leftCols<3>().setIdentity();
  return h;
}

void KalmanCV::predict(double dt, double q) {
  Mat6 F = Mat6::Identity();
  F.topRightCorner<3, 3>() = Mat3::Identity() * dt;
  // Discrete white-noise acceleration model
  Mat6 Q = Mat6::Zero();
  const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
  Q.topLeftCorner<3, 3>() = Mat3::Identity() * (dt4 / 4.0 * q);
  Q.topRightCorner<3, 3>() = Mat3::Identity() * (dt3 / 2.0 * q);
  Q.bottomLeftCorner<3, 3>() = Mat3::Identity() * (dt3 / 2.0 * q);
  Q.bottomRightCorner<3, 3>() = Mat3::Identity() * (dt2 * q);
  x = F * x;
  P = F * P * F.transpose() + Q;
}

double KalmanCV::gateDistance(const Vec3& z, const Mat3& R) const {
  const auto h = H();
  const Vec3 y = z - h * x;
  const Mat3 S = h * P * h.transpose() + R;
  return y.dot(S.ldlt().solve(y));
}

void KalmanCV::update(const Vec3& z, const Mat3& R) {
  const auto h = H();
  const Vec3 y = z - h * x;
  const Mat3 S = h * P * h.transpose() + R;
  const Eigen::Matrix<double, 6, 3> K = P * h.transpose() * S.inverse();
  x += K * y;
  const Mat6 I_KH = Mat6::Identity() - K * h;
  P = I_KH * P * I_KH.transpose() + K * R * K.transpose();  // Joseph form
}

}  // namespace sfsim
