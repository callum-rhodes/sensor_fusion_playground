#include "core/MeasurementModels.hpp"
#include "core/maths.hpp"  // wrapAngle
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sfsim {
namespace {

// Clamps that keep Jacobians finite at the singularities (metres, or whatever
// your world unit is). Below kSingular* the geometry is treated as degenerate.
constexpr double kClampRange = 1e-6;
constexpr double kClampRho = 1e-6;
constexpr double kSingularRange = 1e-3;
constexpr double kSingularRho = 1e-3;

// Rotation taking sensor-frame vectors to world frame.
// ONE place to adapt if SensorPose stores orientation differently
// (e.g. a yaw angle: return Eigen::AngleAxisd(pose.yaw, Vec3::UnitZ()).toRotationMatrix();).
Mat3 sensorToWorld(const SensorPose& pose) {
  return pose.orientation.toRotationMatrix();
}

}  // namespace

namespace rae {

Vec3 fromCartesian(const Vec3& p_world, const SensorPose& pose) {
  const Vec3 p = sensorToWorld(pose).transpose() * (p_world - pose.position);
  const double rho = std::hypot(p.x(), p.y());
  // atan2 forms are well defined everywhere, including the origin and the vertical axis
  return {p.norm(), std::atan2(p.y(), p.x()), std::atan2(p.z(), rho)};
}

Mat3 fromCartesianJacobian(const Vec3& p_world, const SensorPose& pose) {
  const Mat3 R_ws = sensorToWorld(pose);
  const Vec3 p = R_ws.transpose() * (p_world - pose.position);

  const double x = p.x(), y = p.y(), z = p.z();
  const double r = std::max(p.norm(), kClampRange);
  const double rho = std::max(std::hypot(x, y), kClampRho);
  const double r2 = r * r, rho2 = rho * rho;

  Mat3 J_s;  // d(r, az, el) / d(sensor-frame position)
  J_s << x / r,                 y / r,                 z / r,
         -y / rho2,             x / rho2,              0.0,
         -x * z / (r2 * rho),   -y * z / (r2 * rho),   rho / r2;

  // chain rule: p_sensor = R_ws^T (p_world - position)
  return J_s * R_ws.transpose();
}

Vec3 toCartesian(const Vec3& z, const SensorPose& pose) {
  const double r = z(0), az = z(1), el = z(2);
  const double ce = std::cos(el), se = std::sin(el);
  const Vec3 p_sensor(r * ce * std::cos(az), r * ce * std::sin(az), r * se);
  return pose.position + sensorToWorld(pose) * p_sensor;
}

Mat3 toCartesianJacobian(const Vec3& z, const SensorPose& pose) {
  const double r = z(0), az = z(1), el = z(2);
  const double ce = std::cos(el), se = std::sin(el);
  const double ca = std::cos(az), sa = std::sin(az);

  Mat3 J_s;  // d(sensor-frame position) / d(r, az, el)
  J_s << ce * ca,   -r * ce * sa,   -r * se * ca,
         ce * sa,    r * ce * ca,   -r * se * sa,
         se,         0.0,            r * ce;

  return sensorToWorld(pose) * J_s;
}

Mat3 toCartesianCov(const Vec3& z, const Mat3& R, const SensorPose& pose) {
  const Mat3 J = toCartesianJacobian(z, pose);
  return J * R * J.transpose();
}

bool nearSingularity(const Vec3& p_world, const SensorPose& pose) {
  const Vec3 p = sensorToWorld(pose).transpose() * (p_world - pose.position);
  return p.norm() < kSingularRange || std::hypot(p.x(), p.y()) < kSingularRho;
}

}  // namespace rae

Vec3 predictMeasurement(const Vec6& x, const Measurement& m) {
  switch (m.type) {
    case MeasType::Cartesian:
      return x.head<3>();
    case MeasType::RangeAzEl:
      return rae::fromCartesian(x.head<3>(), m.pose);
  }
  throw std::invalid_argument("predictMeasurement: unsupported MeasType");
}

Eigen::Matrix<double, 3, 6> measurementJacobian(const Vec6& x, const Measurement& m) {
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  switch (m.type) {
    case MeasType::Cartesian:
      H.leftCols<3>() = Mat3::Identity();
      return H;
    case MeasType::RangeAzEl:
      H.leftCols<3>() = rae::fromCartesianJacobian(x.head<3>(), m.pose);
      return H;
  }
  throw std::invalid_argument("measurementJacobian: unsupported MeasType");
}

Vec3 residual(const Vec6& x, const Measurement& m) {
  Vec3 y = m.z - predictMeasurement(x, m);
  if (m.type == MeasType::RangeAzEl) {
    y(1) = wrapAngle(y(1));
    // y(2) = wrapAngle(y(2));  // no-op in practice, kept for documentation
  }
  return y;
}

void measurementToPosition(const Measurement& m, Vec3& p_world, Mat3& R_world) {
  switch (m.type) {
    case MeasType::Cartesian:
      p_world = m.z;
      R_world = m.R;
      return;
    case MeasType::RangeAzEl:
      p_world = rae::toCartesian(m.z, m.pose);
      R_world = rae::toCartesianCov(m.z, m.R, m.pose);
      return;
  }
  throw std::invalid_argument("measurementToPosition: unsupported MeasType");
}

bool measurementUsable(const Vec6& x, const Measurement& m) {
  if (m.type == MeasType::RangeAzEl) return !rae::nearSingularity(x.head<3>(), m.pose);
  return true;
}

}  // namespace sfsim
