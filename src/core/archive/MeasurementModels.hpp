#pragma once
#include "core/Types.hpp"
#include "core/Measurement.hpp"

namespace sfsim {

// ---------------------------------------------------------------------------
// Range / azimuth / elevation <-> Cartesian, relative to a sensor pose.
//
// Sensor frame: x forward, y left, z up.
//   range   r  = |p_sensor|
//   azimuth az = atan2(y, x)           in [-pi, pi]
//   elev    el = asin(z / r)           in [-pi/2, pi/2]
// Sensor orientation is applied here, so callers always work in world frame.
// ---------------------------------------------------------------------------
namespace rae {

// World position -> (range, az, el) as seen from the sensor.
Vec3 fromCartesian(const Vec3& p_world, const SensorPose& pose);

// d(range, az, el) / d(world position), 3x3.
// Near the singularities (r ~ 0, or target on the sensor's vertical axis) the
// denominators are clamped, so the result is finite but azimuth is meaningless.
Mat3 fromCartesianJacobian(const Vec3& p_world, const SensorPose& pose);

// (range, az, el) -> world position.
Vec3 toCartesian(const Vec3& z, const SensorPose& pose);

// d(world position) / d(range, az, el), 3x3.
Mat3 toCartesianJacobian(const Vec3& z, const SensorPose& pose);

// Transform a measurement covariance R (in range/az/el) to world-frame position
// covariance, to first order: J R J^T. Used for track initialisation.
Mat3 toCartesianCov(const Vec3& z, const Mat3& R, const SensorPose& pose);

// True if the target is so close to the sensor's vertical axis (or to the
// sensor itself) that azimuth is ill-conditioned. Lets callers skip an update.
bool nearSingularity(const Vec3& p_world, const SensorPose& pose);

}  // namespace rae

// ---------------------------------------------------------------------------
// Dispatch layer keyed on Measurement::type. The EKF only talks to these.
// State layout is [x y z vx vy vz]; measurements depend on position only.
// ---------------------------------------------------------------------------

// h(x): predicted measurement.
Vec3 predictMeasurement(const Vec6& x, const Measurement& m);

// H = dh/dx, 3x6 (velocity columns are zero).
Eigen::Matrix<double, 3, 6> measurementJacobian(const Vec6& x, const Measurement& m);

// z - h(x), with angular components wrapped to [-pi, pi].
Vec3 residual(const Vec6& x, const Measurement& m);

// Convert a measurement to a world-frame position and covariance, for
// initialising a track. Works for every MeasType.
void measurementToPosition(const Measurement& m, Vec3& p_world, Mat3& R_world);

// False if the measurement geometry is degenerate (see rae::nearSingularity)
// and the filter should skip the update.
bool measurementUsable(const Vec6& x, const Measurement& m);

}  // namespace sfsim
