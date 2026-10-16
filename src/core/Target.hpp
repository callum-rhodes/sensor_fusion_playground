#pragma once
#include <memory>
#include "core/Types.hpp"

namespace sfsim {

struct TargetState {
  Vec3 pos = Vec3::Zero();
  Vec3 vel = Vec3::Zero();
  Vec3 acc = Vec3::Zero();
};

// Trajectory profiles plug in here (constant velocity, coordinated turn, ...)
class TrajectoryModel {
 public:
  virtual ~TrajectoryModel() = default;
  virtual void advance(TargetState& s, double dt) = 0;
};

class ConstantVelocity : public TrajectoryModel {
 public:
  void advance(TargetState& s, double dt) override;
};

// Turn rate omega (rad/s) about the +z axis; vertical speed stays constant.
class CoordinatedTurn : public TrajectoryModel {
 public:
  explicit CoordinatedTurn(double omega) : omega_(omega) {}
  void advance(TargetState& s, double dt) override;
 private:
  double omega_;
};

struct Target {
  TargetId id = 0;
  TargetState state;
  std::unique_ptr<TrajectoryModel> model;
  double birth_time = 0;
  double death_time = 1e18;
  bool alive(double t) const { return t >= birth_time && t < death_time; }
};

}  // namespace sfsim
