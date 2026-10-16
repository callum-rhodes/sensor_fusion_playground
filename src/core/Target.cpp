#include "core/Target.hpp"
#include <cmath>

namespace sfsim {

void ConstantVelocity::advance(TargetState& s, double dt) {
  s.pos += s.vel * dt;
}

void CoordinatedTurn::advance(TargetState& s, double dt) {
  if (std::abs(omega_) < 1e-9) { s.pos += s.vel * dt; return; }
  const double c = std::cos(omega_ * dt), sn = std::sin(omega_ * dt);
  const double vx = s.vel.x(), vy = s.vel.y();
  s.pos.x() += (vx * sn - vy * (1 - c)) / omega_;
  s.pos.y() += (vx * (1 - c) + vy * sn) / omega_;
  s.pos.z() += s.vel.z() * dt;
  s.vel.x() = vx * c - vy * sn;
  s.vel.y() = vx * sn + vy * c;
}

}  // namespace sfsim
