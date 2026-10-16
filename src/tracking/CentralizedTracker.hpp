#pragma once
#include <vector>
#include "tracking/IFusion.hpp"
#include "tracking/ExtendedKalmanFilter.hpp"

namespace sfsim {

struct TrackerParams {
  double q_accel = 1.0;          // process noise intensity
  double gate_chi2 = 11.34;      // chi-square 99% for 3 dof
  int confirm_hits = 3;
  int delete_misses = 5;
  double init_vel_sigma = 100.0;
};

// Fusion centre: all sensors' measurements -> one track pool.
// Association is greedy nearest-neighbour for now; replace with GNN (Hungarian) / MHT.
class CentralizedTracker : public IFusion {
 public:
  explicit CentralizedTracker(TrackerParams p = {}) : p_(p) {}
  void step(double t, const std::vector<Sensor>& sensors, const std::vector<Measurement>& ms) override;
  std::vector<Track> tracks() const override;

 private:
  struct Internal { Track meta; EKF kf; };
  TrackerParams p_;
  std::vector<Internal> pool_;
  uint32_t next_id_ = 1;
  double last_t_ = -1;
};

}  // namespace sfsim
