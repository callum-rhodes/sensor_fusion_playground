#pragma once
 
namespace sfsim {
 
// Tunable tracker settings. Owned by Simulation (like the sensor configs) and edited live from the GUI;
// trackers hold a reference and read the current values on every step.
struct TrackerParams {
  double q_accel = 50.0;          // process noise intensity
  double gate_chi2 = 11.34;      // chi-square 99% for 3 dof
  int confirm_hits = 20;
  int delete_misses = 10;
  double init_vel_sigma = 100.0;
};
 
}  // namespace sfsim