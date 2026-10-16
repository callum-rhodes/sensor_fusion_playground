#pragma once
#include <deque>
#include <random>
#include <vector>
#include "tracking/IFusion.hpp"

namespace sfsim {

// Fuse two estimates with unknown cross-correlation. omega picked by minimising trace(P).
void covarianceIntersection(const Vec6& xa, const Mat6& Pa, const Vec6& xb, const Mat6& Pb,
                            Vec6& x, Mat6& P);

struct CommConfig {
  double latency_s = 0.1;
  double drop_prob = 0.05;
  // TODO: bandwidth limit, topology (adjacency matrix)
};

struct Packet {
  SensorId from = 0, to = 0;
  double deliver_at = 0;
  std::vector<Track> tracks;
};

// Simulated comms graph: send() now, deliver() returns what has arrived by t.
class CommNetwork {
 public:
  CommNetwork(CommConfig c, uint64_t seed) : cfg_(c), rng_(seed) {}
  void send(double t, Packet p);
  std::vector<Packet> deliver(double t, SensorId to);
 private:
  CommConfig cfg_;
  std::mt19937_64 rng_;
  std::deque<Packet> in_flight_;
};

// TODO: DecentralizedFusion : IFusion
//   - one local CentralizedTracker per sensor (filter its own measurements only)
//   - each tick: broadcast local tracks via CommNetwork
//   - on receive: associate by gating, fuse with covarianceIntersection
//   - tracks() returns one node's view (selectable in the GUI) or the union

}  // namespace sfsim
