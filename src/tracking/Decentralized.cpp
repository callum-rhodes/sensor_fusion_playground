#include "tracking/Decentralized.hpp"

namespace sfsim {

void covarianceIntersection(const Vec6& xa, const Mat6& Pa, const Vec6& xb, const Mat6& Pb,
                            Vec6& x, Mat6& P) {
  const Mat6 Ia = Pa.inverse(), Ib = Pb.inverse();
  double best_tr = 1e300;
  for (int k = 0; k <= 20; ++k) {
    const double w = k / 20.0;
    const Mat6 Pw = (w * Ia + (1 - w) * Ib).inverse();
    const double tr = Pw.trace();
    if (tr < best_tr) {
      best_tr = tr;
      P = Pw;
      x = Pw * (w * Ia * xa + (1 - w) * Ib * xb);
    }
  }
}

void CommNetwork::send(double t, Packet p) {
  std::uniform_real_distribution<double> u(0, 1);
  if (u(rng_) < cfg_.drop_prob) return;
  p.deliver_at = t + cfg_.latency_s;
  in_flight_.push_back(std::move(p));
}

std::vector<Packet> CommNetwork::deliver(double t, SensorId to) {
  std::vector<Packet> out;
  for (auto it = in_flight_.begin(); it != in_flight_.end();) {
    if (it->to == to && it->deliver_at <= t) { out.push_back(std::move(*it)); it = in_flight_.erase(it); }
    else ++it;
  }
  return out;
}

}  // namespace sfsim
