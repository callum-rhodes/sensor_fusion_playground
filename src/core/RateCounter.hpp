// core/RateCounter.hpp
#pragma once
#include <chrono>

class RateCounter {
  using clock = std::chrono::steady_clock;
public:
  explicit RateCounter(double window_s = 0.5) : window_(window_s) {}

  void tick() {
    const auto now = clock::now();
    if (count_ == 0 && hz_ == 0.0) window_start_ = now;   // first tick
    ++count_;
    last_tick_ = now;
    const double elapsed = std::chrono::duration<double>(now - window_start_).count();
    if (elapsed >= window_) {
      hz_ = count_ / elapsed;
      count_ = 0;
      window_start_ = now;
    }
  }

  // Returns 0 if no tick for a while (paused / stopped), instead of a stale value.
  double hz() const {
    const double since = std::chrono::duration<double>(clock::now() - last_tick_).count();
    return since > 2.0 * window_ ? 0.0 : hz_;
  }

private:
  double window_;
  double hz_ = 0.0;
  int    count_ = 0;
  clock::time_point window_start_ = clock::now();
  clock::time_point last_tick_    = clock::now();
};