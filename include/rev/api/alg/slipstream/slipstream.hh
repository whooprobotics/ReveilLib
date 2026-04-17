#pragma once

#include <memory>
#include "rev/api/async/async_awaitable.hh"
#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/chassis/holonomic_chassis.hh"
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/slipstream/path.hh"

namespace rev {

enum class SlipstreamStatus { ACTIVE, DONE };

class Slipstream : public AsyncRunnable, public AsyncAwaitable {
 public:
  Slipstream(std::shared_ptr<HolonomicChassis> ichassis,
             std::shared_ptr<Odometry> iodometry);

  void step() override;

  void await() override;

  bool is_ready() override;

  void go(SlipstreamPath path);

  void go(std::initializer_list<std::shared_ptr<SlipstreamSegment>> path) {
    go(SlipstreamPath(path));
  }

  SlipstreamStatus get_status();

  double progress();

  bool is_completed();

  void breakout();
 private:
  std::shared_ptr<HolonomicChassis> chassis;
  std::shared_ptr<Odometry> odometry;
  SlipstreamPath current_path;
  SlipstreamStatus status{SlipstreamStatus::DONE};

  size_t current_segment{0};
  long long brake_start_time = -1;
  double partial_progress{-1.0};
};

} // namespace rev