#pragma once

#include <memory>

#include "rev/api/alg/reckless/path.hh"
#include "rev/api/async/async_awaitable.hh"
#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/chassis/chassis.hh"

namespace rev {

enum class RecklessStatus { ACTIVE, DONE };

/**
 * @brief High-speed chassis motion controller
 *
 */
class Reckless : public AsyncRunnable, public AsyncAwaitable {
 public:
  Reckless(std::shared_ptr<Chassis> ichassis,
           std::shared_ptr<Odometry> odometry);
  /**
   * The step function for the asyncrunnable. This is where the bulk of the
   * controller logic is
   */
  void step() override;

  /**
   * @brief Blocks until the motion is completed
   *
   */
  void await() override;

  /**
   * This function starts the robot along a path
   */
  void go(RecklessPath path);

  /**
   * This function returns the current status of the controller
   */
  RecklessStatus get_status();

  /**
   * This function gets the current progress along the total path. [0,1] for the
   * first segment, [1,2] second segment, etc. Returns -1.0 if the controller is
   * not running. Returns the integer upper bound of a motion if that motion has
   * invoked a harsh stop
   */
  double progress();
  /**
   * This function returns true if the status is DONE, and false otherwise
   */
  bool is_completed();
  /**
   * This function immediately sets the status to DONE and ends the current
   * motion
   */
  void breakout();

 private:
  std::shared_ptr<Chassis> chassis;
  std::shared_ptr<Odometry> odometry;
  RecklessPath current_path;
  RecklessStatus status{RecklessStatus::DONE};

  size_t current_segment{0};
  long long brake_time = -1;
  double partial_progress{-1.0};
};
}  // namespace rev
