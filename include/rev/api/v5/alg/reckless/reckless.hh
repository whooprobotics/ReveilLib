#pragma once

#ifdef PLATFORM_BRAIN
#include <memory>
#include "rev/api/v5/alg/reckless/path.hh"
#include "rev/api/v5/async/async_awaitable.hh"
#include "rev/api/v5/async/async_runnable.hh"
#include "rev/api/v5/hardware/chassis/chassis.hh"

namespace rev {

enum class RecklessStatus { ACTIVE, DONE };

/**
 * @brief High-speed chassis motion controller
 *
 */
class Reckless : public AsyncRunnable, public AsyncAwaitable {
 public:
  /**
   * @brief Constructs a new Reckless controller instance
   * 
   * @param ichassis std::shared_ptr<Chassis> pointing to Chassis instance, with motors for control
   * @param odometry std::shared_ptr<Odometry> pointing to odometry with sensors
   */
  Reckless(std::shared_ptr<Chassis> ichassis,
           std::shared_ptr<Odometry> odometry);

  /**
   * @brief Steps the Reckless controller
   * 
   * The step function for the AsyncRunnable. This is where the bulk of the
   * controller logic is
   */
  void step() override;

  /**
   * @brief Blocks until the motion is completed
   *
   */
  void await() override;

  /**
   * @brief Status indicator for awaitable.
   *
   */
  bool is_ready() override;

  /**
   * This function starts the robot along a path
   */
  void go(RecklessPath path);

  /**
   * This function starts the robot along a path
   */
  void go(std::initializer_list<std::shared_ptr<RecklessSegment>> path) {
    go(RecklessPath(path));
  }

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

  size_t current_segment = 0;
  long long brake_start_time = -1;
  double partial_progress = -1.0;
};

}  // namespace rev

#endif