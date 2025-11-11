#pragma once

#include "rev/api/alg/odometry/odometry.hh"

namespace rev {

enum class StopState { GO, COAST, BRAKE, EXIT };

/**
 * @brief Interface for stop controllers
 *
 */
class Stop {
 public:
  /**
   * @brief Find the current stop state
   *
   * @param current_state The current position and velocity of the robot
   * @param target_state The position being targeted
   * @param start_state The place the robot started from
   * @param drop_early The distance from the target that the robot should aim to
   * exit this step of the controller
   * 
   * @return StopState The current stop state of the segment
   */
  virtual StopState get_stop_state(OdometryState current_state,
                                    Position target_state,
                                    Position start_state,
                                    QLength drop_early) = 0;

  virtual double get_coast_power() = 0;
};
} // namespace rev