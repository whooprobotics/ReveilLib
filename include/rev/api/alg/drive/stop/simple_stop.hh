#pragma once

#include "rev/api/alg/drive/stop/stop.hh"
#include "pros/rtos.hpp"

namespace rev {

/**
 * @brief Stop controller implementing a simple stopping algorithm
 *
 */
class SimpleStop : public Stop {
 public:
  /**
   * @brief Construct a new Simple Stop controller
   *
   * @param iharsh_threshold If the robot would reach its target traveling at
   * the current speed in this amount of time or less, this will activate the
   * harsh stop. Set to 0_s to disable harsh braking.
   * @param icoast_threshold If the robot would reach its target traveling at
   * the current speed in this amount of time or less, this will cut power to
   * the motors down to the coast power. Set to 0_s to disable coasting.
   * @param icoast_power The power that will be applied to the motors to allow
   * the robot to coast. This should be just enough to overcome static friction.
   * Range is [0.0, 1.0]
   */
  SimpleStop(QTime iharsh_threshold,
             QTime icoast_threshold,
             double icoast_power);

  /**
   * @brief Construct a new Simple Stop controller
   *
   * @param iharsh_threshold If the robot would reach its target traveling at
   * the current speed in this amount of time or less, this will activate the
   * harsh stop. Set to 0_s to disable harsh braking.
   * @param icoast_threshold If the robot would reach its target traveling at
   * the current speed in this amount of time or less, this will cut power to
   * the motors down to the coast power. Set to 0_s to disable coasting.
   * @param icoast_power The power that will be applied to the motors to allow
   * the robot to coast. This should be just enough to overcome static friction.
   * Range is [0.0, 1.0]
   * @param itimeout The timeout for this
   */
  SimpleStop(QTime iharsh_threshold,
             QTime icoast_threshold,
             double icoast_power,
             QTime itimeout);

  /**
   * @brief Find the current stop state
   *
   * @param current_state The current position and velocity of the robot
   * @param target_state The position being targeted
   * @param start_state The place the robot started from
   * @param drop_early The distance from the target that the robot should aim to
   * exit this step of the controller
   * @return stop_state
   */
  stop_state get_stop_state(OdometryState current_state,
                            Position target_state,
                            Position start_state,
                            QLength drop_early) override;
  double get_coast_power() override;

 private:
  const QTime harsh_threshold;
  const QTime coast_threshold;
  const double coast_power;
  stop_state stop_state_last{stop_state::GO};

  uint32_t time_init {0};
  uint32_t timeout {0};
};

}  // namespace rev