#pragma once

#include <memory>
#include "rev/api/alg/stop/stop.hh"
#include "rev/api/units/q_time.hh"

namespace rev {

/**
 * @brief Stop controller implementing a simple stopping algorithm for holonomic drive
 *
 * This controller considers movement in all three degrees of freedom (x, y, theta)
 * independently, making it suitable for holonomic drive systems like mecanum or
 * omni-wheel configurations.
 */
class SimpleHolonomicStop : public Stop {
 public:
  /**
   * @brief Construct a new Simple Holonomic Stop controller
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
  SimpleHolonomicStop(QTime iharsh_threshold,
                      QTime icoast_threshold,
                      double icoast_power);

  /**
   * @brief Construct a new Simple Holonomic Stop controller
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
   * @param itimeout The timeout for this controller
   */
  SimpleHolonomicStop(QTime iharsh_threshold,
                      QTime icoast_threshold,
                      double icoast_power,
                      QTime itimeout);

  /**
   * @brief Find the current stop state for holonomic drive
   *
   * This method considers translational movement in both x and y directions
   * as well as rotational movement when determining the stopping state.
   *
   * @param current_state The current position and velocity of the robot
   * @param target_state The position being targeted
   * @param start_state The place the robot started from
   * @param drop_early The distance from the target that the robot should aim to
   * exit this step of the controller
   * @return StopState
   */
  StopState get_stop_state(OdometryState current_state,
                            Position target_state,
                            Position start_state,
                            QLength drop_early) override;

  double get_coast_power() override;

  std::shared_ptr<SimpleHolonomicStop> operator&() {
    return std::make_shared<SimpleHolonomicStop>(*this);
  }

 private:
  const QTime harsh_threshold;
  const QTime coast_threshold;
  const double coast_power;
  StopState stop_state_last{StopState::GO};

  QTime timeout{0_s};

  /**
   * @brief Calculate the total speed considering all movement components
   *
   * @param velocity The current velocity state
   * @return QSpeed The magnitude of translational velocity
   */
  QSpeed calculate_total_speed(const Velocity& velocity);

  /**
   * @brief Calculate the total distance to target considering all position components
   *
   * @param current_pos Current position
   * @param target_pos Target position
   * @param drop_early Early exit distance
   * @return QLength The total distance to target
   */
  QLength calculate_total_distance(const Pose& current_pos,
                                   const Pose& target_pos,
                                   QLength drop_early);
};

}  // namespace rev
