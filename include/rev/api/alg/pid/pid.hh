#pragma once

#include "pros/rtos.hpp"

namespace rev {

/**
 * @brief Simple PID controller
 *
 * Returns a power in the range [-1, 1]. Multiply by 12000 to get millivolts.
 */
class PID {
 public:
  /**
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   * @param integral_limit Clamps the integral term to [-integral_limit, integral_limit] to prevent windup
   */
  PID(double kp, double ki, double kd, double integral_limit = 1.0);

  /**
   * @brief Compute PID output given a target and current position.
   *
   * @param target  Desired position (encoder ticks, degrees, etc.)
   * @param current Current position in the same units
   * @return Power in [-1, 1]
   */
  double update(double target, double current);

  /**
   * @brief Reset integral and derivative state (call when re-using for a new move)
   */
  void reset();

 private:
  double kp;
  double ki;
  double kd;
  double integral_limit;

  double integral;
  double prev_error;
  uint32_t prev_time;
};

} // namespace rev
