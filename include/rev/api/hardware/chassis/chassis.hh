#pragma once

namespace rev {
class Chassis {
  /**
   * @brief Moves the robot
   *
   * @param left From [-1.0, 1.0]. Applies a voltage to the motor group.
   * @param right From [-1.0, 1.0]. Applies a voltage to the motor group.
   */
  virtual void drive_tank(double left, double right) = 0;
  /**
   * @brief Moves the robot
   *
   * @param forward From [-1.0, 1.0]. The forward component of the motion.
   * @param yaw From [-1.0, 1.0]. The clockwise component of the motion.
   */
  virtual void drive_arcade(double forward, double yaw) = 0;
};
}  // namespace rev