#pragma once

namespace rev {
/**
 * @brief Interface for chassis objects
 *
 */
class Chassis {
 public:
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
   * @param yaw From [-1.0, 1.0]. The yaw component of the motion.
   */
  virtual void drive_arcade(double forward, double yaw) = 0;
  /**
   * @brief Sets the brake types of all motors to brake
   */
  virtual void set_brake_harsh() = 0;
  /**
   * @brief Sets the brake types of all motors to coast
   */
  virtual void set_brake_coast() = 0;
  /**
   * @brief Stops all of the motors
   */
  virtual void stop() = 0;
};
}  // namespace rev