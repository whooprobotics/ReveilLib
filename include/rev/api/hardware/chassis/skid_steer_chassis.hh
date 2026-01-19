#pragma once

#include "api.h"
#include "rev/api/hardware/chassis/chassis.hh"
#include "rev/api/hardware/motor/any_motor.hh"

namespace rev {
/**
 * @brief Implementation of a skid-steer chassis
 *
 */
class SkidSteerChassis : public Chassis {
 public:
  /**
   * @brief Construct a new Skid Steer Chassis
   *
   * @param ileft A motor group of the left side chassis motors. Applying a
   * positive voltage to these should cause the left side of the chassis to move
   * forward.
   * @param iright A motor group of the right side chassis motors. Applying a
   * positive voltage to these should cause the right side of the chassis to
   * move forward.
   */
  SkidSteerChassis(rev::AnyMotor& ileft, rev::AnyMotor& iright);

  /**
   * @brief Moves the robot
   * 
   * @param leftv From [-1.0, 1.0]. Applies a voltage to the motor group.
   * @param rightv From [-1.0, 1.0]. Applies a voltage to the motor group.
   */
  void drive_tank(double leftv, double rightv) override;

  /**
   * @brief Moves the robot
   * 
   * @param forward From [-1.0, 1.0]. The forward component of motion.
   * @param yaw From [-1.0, 1.0]. The yaw component of motion.
   */
  void drive_arcade(double forward, double yaw) override;

  /**
   * @brief Sets the brake types of all motors to brake
   */
  void set_brake_harsh() override;

  /**
   * @brief Sets the brake types of all motors to coast
   */
  void set_brake_coast() override;

  /**
   * @brief Stops all of the motors
   */
  void stop() override;

 private:
  std::shared_ptr<rev::AnyMotor> left;
  std::shared_ptr<rev::AnyMotor> right;
};
}  // namespace rev