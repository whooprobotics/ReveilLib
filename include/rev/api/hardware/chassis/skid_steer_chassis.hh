#pragma once

#include "api.h"
#include "rev/api/hardware/chassis/chassis.hh"

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
   * @param left A motor group of the left side chassis motors. Applying a
   * positive voltage to these should cause the left side of the chassis to move
   * forward.
   * @param right A motor group of the right side chassis motors. Applying a
   * positive voltage to these should cause the right side of the chassis to
   * move forward.
   */
  SkidSteerChassis(pros::MotorGroup& ileft, pros::MotorGroup& iright);

  void drive_tank(double leftv, double rightv) override;
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
  pros::MotorGroup* left;
  pros::MotorGroup* right;
};
}  // namespace rev