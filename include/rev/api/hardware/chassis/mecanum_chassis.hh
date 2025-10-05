#pragma once

#include "api.h"
#include "rev/api/hardware/chassis/chassis.hh"
#include "rev/api/hardware/motor/any_motor.hh"

namespace rev {
/**
 * @brief Implementation of a mecanum chassis
 *
 */
class MecanumChassis : public Chassis {
 public:
  /**
   * @brief Construct a new Mecanum Chassis
   *
   * @param frontleft A motor group of the front left side chassis motors. Applying a
   * positive voltage to these should cause the left side of the chassis to move
   * forward.
   * @param frontright A motor group of the front right side chassis motors. Applying a
   * positive voltage to these should cause the right side of the chassis to
   * move forward.
   * @param backleft A motor group of the back left side chassis motors. Applying a
   * positive voltage to these should cause the left side of the chassis to move
   * forward.
   * @param backright A motor group of the back right side chassis motors. Applying a
   * positive voltage to these should cause the right side of the chassis to
   * move forward.
   */
  MecanumChassis(rev::AnyMotor& ifrontleft, rev::AnyMotor& ifrontright, rev::AnyMotor& ibackleft, rev::AnyMotor& ibackright);

  /**
   * @brief Drives mecanum chassis
   *
   * @param horizontal_v Horizontal (Side to side) direction input from controller
   * @param vertical_v Vertical (Front to back) direction input from controller
   * @param angular_v Angular (Turning) direction input from controller
   */
  void drive(double horizontal_v, double vertical_v, double angular_v);

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
  rev::AnyMotor* frontleft;
  rev::AnyMotor* frontright;
  rev::AnyMotor* backleft;
  rev::AnyMotor* backright;
};
}  // namespace rev