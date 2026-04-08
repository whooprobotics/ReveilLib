#pragma once

#include <memory>
#include "rev/api/alg/slipstream/power.hh"
#include "rev/api/hardware/chassis/holonomic_chassis.hh"
#include "rev/api/hardware/motor/any_motor.hh"

namespace rev {
/**
 * @brief Implementation of a Mecanum chassis
 * 
 */
class MecanumChassis : public HolonomicChassis {
 public:
  /**
   * @brief Construct a new MecanumChassis object

   * @param front_left A motor (group) of the front left chassis motor(s).
   * 
   * @param back_left A motor (group) of the back left chassis motor(s).
   * 
   * @param front_right A motor (group) of the front right chassis motor(s).
   * 
   * @param back_right A motor (group) of the back right chassis motor(s).
   *
   */
  MecanumChassis(rev::AnyMotor& ifront_left, rev::AnyMotor& iback_left, 
                 rev::AnyMotor& ifront_right, rev::AnyMotor& iback_right);
  
  /**
   * @brief Drive the robot in a tank drive style
   */
  void drive_tank(double leftv, double rightv) override;

  /**
   * @brief Heuristic function for a tank drive with arcade joystick controls
   */
  void drive_arcade(double forward, double yaw) override;

  /**
   * @brief Drive the robot in a mecanum drive style
   */
  void drive_holonomic(SlipstreamPower power) override;
  
  /**
   * @brief Heuristic function for a holonomic drive from joystick controls
   */
  void drive_holonomic(double forward, double yaw, double strafe, double angle);
  
  /**
   * @brief Heuristic function for a holonomic drive from joystick controls
   */
  void drive_holonomic(double forward, double yaw, double strafe) override;
  
  /**
   * @brief Sets the brake types of all chassis motors to brake
   */
  void set_brake_harsh() override;

  /**
   * @brief Sets the brake types of all chassis motors to coast
   */
  void set_brake_coast() override;

  /**
   * @brief Stops all chassis motors
   */
  void stop() override;

 private:
  std::shared_ptr<rev::AnyMotor> front_left;
  std::shared_ptr<rev::AnyMotor> back_left;
  std::shared_ptr<rev::AnyMotor> front_right;
  std::shared_ptr<rev::AnyMotor> back_right;
};
} // namespace rev