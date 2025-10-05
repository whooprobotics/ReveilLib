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

   * @param front_left 
   * 
   * @param back_left
   * 
   * @param front_right
   * 
   * @param back_right
   *
   */
  MecanumChassis(rev::AnyMotor& ifront_left, rev::AnyMotor& iback_left, rev::AnyMotor& ifront_right, rev::AnyMotor& iback_right);
  
  void drive_tank(double leftv, double rightv) override;
  void drive_arcade(double forward, double yaw) override;
  void drive_holonomic(SlipstreamPower power) override;
  void drive_holonomic(double forward, double yaw, double strafe) override;
  
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
  std::shared_ptr<rev::AnyMotor> front_left;
  std::shared_ptr<rev::AnyMotor> back_left;
  std::shared_ptr<rev::AnyMotor> front_right;
  std::shared_ptr<rev::AnyMotor> back_right;
};
} // namespace rev