#pragma once

#include <memory> // why is this included here, but not in skid_steer
#include "rev/api/hardware/chassis/holonomic_chassis.hh"
#include "rev/api/hardware/motor/any_motor.hh"

namespace rev {
/**
 * @brief Implementation of a mecanum chassis
 * 
 */
class MecanumChassis : public Chassis {
 public:
  /**
   * @brief Construct a new Mecanum Chassis object
    * //Deeply unclear how the motors power each wheel.
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
  rev::AnyMotor* front_left;
  rev::AnyMotor* back_left;
  rev::AnyMotor* front_right;
  rev::AnyMotor* back_right;

};
}