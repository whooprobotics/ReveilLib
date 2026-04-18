#pragma once

#ifdef PLATFORM_BRAIN

#include <memory>
#include "rev/api/v5/alg/slipstream/power.hh"
#include "rev/api/v5/hardware/chassis/holonomic_chassis.hh"
#include "rev/api/v5/hardware/motor/any_motor.hh"

namespace rev {
/**
 * @brief Implementation of an "Asterisk" Chassis (4 mecanum wheels and 1 omni on either side between the mecanums)
 * 
 */
class AsteriskChassis : public HolonomicChassis {
 public:
  AsteriskChassis(rev::AnyMotor& ifront_left, rev::AnyMotor& iback_left,
                  rev::AnyMotor& ifront_right, rev::AnyMotor& iback_right, rev::AnyMotor& icenter_left, rev::AnyMotor& icenter_right);

  void drive_tank(double leftv, double rightv) override;

  void drive_arcade(double forward, double yaw) override;

  void drive_holonomic(SlipstreamPower power) override;

  void drive_holonomic(double forward, double turn, double strafe) override;

  void set_brake_harsh() override;

  void set_brake_coast() override;

  void stop() override;
 private:
  std::shared_ptr<rev::AnyMotor> front_left;
  std::shared_ptr<rev::AnyMotor> center_left;
  std::shared_ptr<rev::AnyMotor> back_left;

  std::shared_ptr<rev::AnyMotor> front_right;
  std::shared_ptr<rev::AnyMotor> center_right;
  std::shared_ptr<rev::AnyMotor> back_right;
};
} // namespace rev

#endif