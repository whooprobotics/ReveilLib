#ifdef PLATFORM_BRAIN

#include <algorithm> 
#include <cmath>
#include "rev/rev.hh"
#include "rev/api/v5/hardware/chassis/butterfly_chassis.hh"

namespace rev {

ButterflyChassis::ButterflyChassis(rev::AnyMotor& ifront_left, rev::AnyMotor& ifront_right, rev::AnyMotor& iback_left, 
  rev::AnyMotor& iback_right, pros::ADIDigitalOut& ileft_piston, pros::ADIDigitalOut& iright_piston)
  : MecanumChassis(ifront_left, ifront_right, iback_left, iback_right),
    left_piston(&ileft_piston), right_piston(&iright_piston), pistons_down(false) {}

void ButterflyChassis::drive_auto(double forward, double yaw, double strafe){
  if(pistons_down){
    drive_arcade(forward, yaw);
  } else {
    drive_holonomic(forward, yaw, strafe);
  }
}

void ButterflyChassis::toggle_pistons() {
  left_piston->set_value(!pistons_down);
  right_piston->set_value(!pistons_down);
  pistons_down = !pistons_down;
}

void ButterflyChassis::set_pistons(bool piston_val) {
  left_piston->set_value(piston_val);
  right_piston->set_value(piston_val);
  pistons_down = piston_val;
}

bool ButterflyChassis::get_pistons() {
  return pistons_down;
}

}  // namespace rev

#endif