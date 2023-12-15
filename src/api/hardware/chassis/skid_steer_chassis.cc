#include <algorithm>

#include "rev/api/hardware/chassis/skid_steer_chassis.hh"

namespace rev {
SkidSteerChassis::SkidSteerChassis(pros::MotorGroup& ileft,
                                   pros::MotorGroup& iright)
    : left(&ileft), right(&iright) {}

void SkidSteerChassis::drive_tank(double leftv, double rightv) {
  leftv = std::clamp(leftv, -1.0, 1.0);
  rightv = std::clamp(rightv, -1.0, 1.0);

  left->move_voltage(12000 * leftv);
  right->move_voltage(12000 * rightv);
}
void SkidSteerChassis::drive_arcade(double forward, double yaw) {
  double scale = fabs(forward) + fabs(yaw);
  if (scale > 1.0) {
    forward /= scale;
    yaw /= scale;
  }

  drive_tank(forward + yaw, forward - yaw);
}

void SkidSteerChassis::set_brake_harsh() {
  left->set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  right->set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
}

void SkidSteerChassis::set_brake_coast() {
  left->set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  right->set_brake_modes(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
}

void SkidSteerChassis::stop() {
  left->brake();
  right->brake();
}
}  // namespace rev