#include <algorithm>
#include <cstdlib>

#include "rev/api/hardware/chassis/mecanum_chassis.hh"

namespace rev {
MecanumChassis::MecanumChassis(rev::AnyMotor& ifrontleft, rev::AnyMotor& ifrontright, rev::AnyMotor& ibackleft, rev::AnyMotor& ibackright)
    : frontleft(&ifrontleft), frontright(&ifrontright), backleft(&ibackleft), backright(&ibackright) {}

void MecanumChassis::drive(double horizontal_v, double vertical_v, double angular_v) {
  /* horizontal_v = std::clamp(horizontal_v, -1.0, 1.0);
  vertical_v = std::clamp(vertical_v, -1.0, 1.0);
  angular_v = std::clamp(angular_v, -1.0, 1.0); */
  
  // denominator ensures that each motor group has the same "ratio" of power
  double denominator = std::max(std::abs(horizontal_v), std::abs(vertical_v), std::abs(angular_v));
  double front_left_power = (vertical_v + horizontal_v + angular_v) / denominator;
  double front_right_power = (vertical_v - horizontal_v - angular_v) / denominator;
  double back_left_power = (vertical_v - horizontal_v + angular_v) / denominator;
  double back_right_power = (vertical_v + horizontal_v - angular_v) / denominator;

  frontleft->move_voltage(12000 * front_left_power);
  frontright->move_voltage(12000 * front_left_power);
  backleft->move_voltage(12000 * front_left_power);
  backright->move_voltage(12000 * front_left_power);
}

void MecanumChassis::set_brake_harsh() {
  frontleft->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  frontright->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  backleft->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  backright->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
}

void MecanumChassis::set_brake_coast() {
  frontleft->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  frontright->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  backleft->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  backright->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
}

void MecanumChassis::stop() {
  frontleft->brake();
  frontright->brake();
  backleft->brake();
  backright->brake();
}
}  