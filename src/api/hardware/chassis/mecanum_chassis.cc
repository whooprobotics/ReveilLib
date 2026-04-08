#include <algorithm> 

#include "rev/api/hardware/chassis/mecanum_chassis.hh"
#include <cmath>

namespace rev {
MecanumChassis::MecanumChassis(
        rev::AnyMotor& ifront_left,
        rev::AnyMotor& ifront_right,
        rev::AnyMotor& iback_left,
        rev::AnyMotor& iback_right)
    : front_left(&ifront_left),
      back_left(&iback_left),
      front_right(&ifront_right),
      back_right(&iback_right) {}

void MecanumChassis::drive_tank(double leftv, double rightv) {
  leftv = std::clamp(leftv, -1.0, 1.0);
  rightv = std::clamp(rightv, -1.0, 1.0);

  front_left->move_voltage(12000 * leftv);
  back_left->move_voltage(12000 * leftv);
  front_right->move_voltage(12000 * rightv);
  back_right->move_voltage(12000 * rightv);
}

void MecanumChassis::drive_arcade(double forward, double yaw) {
  double scale = fabs(forward) + fabs(yaw);
  if (scale > 1.0) {
    forward /= scale;
    yaw /= scale;
  }
  drive_tank(forward + yaw, forward - yaw);
}

void MecanumChassis::drive_holonomic(SlipstreamPower power) {
  power.clamp_powers();
  front_left->move_voltage(12000 * power.front_left_forward);
  back_left->move_voltage(12000 * power.rear_left_forward);
  front_right->move_voltage(12000 * power.front_right_forward);
  back_right->move_voltage(12000 * power.rear_right_forward);
}

void MecanumChassis::drive_holonomic(double forward, double yaw, double strafe, double angle) {
  // double scale = fabs(forward) + fabs(yaw) + fabs(strafe);
  // if (scale > 1.0) {
  //   forward /= scale;
  //   yaw /= scale;
  //   strafe /= scale;
  // }

  float robotFwd =  forward * std::cos(angle) + strafe * std::sin(angle);
  float robotStrafe = -forward * std::sin(angle) + strafe * std::cos(angle);


  SlipstreamPower power;
  power.front_left_forward = forward + yaw + strafe;
  power.rear_left_forward = forward + yaw - strafe;
  power.front_right_forward = forward - yaw - strafe;
  power.rear_right_forward = forward - yaw + strafe;

  drive_holonomic(power);
}

void MecanumChassis::drive_holonomic(double forward, double yaw, double strafe) {
  // double scale = fabs(forward) + fabs(yaw) + fabs(strafe);
  // if (scale > 1.0) {
  //   forward /= scale;
  //   yaw /= scale;
  //   strafe /= scale;
  // }

  SlipstreamPower power;
  power.front_left_forward = forward + yaw + strafe;
  power.rear_left_forward = forward + yaw - strafe;
  power.front_right_forward = forward - yaw - strafe;
  power.rear_right_forward = forward - yaw + strafe;

  drive_holonomic(power);
}


void MecanumChassis::set_brake_harsh() {
  front_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  back_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  front_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  back_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
}

void MecanumChassis::set_brake_coast() {
  front_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  back_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  front_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  back_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
}

void MecanumChassis::stop() {
  front_left->brake();
  back_left->brake();
  front_right->brake();
  back_right->brake();
}
} //namespace rev
