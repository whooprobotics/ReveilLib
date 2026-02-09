#include "rev/api/hardware/chassis/asterisk_chassis.hh"
#include <cmath>

namespace rev {
AsteriskChassis::AsteriskChassis(AnyMotor& ifront_left, AnyMotor& ifront_right,
                                 AnyMotor& iback_left, AnyMotor& iback_right,
                                 AnyMotor& icenter_left, AnyMotor& icenter_right)
                             : front_left(&ifront_left),
                               back_left(&iback_left),
                               front_right(&ifront_right),
                               back_right(&iback_right),
                               center_left(&icenter_left),
                               center_right(&icenter_right) {}

void AsteriskChassis::drive_tank(double leftv, double rightv) {
  leftv = std::clamp(leftv, -1.0, 1.0);
  rightv = std::clamp(rightv, -1.0, 1.0);

  front_left->move_voltage(12000 * leftv);
  center_left->move_voltage(12000 * leftv);
  back_left->move_voltage(12000 * leftv);

  front_right->move_voltage(12000 * rightv);
  center_right->move_voltage(12000 * rightv);
  back_right->move_voltage(12000 * rightv);
}

void AsteriskChassis::drive_arcade(double forward, double yaw) {
  double scale = fabs(forward) + fabs(yaw);
  if (scale > 1.0) {
    forward /= scale;
    yaw /= scale;
  }
  drive_tank(forward + yaw, forward - yaw);
}

void AsteriskChassis::drive_holonomic(double forward, double yaw, double strafe) {
  double scale = fabs(forward) + fabs(yaw) + fabs(strafe);
  if (scale > 1.0) {
    forward /= scale;
    yaw /= scale;
    strafe /= scale;
  }

  double front_left_power = std::clamp(forward + yaw + strafe, -1.0, 1.0);
  double rear_left_power = std::clamp(forward + yaw - strafe, -1.0, 1.0);
  double front_right_power = std::clamp(forward - yaw - strafe, -1.0, 1.0);
  double rear_right_power = std::clamp(forward - yaw + strafe, -1.0, 1.0);

  double center_left_power = std::clamp(forward + yaw, -1.0, 1.0);
  double center_right_power = std::clamp(forward - yaw, -1.0, 1.0);

  drive_holonomic({front_left_power, front_right_power, rear_left_power, rear_right_power, center_left_power, center_right_power});
}

void AsteriskChassis::drive_holonomic(SlipstreamPower power) {
  front_left->move_voltage(12000 * power.front_left_forward);
  back_left->move_voltage(12000 * power.rear_left_forward);
  front_right->move_voltage(12000 * power.front_right_forward);
  back_right->move_voltage(12000 * power.rear_right_forward);

  center_left->move_voltage(12000 * power.front_left_steer);
  center_right->move_voltage(12000 * power.front_right_steer);
}

void AsteriskChassis::set_brake_harsh() {
  front_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  back_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  front_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  back_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);

  center_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
  center_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
}

void AsteriskChassis::set_brake_coast() {
  front_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  back_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  front_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  back_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

  center_left->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
  center_right->set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
}

void AsteriskChassis::stop() {
  front_left->brake();
  back_left->brake();
  front_right->brake();
  back_right->brake();

  center_left->brake();
  center_right->brake();
}

}  // namespace rev