#include "rev/api/hardware/motor/motor_group.hh"
#include <cmath>
rev::MotorGroup::MotorGroup(const std::initializer_list<Motor> motors)
    : motors(motors), motor_count(motors.size()) {}
rev::MotorGroup::MotorGroup(const std::vector<rev::Motor>& motors)
    : motors(motors), motor_count(motors.size()) {}
rev::MotorGroup::MotorGroup(
    const std::initializer_list<std::int8_t> motor_ports)
    : motor_count(motor_ports.size()) {
  for (auto& i : motor_ports) {
    motors.emplace_back(Motor(i));
  }
}
rev::MotorGroup::MotorGroup(const std::vector<std::int8_t> motor_ports)
    : motor_count(motor_ports.size()) {
  for (auto& i : motor_ports) {
    motors.emplace_back(Motor(i));
  }
}

std::int32_t rev::MotorGroup::move(std::int32_t voltage) const {
  for (Motor m : motors)
    m.move(voltage);
  return 0;
}

std::int32_t rev::MotorGroup::move_absolute(const double position,
                                            const std::int32_t velocity) const {
  for (Motor m : motors)
    m.move_absolute(position, velocity);
  return 0;
}

std::int32_t rev::MotorGroup::move_relative(const double position,
                                            const std::int32_t velocity) const {
  for (Motor m : motors)
    m.move_relative(position, velocity);
  return 0;
}

std::int32_t rev::MotorGroup::move_velocity(const std::int32_t velocity) const {
  for (Motor m : motors)
    m.move_velocity(velocity);
  return 0;
}

std::int32_t rev::MotorGroup::move_voltage(const std::int32_t voltage) const {
  for (Motor m : motors)
    m.move_voltage(voltage);
  return 0;
}

std::int32_t rev::MotorGroup::brake(void) const {
  for (Motor m : motors)
    m.brake();
  return 0;
}

std::int32_t rev::MotorGroup::modify_profiled_velocity(
    const std::int32_t velocity) const {
  for (Motor m : motors)
    m.modify_profiled_velocity(velocity);
  return 0;
}

double rev::MotorGroup::get_actual_velocity(void) const {
  return motors.at(0).get_actual_velocity();
}

std::int32_t rev::MotorGroup::get_direction(void) const {
  return motors.at(0).get_direction();
}

std::int32_t rev::MotorGroup::is_stopped(void) const {
  return motors.at(0).is_stopped();
}

std::int32_t rev::MotorGroup::get_raw_position(
    std::uint32_t* const timestamp) const {
  return motors.at(0).get_raw_position(timestamp);
}

double rev::MotorGroup::get_position(void) const {
  return motors.at(0).get_position();
}

std::int32_t rev::MotorGroup::set_zero_position(const double position) const {
  for (Motor m : motors)
    m.set_zero_position(position);
}

std::int32_t rev::MotorGroup::tare_position(void) const {
  for (Motor m : motors)
    m.tare_position();
}

std::int32_t rev::MotorGroup::set_brake_mode(
    const motor_brake_mode_e_t mode) const {
  for (Motor m : motors)
    m.set_brake_mode(mode);
}

std::int32_t rev::MotorGroup::set_encoder_units(
    const motor_encoder_units_e_t units) const {
  for (Motor m : motors)
    m.set_encoder_units(units);
}

std::int32_t rev::MotorGroup::set_gearing(
    const motor_gearset_e_t gearset) const {
  for (Motor m : motors)
    m.set_gearing(gearset);
}

std::int32_t rev::MotorGroup::set_reversed(const bool reverse) {
  std::cout << "WARNING: set_reversed on rev::MotorGroup does not work"
            << std::endl;
}

rev::motor_brake_mode_e_t rev::MotorGroup::get_brake_mode(void) const {
  return motors.at(0).get_brake_mode();
}

rev::motor_encoder_units_e_t rev::MotorGroup::get_encoder_units(void) const {
  return motors.at(0).get_encoder_units();
}