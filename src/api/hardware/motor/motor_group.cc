#include <cmath>
#include "rev/api/hardware/motor/motor_group.hh"

namespace rev {


MotorGroup::MotorGroup(const std::initializer_list<Motor> motors)
    : motors(motors), motor_count(motors.size()) {}
MotorGroup::MotorGroup(const std::vector<Motor>& motors)
    : motors(motors), motor_count(motors.size()) {}
MotorGroup::MotorGroup(
    const std::initializer_list<std::int8_t> motor_ports)
    : motor_count(motor_ports.size()) {
  for (auto& i : motor_ports) {
    motors.emplace_back(Motor(i));
  }
}

MotorGroup::MotorGroup(const std::vector<std::int8_t> motor_ports)
    : motor_count(motor_ports.size()) {
  for (auto& i : motor_ports) {
    motors.emplace_back(Motor(i));
  }
}

std::int32_t MotorGroup::move(std::int32_t voltage) const {
  for (Motor m : motors)
    m.move(voltage);
  return 0;
}

std::int32_t MotorGroup::move_absolute(const double position,
                                            const std::int32_t velocity) const {
  for (Motor m : motors)
    m.move_absolute(position, velocity);
  return 0;
}

std::int32_t MotorGroup::move_relative(const double position,
                                            const std::int32_t velocity) const {
  for (Motor m : motors)
    m.move_relative(position, velocity);
  return 0;
}

std::int32_t MotorGroup::move_velocity(const std::int32_t velocity) const {
  for (Motor m : motors)
    m.move_velocity(velocity);
  return 0;
}

std::int32_t MotorGroup::move_voltage(const std::int32_t voltage) const {
  for (Motor m : motors)
    m.move_voltage(voltage);
  return 0;
}

std::int32_t MotorGroup::brake(void) const {
  for (Motor m : motors)
    m.brake();
  return 0;
}

std::int32_t MotorGroup::modify_profiled_velocity(
    const std::int32_t velocity) const {
  for (Motor m : motors)
    m.modify_profiled_velocity(velocity);
  return 0;
}

double MotorGroup::get_actual_velocity(void) const {
  return motors.at(0).get_actual_velocity();
}

std::int32_t MotorGroup::get_direction(void) const {
  return motors.at(0).get_direction();
}

std::int32_t MotorGroup::is_stopped(void) const {
  return motors.at(0).is_stopped();
}

std::int32_t MotorGroup::get_raw_position(
    std::uint32_t* const timestamp) const {
  return motors.at(0).get_raw_position(timestamp);
}

double MotorGroup::get_position(void) const {
  return motors.at(0).get_position();
}

std::int32_t MotorGroup::set_zero_position(const double position) const {
  for (Motor m : motors)
    m.set_zero_position(position);
  return 0;
}

std::int32_t MotorGroup::tare_position(void) const {
  for (Motor m : motors)
    m.tare_position();
  return 0;
}

std::int32_t MotorGroup::set_brake_mode(
    const motor_brake_mode_e_t mode) const {
  for (Motor m : motors)
    m.set_brake_mode(mode);
  return 0;
}

std::int32_t MotorGroup::set_encoder_units(
    const motor_encoder_units_e_t units) const {
  for (Motor m : motors)
    m.set_encoder_units(units);
  return 0;
}

std::int32_t MotorGroup::set_gearing(
    const motor_gearset_e_t gearset) const {
  for (Motor m : motors)
    m.set_gearing(gearset);
  return 0;
}

std::int32_t MotorGroup::set_reversed(const bool reverse) {
  std::cout << "WARNING: set_reversed on rev::MotorGroup does not work"
            << std::endl;
  return 1;
}

motor_brake_mode_e_t MotorGroup::get_brake_mode(void) const {
  return motors.at(0).get_brake_mode();
}

motor_encoder_units_e_t MotorGroup::get_encoder_units(void) const {
  return motors.at(0).get_encoder_units();
}

double MotorGroup::get_temperature(void) const {
  double maxTemp = motors.at(0).get_temperature();
  for (Motor m : motors)
    if(m.get_temperature() > maxTemp)
      maxTemp = m.get_temperature();
  return maxTemp;
}

std::uint8_t MotorGroup::check_ports(void) const {
  for (Motor m : motors) {
    if (std::abs(m.get_actual_velocity()) == PROS_ERR_F) {
      return m.get_port(); 
    }
  }
  return 0;
}

}  // namespace rev