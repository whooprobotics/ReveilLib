#include "rev/api/hardware/motor/motor.hh"
#include <cmath>
rev::Motor::Motor(const std::int8_t port,
                  const motor_gearset_e_t gearset,
                  const bool reverse,
                  const motor_encoder_units_e_t encoder_units)
    : port(abs(port)), reversed(reverse) {
  set_encoder_units(encoder_units);
  set_gearing(gearset);
}

rev::Motor::Motor(const std::int8_t port,
                  const motor_gearset_e_t gearset,
                  const bool reverse)
    : port(abs(port)), reversed(reverse) {
  set_gearing(gearset);
}

rev::Motor::Motor(const std::int8_t port, const motor_gearset_e_t gearset)
    : port(abs(port)) {
  set_gearing(gearset);
  if (port < 0)
    set_reversed(true);
}

rev::Motor::Motor(const std::int8_t port, const bool reverse)
    : port(abs(port)), reversed(reverse) {}

rev::Motor::Motor(const std::int8_t port) : port(abs(port)) {
  if (port < 0)
    set_reversed(true);
}

std::int32_t rev::Motor::move(std::int32_t voltage) const {
  return pros::c::motor_move(port, reversed * voltage);
}

std::int32_t rev::Motor::move_absolute(const double position,
                                       const std::int32_t velocity) const {
  return pros::c::motor_move_absolute(port, reversed * position, velocity);
}

std::int32_t rev::Motor::move_relative(const double position,
                                       const std::int32_t velocity) const {
  return pros::c::motor_move_relative(port, reversed * position, velocity);
}

std::int32_t rev::Motor::move_velocity(const std::int32_t velocity) const {
  return pros::c::motor_move_velocity(port, velocity * reversed);
}

std::int32_t rev::Motor::move_voltage(const std::int32_t voltage) const {
  return pros::c::motor_move_voltage(port, voltage * reversed);
}

std::int32_t rev::Motor::brake(void) const {
  return pros::c::motor_brake(port);
}

std::int32_t rev::Motor::modify_profiled_velocity(
    const std::int32_t velocity) const {
  return pros::c::motor_modify_profiled_velocity(port, velocity * reversed);
}

double rev::Motor::get_actual_velocity(void) const {
  return pros::c::motor_get_actual_velocity(port) * reversed;
}

std::int32_t rev::Motor::get_direction(void) const {
  return pros::c::motor_get_direction(port) * reversed;
}

std::int32_t rev::Motor::is_stopped(void) const {
  return pros::c::motor_get_flags(port) & pros::E_MOTOR_FLAGS_ZERO_VELOCITY;
}

std::int32_t rev::Motor::get_raw_position(
    std::uint32_t* const timestamp) const {
  return pros::c::motor_get_raw_position(port, timestamp) * reversed;
}

double rev::Motor::get_position(void) const {
  return pros::c::motor_get_position(port) * reversed;
}

std::int32_t rev::Motor::set_zero_position(const double position) const {
  return pros::c::motor_set_zero_position(port, position * reversed);
}

std::int32_t rev::Motor::tare_position(void) const {
  return pros::c::motor_tare_position(port);
}

std::int32_t rev::Motor::set_brake_mode(const motor_brake_mode_e_t mode) const {
  return pros::c::motor_set_brake_mode(port, mode);
}

std::int32_t rev::Motor::set_encoder_units(
    const motor_encoder_units_e_t units) const {
  return pros::c::motor_set_encoder_units(port, units);
}

std::int32_t rev::Motor::set_gearing(const motor_gearset_e_t gearset) const {
  return pros::c::motor_set_gearing(port, gearset);
}

std::int32_t rev::Motor::set_reversed(const bool reverse) {
  reversed = reverse ? -1 : 1;
  return 0;
}

rev::motor_brake_mode_e_t rev::Motor::get_brake_mode(void) const {
  return pros::c::motor_get_brake_mode(port);
}

rev::motor_encoder_units_e_t rev::Motor::get_encoder_units(void) const {
  return pros::c::motor_get_encoder_units(port);
}

std::uint8_t rev::Motor::get_port(void) const {
  return port;
}