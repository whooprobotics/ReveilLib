#include "rev/api/alg/odometry/two_rotation_inertial_odometry.hh"

namespace rev {
TwoRotationInertialOdometry::TwoRotationInertialOdometry(
    pros::Rotation ilongitudinal_sensor,
    pros::Rotation ilateral_sensor,
    pros::Imu iinertial)
    : longitudinal_sensor(ilongitudinal_sensor),
      lateral_sensor(ilateral_sensor),
      inertial(iinertial) {
  longitude_ticks_last = (double)(longitudinal_sensor.get_position()) / 100;
  latitude_ticks_last = (double)(lateral_sensor.get_position()) / 100;
  heading_ticks_init = inertial.get_heading();
}

OdometryState TwoRotationInertialOdometry::get_state() {
  current_position_mutex.take(TIMEOUT_MAX);
  OdometryState ret = current_position;
  current_position_mutex.give();
  return ret;
}
void TwoRotationInertialOdometry::set_position(Position pos) {
  current_position_mutex.take(TIMEOUT_MAX);

  current_position.pos = pos;
  current_position.vel = {0 * inch / second, 0 * inch / second};

  heading_ticks_init = inertial.get_heading() - pos.facing.convert(degree);

  current_position_mutex.give();
}
void TwoRotationInertialOdometry::reset_position() {
  this->set_position({0 * inch, 0 * inch, 0 * degree});
}
void TwoRotationInertialOdometry::step() {
  double longitude_ticks = (double)(longitudinal_sensor.get_position()) / 100;
  double latitude_ticks = (double)(lateral_sensor.get_position()) / 100;
  double heading_ticks = inertial.get_heading();

  // Get differences
  double d_longitudinal_ticks = longitude_ticks - longitude_ticks_last;
  double d_latitude_ticks = latitude_ticks - latitude_ticks_last;

  // Update last ticks
  longitude_ticks_last = longitude_ticks;
  latitude_ticks_last = latitude_ticks;

  // Get global facing angle
  double facing = heading_ticks - heading_ticks_init;
}
}  // namespace rev