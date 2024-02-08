#include "rev/api/alg/odometry/two_rotation_inertial_odometry.hh"
#include <cerrno>
#include <iostream>
#include "pros/error.h"

namespace rev {
TwoRotationInertialOdometry::TwoRotationInertialOdometry(
    pros::Rotation ilongitudinal_sensor,
    pros::Rotation ilateral_sensor,
    pros::Imu iinertial,
    QLength ilongitudinal_wheel_diameter,
    QLength ilateral_wheel_diameter,
    QLength ilongitudinal_wheel_offset,
    QLength ilateral_wheel_offset)
    : longitudinal_sensor(ilongitudinal_sensor),
      lateral_sensor(ilateral_sensor),
      inertial(iinertial),
      longitudinal_wheel_diameter(ilongitudinal_wheel_diameter),
      lateral_wheel_diameter(ilateral_wheel_diameter),
      longitudinal_wheel_offset(ilongitudinal_wheel_offset),
      lateral_wheel_offset(ilateral_wheel_offset) {
  longitude_ticks_last = (double)(longitudinal_sensor.get_position()) / 100;
  latitude_ticks_last = (double)(lateral_sensor.get_position()) / 100;
  heading_ticks_init = inertial.get_heading();
  time_last = pros::millis();
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
  current_position.vel = {0 * inch / second, 0 * inch / second,
                          0 * radian / second};

  current_position_mutex.give();
}
void TwoRotationInertialOdometry::reset_position() {
  this->set_position({0 * inch, 0 * inch, 0 * degree});
}
void TwoRotationInertialOdometry::step() {
  double longitude_ticks = (double)(longitudinal_sensor.get_position()) / 100;
  double latitude_ticks = (double)(lateral_sensor.get_position()) / 100;
  double heading_ticks = inertial.get_heading();

  if (heading_ticks == PROS_ERR_F || inertial.is_calibrating()) {
    return;
  }

  if(!is_initialized) {
    longitude_ticks_last = (double)(longitudinal_sensor.get_position()) / 100;
    latitude_ticks_last = (double)(lateral_sensor.get_position()) / 100;
    heading_ticks_last = inertial.get_heading();
    heading_ticks_init = inertial.get_heading() - current_position.pos.theta.convert(degree);

    is_initialized = true;
    return;
  }

  int32_t time = pros::millis();

  // Take the mutex so we can make sure things don't get race conditioned
  current_position_mutex.take(TIMEOUT_MAX);

  // Get differences
  double d_longitudinal_ticks = longitude_ticks - longitude_ticks_last;
  double d_latitude_ticks = latitude_ticks - latitude_ticks_last;
  double d_heading_ticks = heading_ticks - heading_ticks_last;
  int32_t d_time = time - time_last;

  // Update last ticks
  longitude_ticks_last = longitude_ticks;
  latitude_ticks_last = latitude_ticks;
  heading_ticks_last = heading_ticks;
  time_last = time;

  if (heading_ticks == PROS_ERR_F) {
    current_position_mutex.give();
    return;
  }

  // Get global facing angle
  double facing = heading_ticks - heading_ticks_init;

  if (std::isnan(d_longitudinal_ticks))
    d_longitudinal_ticks = 0.0;

  if (std::isnan(d_latitude_ticks))
    d_latitude_ticks = 0.0;

  // Early exit/skip iteration if no changes
  if (d_longitudinal_ticks == 0.0 && d_latitude_ticks == 0.0 &&
      d_heading_ticks == 0.0) {
    current_position_mutex.give();
    return;
  }

  // Raw translation values
  QLength raw_fwd_translation =
      d_longitudinal_ticks / 360 * 3.1415926535 * longitudinal_wheel_diameter;
  QLength raw_right_translation =
      d_latitude_ticks / 360 * 3.1415926535 * lateral_wheel_diameter;

  // Adjusted local translation values
  QLength local_off_lat, local_off_long;

  // Normalize d_heading_ticks to [-180, 180] degrees
  d_heading_ticks =
      d_heading_ticks - 360 * std::floor((d_heading_ticks + 180) / 360);

  // Calculate the local arc-adjusted translation values
  if (d_heading_ticks != 0) {
    double dht_radian = d_heading_ticks * degree.convert(radian);
    double sindt2 = std::sin(dht_radian / 2);
    local_off_lat = 2 * sindt2 *
                    (raw_right_translation / dht_radian + lateral_wheel_offset);
    local_off_long =
        2 * sindt2 *
        (raw_fwd_translation / dht_radian + longitudinal_wheel_offset);
  } else {
    local_off_lat = raw_right_translation;
    local_off_long = raw_fwd_translation;
  }

  // Average angle
  double avga = facing - 0.5 * d_heading_ticks;
  avga -= 360 * std::floor((avga + 180) / 360);

  // Polar form for rotating
  QLength polar_r = rev::sqrt(local_off_lat * local_off_lat +
                              local_off_long * local_off_long);
  QAngle polar_angle =
      rev::atan2(local_off_long, local_off_lat) - avga * degree;

  // global offsets
  QLength dX = polar_r * sin(polar_angle);
  QLength dY = polar_r * cos(polar_angle);

  // vels
  QSpeed vX = dX / (d_time * millisecond);
  QSpeed vY = dY / (d_time * millisecond);
  QAngularSpeed w = d_heading_ticks * degree / (d_time * millisecond);

  // Constrain facing to +-180 degrees
  facing = facing - 360 * std::floor((facing + 180) / 360);

  // Update position information
  current_position.pos.x += dX;
  current_position.pos.y += dY;
  current_position.pos.theta = facing * degree;
  current_position.vel.xv = vX;
  current_position.vel.yv = vY;
  current_position.vel.angular = w;

  // Before exiting, release mutex
  current_position_mutex.give();
}
}  // namespace rev