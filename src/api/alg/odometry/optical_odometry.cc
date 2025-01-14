#include "rev/api/alg/odometry/optical_odometry.hh"
#include <cerrno>
#include <iostream>

namespace rev {

OpticalOdometry::OpticalOdometry(std::shared_ptr<OTOS> sensor, QLength ilongitudinal_offset, QLength ilateral_offset) : optical_sensor(sensor), longitudinal_offset(ilongitudinal_offset), lateral_offset(ilateral_offset) {
  
}

OdometryState OpticalOdometry::get_state() {
  current_position_mutex.take(TIMEOUT_MAX);
  OdometryState ret = current_position;
  current_position_mutex.give();
  return ret;
}

void OpticalOdometry::set_position(Position pos) {
  current_position_mutex.take(TIMEOUT_MAX);

  current_position.pos = pos;
  current_position.vel = {0 * inch / second, 0 * inch / second, 0 * radian / second};

  current_position_mutex.give();
}

void OpticalOdometry::reset_position() {
  this->set_position({0 * inch, 0 * inch, 0 * degree});
}

void OpticalOdometry::step() {  
  optical_sensor->update();

  current_position_mutex.take(TIMEOUT_MAX);

  current_position.pos.x = optical_sensor->get_x() * inch;
  current_position.pos.y = optical_sensor->get_y() * inch;
  current_position.pos.theta = optical_sensor->get_h() * degree;

  current_position_mutex.give();
}

} // namespace rev
