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
  optical_sensor->update(); // get new readings from the optical sensor

  current_position_mutex.take(TIMEOUT_MAX);

  // update the OdometryState with the current sensor readings
  sensor_reading.pos.x = optical_sensor->get_x() * inch;
  sensor_reading.pos.y = optical_sensor->get_y() * inch;
  sensor_reading.pos.theta = optical_sensor->get_h() * degree;

  // calculate the difference in position in each dimension
  x_diff = sensor_reading.pos.x - last_sensor_reading.pos.x;
  y_diff = sensor_reading.pos.y - last_sensor_reading.pos.y;
  h_diff = sensor_reading.pos.theta - last_sensor_reading.pos.theta;

  // set the "last sensor reading" to the current one, for the next cycle of the step function
  last_sensor_reading.pos.x = sensor_reading.pos.x;
  last_sensor_reading.pos.y = sensor_reading.pos.y;
  last_sensor_reading.pos.theta = sensor_reading.pos.theta;

  // update the current position
  current_position.pos.x += x_diff;
  current_position.pos.y += y_diff;
  current_position.pos.theta += h_diff;

  current_position_mutex.give();
}

} // namespace rev
