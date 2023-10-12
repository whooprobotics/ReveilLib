#include "rev/api/alg/odometry/two_rotation_inertial_odometry.hh"

namespace rev {
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

  inertial.set_yaw(pos.facing.convert(degree));

  current_position_mutex.give();
}
void TwoRotationInertialOdometry::reset_position() {}
void TwoRotationInertialOdometry::step() {}
}  // namespace rev