#include "rev/api/alg/odometry/two_rotation_inertial_odometry.hh"

namespace rev {
OdometryState TwoRotationInertialOdometry::get_state() {
  current_position_mutex.take(20);
  Position ret = current_position;
  current_position_mutex.give();
  return ret;
}
void TwoRotationInertialOdometry::set_position(Position pos) {}
void TwoRotationInertialOdometry::reset_position() {}
void TwoRotationInertialOdometry::step() {}
}  // namespace rev