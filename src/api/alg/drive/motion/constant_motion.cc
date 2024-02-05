#include "rev/api/alg/drive/motion/constant_motion.hh"

rev::ConstantMotion::ConstantMotion(double ipower) : power(fabs(ipower)) {}

std::tuple<double, double> rev::ConstantMotion::gen_powers(
    rev::OdometryState current_state,
    rev::Position target_state,
    Position start_state,
    QLength drop_earlys) {
  Number xi_facing = cos(start_state.theta);
  Number yi_facing = sin(start_state.theta);

  // Find dot product of initial facing and initial offset. If this dot product
  // is negative, the target point is behind the robot and it needs to reverse
  // to get there.
  QLength initial_longitudinal_distance =
      xi_facing * (target_state.x - start_state.x) +
      yi_facing * (target_state.y - start_state.y);

  bool isBackwards = (initial_longitudinal_distance.get_value() < 0);

  double opower = isBackwards ? -power : power;

  return std::make_tuple(opower, opower);
}