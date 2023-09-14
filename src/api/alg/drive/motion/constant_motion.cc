#include "rev/api/alg/drive/motion/constant_motion.hh"

rev::ConstantMotion::ConstantMotion(double ipower) : power(ipower) {}

std::tuple<double, double> rev::ConstantMotion::gen_powers(
    rev::OdometryState current_state,
    rev::Position target_state,
    Position start_state,
    QLength drop_earlys) {
  return std::make_tuple(power, power);
}