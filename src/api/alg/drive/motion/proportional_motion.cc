#include "rev/api/alg/drive/motion/proportional_motion.hh"
#include "rev/util/mathutil.hh"

#include <algorithm>
#include <cmath>

rev::ProportionalMotion::ProportionalMotion(double ipower, double ik_p)
    : power(fabs(ipower)), k_p(ik_p) {}

std::tuple<double, double> rev::ProportionalMotion::gen_powers(
    rev::OdometryState current_state,
    rev::Position target_state,
    Position start_state,
    QLength drop_early) {
  // Calculate the absolute angle from the robot's facing direction to the
  // target point
  QAngle angle_to_target = atan2(target_state.y - current_state.pos.y, 
      target_state.x - current_state.pos.x);
  // Calculate the difference between where the robot is facing and that angle
  QAngle err_a = current_state.pos.facing - angle_to_target;
  QLength distance_to_target =
      std::sqrt(std::pow(target_state.x.convert(inch) - current_state.pos.x.convert(inch), 2) +
           std::pow(target_state.y.convert(inch) - current_state.pos.y.convert(inch), 2)) * inch;

  // Scale down distance to just get the longitudinal component
  // apply drop_early term
  QLength err_y = cos(err_a) * (distance_to_target - drop_early);

  double finalPower = k_p * err_y.convert(inch);

  finalPower = std::clamp(finalPower, -std::abs(power), std::abs(power));

  return std::make_tuple(finalPower, finalPower);
}