#include "rev/api/alg/drive/motion/proportional_motion.hh"
#include "rev/api/unit/unit.hh"
#include "rev/util/math.hh"

#include <algorithm>
#include <cmath>

rev::ProportionalMotion::ProportionalMotion(double ipower, double ik_p)
    : power(ipower), k_p(ik_p) {}

std::tuple<double, double> rev::ProportionalMotion::gen_powers(
    rev::OdometryState current_state,
    rev::Position target_state) {
  // Calculate the absolute angle from the robot's facing direction to the
  // target point
  Angle angle_to_target = atan2(target_state.x - current_state.pos.x,
                                target_state.y - current_state.pos.y);
  // Calculate the difference between where the robot is facing and that angle
  double err_a = current_state.pos.facing - angle_to_target;
  double distance_to_target =
      sqrt(pow(target_state.x - current_state.pos.x, 2) +
           pow(target_state.y - current_state.pos.y, 2));

  // Scale down distance to just get the longitudinal component
  double err_y = cos(err_a) * distance_to_target;

  double finalPower = k_p * err_y * sgn(power);

  finalPower = std::clamp(-power, finalPower, power);

  return std::make_tuple(finalPower, finalPower);
}