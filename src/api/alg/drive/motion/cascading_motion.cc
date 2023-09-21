#include "rev/api/alg/drive/motion/cascading_motion.hh"
#include "rev/util/mathutil.hh"

#include <algorithm>
#include <cmath>

std::tuple<double, double> rev::CascadingMotion::gen_powers(
    rev::OdometryState current_state,
    rev::Position target_state,
    Position start_state,
    QLength drop_early) {
  // Calculate the absolute angle from the robot's facing direction to the
  // target point
  QAngle angle_to_target = atan2(target_state.x - current_state.pos.x,
                                target_state.y - current_state.pos.y);
  // Calculate the difference between where the robot is facing and that angle
  QAngle err_a = current_state.pos.facing - angle_to_target;
  QLength distance_to_target =
      std::sqrt(std::pow(target_state.x.convert(inch) - current_state.pos.x.convert(inch), 2) +
           std::pow(target_state.y.convert(inch) - current_state.pos.y.convert(inch), 2)) * inch;

  // Scale down distance to just get the longitudinal component
  // apply drop_early term
  QLength err_y = cos(err_a) * (distance_to_target - drop_early);

  // Calculate target velocity
  QSpeed v_target = max_v * (1 - exp(k_v * (err_y - drop_early).convert(inch)));

  // Get longitudinal speed
  // Its just the dot product of the velocity vector and the facing unit vector
  QSpeed v = current_state.vel.xv * cos(current_state.pos.facing) + current_state.vel.yv * sin(current_state.pos.facing);

  double finalPower = (k_p * (v_target - v).convert(inch / second) + v_target.convert(inch / second) * k_v) * sgn(power);

  finalPower = std::clamp(finalPower, -power, power);

  return std::make_tuple(finalPower, finalPower);
}