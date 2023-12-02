#include "rev/api/alg/drive/correction/pilons_correction.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/units/q_angle.hh"
#include <cmath>
#include <tuple>

rev::PilonsCorrection::PilonsCorrection(double ikCorrection, QLength imaxError)
    : k_correction(ikCorrection), max_error(imaxError) {}

std::tuple<double, double> rev::PilonsCorrection::apply_correction(
    rev::OdometryState current_state,
    rev::Position target_state,
      Position start_state,
      QLength drop_early,
    std::tuple<double, double> powers) {
  QAngle angle_to_target = atan2(target_state.y - current_state.pos.y, target_state.x - current_state.pos.x);
  // Find the x error (how far the target point is from the line the robot is
  // traveling along) This is a bit different from the PiLons
  QAngle err_a = current_state.pos.facing - angle_to_target;
  QLength distance_to_target =
      std::sqrt(
        std::pow(target_state.x.convert(inch) - current_state.pos.x.convert(inch), 2) +
        std::pow(target_state.y.convert(inch) - current_state.pos.y.convert(inch), 2)) 
           * inch;

  // Use trig to find that other side. sin = opposite / hypotenuse, so opposite
  // = sin * hypotenuse
  QLength err_x = rev::sin(err_a) * distance_to_target;

  // Calculate power sign
  char sgn_power = (std::get<0>(powers) + std::get<1>(powers)) >= 0 ? 1 : -1;

  // If driving in reverse, the angle to target is 180 degrees greater, or pi
  // radians
  if (sgn_power == -1)
    angle_to_target += 3.141592653*radian;

  // Find the absolute smallest angle difference between where the robot is
  // facing and where the robot needs to face
  QAngle correction_angle =
      (std::round((current_state.pos.facing - angle_to_target).convert(radian) / (2 * 3.141592653)) *
          (2 * 3.141592653) +
      (angle_to_target - current_state.pos.facing).convert(radian)) * radian;

  // Calculate correction factor
  double correction =
      err_x > max_error ? k_correction * correction_angle.convert(radian) * sgn_power : 0.0;

  if (correction > 0)
    return std::make_tuple(std::get<0>(powers),
                           std::get<1>(powers) * exp(-correction));
  else if (correction < 0)
    return std::make_tuple(std::get<0>(powers) * exp(correction),
                           std::get<1>(powers));
  else
    return powers;
}