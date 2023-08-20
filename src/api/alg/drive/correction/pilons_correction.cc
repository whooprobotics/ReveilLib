#include "rev/api/alg/drive/correction/pilons_correction.hh"
#include <cmath>
#include <tuple>

rev::PilonsCorrection::PilonsCorrection(double ikCorrection, Length imaxError)
    : kCorrection(ikCorrection), maxError(imaxError) {}

std::tuple<double, double> rev::PilonsCorrection::apply_correction(
    rev::OdometryState current_state,
    rev::Position target_state,
    std::tuple<double, double> powers) {
  Angle angle_to_target = atan2(target_state.x - current_state.pos.x,
                                target_state.y - current_state.pos.y);
  // Find the x error (how far the target point is from the line the robot is
  // traveling along) This is a bit different from the PiLons
  double err_a = current_state.pos.facing - angle_to_target;
  double distance_to_target =
      sqrt(pow(target_state.x - current_state.pos.x, 2) +
           pow(target_state.y - current_state.pos.y, 2));

  // Use trig to find that other side. sin = opposite / hypotenuse, so opposite
  // = sin * hypotenuse
  double err_x = sin(err_a) * distance_to_target;

  // Calculate power sign
  char sgn_power = (std::get<0>(powers) + std::get<1>(powers)) >= 0 ? 1 : -1;

  // If driving in reverse, the angle to target is 180 degrees greater, or pi
  // radians
  if (sgn_power == -1)
    angle_to_target += M_PI;

  // Find the absolute smallest angle difference between where the robot is
  // facing and where the robot needs to face
  Angle correction_angle =
      round((current_state.pos.facing - angle_to_target) / (2 * M_PI)) *
          (2 * M_PI) +
      angle_to_target - current_state.pos.facing;

  // Calculate correction factor
  double correction = kCorrection * correction_angle * sgn_power;
}