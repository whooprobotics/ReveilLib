#include "rev/api/alg/drive/correction/pilons_correction.hh"
#include <cmath>
#include <iostream>
#include <tuple>
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"

#define PI 3.1415926535
// Helper function, stolen from Nick Mertin
rev::QAngle nearAngle(rev::QAngle angle, rev::QAngle reference) {
  return rev::radian *
         (round((reference.get_value() - angle.get_value()) / (2 * PI)) *
              (2 * PI) +
          angle.get_value());
}

rev::PilonsCorrection::PilonsCorrection(double ikCorrection, QLength imaxError)
    : k_correction(ikCorrection), max_error(imaxError) {}

std::tuple<double, double> rev::PilonsCorrection::apply_correction(
    rev::OdometryState current_state,
    rev::Position target_state,
    Position start_state,
    QLength drop_early,
    std::tuple<double, double> powers) {
  // TODO: Find a more efficient way to calculate backwards drive handling
  Number xi_facing = cos(start_state.facing);
  Number yi_facing = sin(start_state.facing);

  // Find dot product of initial facing and initial offset. If this dot product
  // is negative, the target point is behind the robot and it needs to reverse
  // to get there.
  QLength initial_longitudinal_distance =
      xi_facing * (target_state.x - start_state.x) +
      yi_facing * (target_state.y - start_state.y);

  bool isBackwards = (initial_longitudinal_distance.get_value() < 0);

  // Now that we know if the robot is driving backwards, lets find its angle
  // This is the global line angle
  QAngle raw_line_angle =
      atan2(target_state.y - start_state.y, target_state.x - start_state.x);

  // This is the line angle, reversed if required, and then constrained to
  // (start angle)\pm 180 degrees
  QAngle start_line_angle =
      nearAngle(raw_line_angle - (isBackwards ? PI * radian : 0 * radian),
                start_state.facing);

  // How far off the robot is from the start_line_angle
  QAngle ang = current_state.pos.facing - start_line_angle;

  QLength tarposx = target_state.x - current_state.pos.x;
  QLength tarposy = target_state.y - current_state.pos.y;

  // Transform into coordinates in the coordinate system established by the
  // initial line angle
  QLength tarposxi =
      tarposx * cos(raw_line_angle) + tarposy * sin(raw_line_angle);
  QLength tarposyi =
      tarposx * -sin(raw_line_angle) + tarposy * cos(raw_line_angle);

  // Find how far left or right the point we are heading toward is from the
  // target point
  // TODO: Verify signs here
  QLength target_error = tarposyi + tan(ang) * tarposxi;

  QAngle correct_angle = atan2(target_state.y - current_state.pos.y,
                               target_state.x - current_state.pos.x);

  if (isBackwards)
    correct_angle += PI * radian;

  double correction =
      abs(target_error) > abs(max_error)
          ? k_correction *
                (nearAngle(correct_angle, current_state.pos.facing) -
                 current_state.pos.facing)
                    .get_value() *
                (isBackwards ? -1 : 1)
          : 0.0;

  if (correction > 0)
    return std::make_tuple(std::get<0>(powers),
                           std::get<1>(powers) * exp(-correction));
  else if (correction < 0)
    return std::make_tuple(std::get<0>(powers) * exp(correction),
                           std::get<1>(powers));
  else
    return powers;
}