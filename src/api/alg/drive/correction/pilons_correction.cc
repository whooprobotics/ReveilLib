#include "rev/api/alg/drive/correction/pilons_correction.hh"
#include <cmath>
#include <iostream>
#include <tuple>
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"

#define PI 3.1415926535
// Helper function, stolen from Nick Mertin
/**
 * @brief Helper function which constrains angle to within 180 degrees of reference
*/
rev::QAngle near_circle(rev::QAngle angle, rev::QAngle reference) {
  return rev::radian *
         (round((reference.get_value() - angle.get_value()) / (2 * PI)) *
              (2 * PI) +
          angle.get_value());
}

/**
 * @brief Helper function which constrains angle to within 90 degrees of reference
 * 
 * @param angle 
 * @param reference 
 * @return rev::QAngle 
 */
rev::QAngle near_semicircle(rev::QAngle angle, rev::QAngle reference) {
  return rev::radian *
    (round((reference.get_value() - angle.get_value()) / PI) *
      PI + angle.get_value());
}

rev::PilonsCorrection::PilonsCorrection(double ikCorrection, QLength imaxError)
    : k_correction(ikCorrection), max_error(imaxError) {}

std::tuple<double, double> rev::PilonsCorrection::apply_correction(
    rev::OdometryState current_state,
    rev::Position target_state,
    Position start_state,
    QLength drop_early,
    std::tuple<double, double> powers) {
  /*/ Backwards driving handling
  // If the final state is somewhere behind the start state, we need to invert
  // the facing vector

  // TODO: Find a more efficient way to calculate backwards drive handling
  Number xi_facing = cos(start_state.theta);
  Number yi_facing = sin(start_state.theta);

  // Find dot product of initial facing and initial offset. If this dot product
  // is negative, the target point is behind the robot and it needs to reverse
  // to get there.
  QLength initial_longitudinal_distance =
      xi_facing * (target_state.x - start_state.x) +
      yi_facing * (target_state.y - start_state.y);

  bool isBackwards = (initial_longitudinal_distance.get_value() < 0);

  QAngle angle_to_target_from_start =
      atan2(target_state.y - start_state.y, target_state.x - start_state.x);
  QAngle pid_angle = near_circle(
      angle_to_target_from_start - (isBackwards ? PI * radian : 0 * radian),
      current_state.pos.theta);
  QAngle ang = angle_to_target_from_start - current_state.pos.theta;

  QLength tarposx = target_state.x - current_state.pos.x;
  QLength tarposy = target_state.y - current_state.pos.y;

  // err_x is calculated in reference to initial pos angle, not current pos
  // angle
  QLength err_x = tarposx * xi_facing - tarposy * yi_facing;

  QAngle correct_angle = atan2(target_state.y - current_state.pos.y,
                               target_state.x - current_state.pos.x);

  if (isBackwards)
    correct_angle += PI * radian;

  double correction =
      abs(err_x) > abs(max_error)
          ? k_correction *
                (near_circle(correct_angle, current_state.pos.theta) -
                 current_state.pos.theta)
                    .get_value() *
                (isBackwards ? -1 : 1)
          : 0.0;*/

  // TODO: Refactor code
  Pose pos_current = current_state.pos;

  // Find the pose which is at target_state
  Pose pos_final = target_state;
  // but make this reference frame face directly away from the start state
  pos_final.theta = atan2(pos_final.y - start_state.y, pos_final.x - start_state.x);

  // If the robot starts facing more than 90 degrees from that, flip it to face towards the robot
  pos_final.theta = near_semicircle(pos_final.theta, start_state.theta);

  // Reframe the robots current position in reference to the target state
  Pose error = pos_current.to_relative(pos_final);

  // The angle from the perspective of the target point
  // We subtract from the facing angle so we have the actual error angle
  QAngle error_angle = -error.theta + atan2(error.y, error.x);

  error_angle = near_semicircle(error_angle, 0_deg);

  double correction = abs(error.y + error.x * tan(error.theta)) > abs(max_error) ? k_correction * error_angle.get_value() : 0.0;

  if(std::get<0>(powers) < 0)
    correction = -correction;

  if (correction > 0)
    return std::make_tuple(std::get<0>(powers),
                           std::get<1>(powers) * exp(-correction));
  else if (correction < 0)
    return std::make_tuple(std::get<0>(powers) * exp(correction),
                           std::get<1>(powers));
  else
    return powers;

}