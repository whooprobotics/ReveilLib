#include "rev/api/alg/drive/correction/pilons_correction.hh"
#include <cmath>
#include <iostream>
#include <tuple>
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"

#define PI 3.1415926535
// Helper function, stolen from Nick Mertin
rev::QAngle nearAngle(rev::QAngle angle, rev::QAngle reference)
{
	return rev::radian*(round((reference.get_value() - angle.get_value()) / (2 * PI)) * (2 * PI) + angle.get_value());
}

rev::PilonsCorrection::PilonsCorrection(double ikCorrection, QLength imaxError)
    : k_correction(ikCorrection), max_error(imaxError) {}

std::tuple<double, double> rev::PilonsCorrection::apply_correction(
    rev::OdometryState current_state,
    rev::Position target_state,
    Position start_state,
    QLength drop_early,
    std::tuple<double, double> powers) {
  /*// Dot product properties are nice
  Number x_dot = cos(current_state.pos.facing);
  Number y_dot = sin(current_state.pos.facing);

  // Backwards driving handling
  // If the final state is somewhere behind the start state, we need to invert the facing vector
  Number xi_facing = cos(start_state.facing);
  Number yi_facing = sin(start_state.facing);

  // Find dot product of initial facing and initial offset. If this dot product is negative, the target point is behind the robot and it needs to reverse to get there.
  QLength initial_longitudinal_distance = xi_facing * (target_state.x - start_state.x) + yi_facing * (target_state.y - start_state.y);


  char sgn_go = 1;

  // If its negative, we're goin backwards
  if(initial_longitudinal_distance.get_value() < 0) {
    x_dot = -x_dot;
    y_dot = -y_dot;
    sgn_go = -1;
    //std::cout << "backwardsing" << std::endl;
  }



  QLength tarposx = target_state.x - current_state.pos.x;
  QLength tarposy = target_state.y - current_state.pos.y;
  QLength tarposabs = sqrt(tarposx * tarposx + tarposy * tarposy);

  // Use <aob = a dot b / ||a||||b||
  Number cosang = (tarposx * x_dot + tarposy * y_dot) / tarposabs;

  // Similarly find the angle using vector rejection dot product
  // The unit vector 90 degrees greater than facing is (-y, x)
  QLength err_x = tarposy * x_dot - tarposx * y_dot;

  // Invert cosine to get actual angle
  QAngle ang = acos(cosang);

  if(abs(ang) > 3.141592653/2 * radian)
    std::cout << "Trying to turn too far, this is what you might call incorrect" << std::endl;

  // Account for if it is the other direction
  // AKA if we... need to turn left
  //if (ang > 3.1415926535 / 2 * radian)
  //  ang -= 3.1415926535 * radian;

  if (err_x < 0_m)
    ang = -ang;

  //char sgn_power = (std::get<0>(powers) + std::get<1>(powers)) >= 0 ? 1 : -1;


  */

  // Backwards driving handling
  // If the final state is somewhere behind the start state, we need to invert the facing vector
  Number xi_facing = cos(start_state.facing);
  Number yi_facing = sin(start_state.facing);

  // Find dot product of initial facing and initial offset. If this dot product is negative, the target point is behind the robot and it needs to reverse to get there.
  QLength initial_longitudinal_distance = xi_facing * (target_state.x - start_state.x) + yi_facing * (target_state.y - start_state.y);


  bool isBackwards = (initial_longitudinal_distance.get_value() < 0);

  QAngle angle_to_target_from_start = atan2(target_state.y - start_state.y, target_state.x - start_state.x);
  QAngle pid_angle = nearAngle(angle_to_target_from_start - (isBackwards ? PI*radian : 0*radian), current_state.pos.facing);
  QAngle ang = angle_to_target_from_start - current_state.pos.facing;

  QLength tarposx = target_state.x - current_state.pos.x;
  QLength tarposy = target_state.y - current_state.pos.y;

  // err_x is calculated in reference to initial pos angle, not current pos angle
  QLength err_x = tarposx * xi_facing - tarposy * yi_facing;

  QAngle correct_angle = atan2(target_state.y - current_state.pos.y, target_state.x - current_state.pos.x);

  if(isBackwards)
    correct_angle += PI*radian;


  


  double correction = abs(err_x) > abs(max_error)
                          ? -k_correction * 
                          (nearAngle(correct_angle, current_state.pos.facing) - current_state.pos.facing).get_value()
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