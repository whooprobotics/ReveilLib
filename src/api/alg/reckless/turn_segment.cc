#include "rev/api/alg/reckless/turn_segment.hh"
#include "api.h"
#include "rev/api/alg/reckless/segment.hh"
#include "rev/api/units/q_time.hh"

namespace rev {

RecklessTurnSegment::RecklessTurnSegment(double imax_power,
                                         double icoast_power,
                                         QAngle iangle,
                                         double iharsh_coeff,
                                         double icoast_coeff,
                                         QTime ibrake_time)
    : max_power(imax_power),
      coast_power(icoast_power),
      harsh_coeff(iharsh_coeff),
      coast_coeff(icoast_coeff),
      angle_goal(iangle),
      brake_time(ibrake_time.convert(millisecond)) {}

void RecklessTurnSegment::init(OdometryState initial_state) {
  start_angle = initial_state.pos.theta;  // normalized (-180 to 180)
  start_angle = start_angle -
                360 * std::floor((start_angle.convert(degree) + 180) / 360) *
                    degree;  // for google tests

  // convert given goal to be normalized (-180 to 180)
  // will move the heading to MATCH this angle
  angle_goal =
      angle_goal -
      360 * std::floor((angle_goal.convert(degree) + 180) / 360) * degree;

  angle_difference = angle_goal - initial_state.pos.theta;

  target_relative_original =
      (angle_difference.convert(degree) -
       360 * std::floor((angle_difference.convert(degree) + 180) / 360)) *
      degree;

  target_relative = target_relative_original;

  controller_state = TurnState::FULLPOWER;

  if (target_relative_original <
      0 * degree) {  // if direction is negative, flip directions
    left_direction = -1;
    right_direction = 1;
  } else {
    left_direction = 1;
    right_direction = -1;
  }
}

SegmentStatus RecklessTurnSegment::step(OdometryState current_state) {
  OdometryState state = current_state;

  // #################### Determine Turn State ####################

  // normalize state.pos.theta for google tests
  // angle_difference = angle_goal - state.pos.theta;
  angle_difference =
      angle_goal -
      (state.pos.theta -
       360 * std::floor((state.pos.theta.convert(degree) + 180) / 360) *
           degree);
  target_relative =
      (angle_difference.convert(degree) -
       360 * std::floor((angle_difference.convert(degree) + 180) / 360)) *
      degree;

  // Check Current angle/angular velocity compared to distance from target and
  // set controller_state

  // Remain in fullpower if we are currently in fullpower and we arent ready to
  // advance

  // edge case test if already at angle (having trouble in google test )
  if (fabs(target_relative_original.convert(degree)) < 5.0) {
    SegmentStatus::next();
  }

  //overshoot protection
  if(std::copysign(1.0, target_relative.get_value()) != std::copysign(1.0, target_relative_original.get_value()))
    return SegmentStatus::next();

  // Start slowdown if we are ready for that and we haven't already started
  // harsh-braking
  if (fabs(target_relative.convert(degree)) <=
          fabs(current_state.vel.angular.convert(degree / second) *
               coast_coeff) &&
      controller_state != TurnState::BRAKE &&
      controller_state != TurnState::COAST) {
    controller_state = TurnState::COAST;
  }
  // Harsh-brake if we're at that point
  if (fabs(target_relative.convert(degree)) <
          fabs(current_state.vel.angular.convert(degree / second) *
               harsh_coeff) &&
      controller_state != TurnState::BRAKE) {
    controller_state = TurnState::BRAKE;
  }

  // ################## Return to Reckless controller robot movement on each
  // step ##################

  switch (controller_state) {
    case TurnState::COAST:
      return SegmentStatus::drive(left_direction * coast_power,
                                  right_direction * coast_power);
      break;
    case TurnState::BRAKE:
      if (brake_start_time == -1) {
        brake_start_time = pros::millis();
      } else if (brake_start_time < pros::millis() - brake_time ||
                 fabs(current_state.vel.angular.convert(degree / second)) <=
                     0.25) {           // Check if brake_time ms has elapsed
        brake_start_time = -1;         // reset for next run
        return SegmentStatus::next();  // move onto next Segment
        break;                         // Drew told me to
      }
      return SegmentStatus::brake();
      break;
    case TurnState::FULLPOWER:
    default:
      return SegmentStatus::drive(max_power * left_direction,
                                  max_power * right_direction);
  }
}

void RecklessTurnSegment::clean_up() {}

}  // namespace rev