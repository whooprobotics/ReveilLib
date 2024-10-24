#include "rev/api/alg/reckless/turn_segment.hh"
#include "api.h"
#include "rev/api/alg/drive/turn/turn.hh"
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
  angle_difference = angle_goal - initial_state.pos.theta;
  target_relative_original =
      angle_difference -
      360 * std::floor((angle_difference.convert(degree) + 180) / 360) * degree;

  angle_goal = initial_state.pos.theta + target_relative_original;

  target_relative =
      angle_difference -
      360 * std::floor((angle_difference.convert(degree) + 180) / 360) * degree;
  controller_state = TurnState::FULLPOWER;

  if (target_relative <
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

  angle_difference = angle_goal - state.pos.theta;

  // Check Current angle/angular velocity and set controller_state
  target_relative =
      angle_difference -
      360 * std::floor((angle_difference.convert(degree) + 180) / 360) * degree;
  // Remain in fullpower if we are currently in fullpower and we arent ready to
  // advance

  // edge case test if already at angle (having trouble in google test )
  if (controller_state == TurnState::FULLPOWER &&
      fabs(target_relative.convert(degree)) < 5.0) {
    SegmentStatus::next();
  }

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
    // printf("Setting brake\n");
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
                     1) {              // Check if brake_time ms has elapsed
        brake_start_time = -1;         // reset for next run
        return SegmentStatus::next();  // move onto next Segment
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