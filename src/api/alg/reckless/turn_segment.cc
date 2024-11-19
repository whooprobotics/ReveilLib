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

QAngle RecklessTurnSegment::getRelativeAngle(QAngle goal, QAngle currentAngle) {
  angle_difference = angle_goal - currentAngle;
  return angle_difference -
         360 * std::floor((angle_difference.convert(degree) + 180) / 360) *
             degree;
}

void RecklessTurnSegment::init(OdometryState initial_state) {
  // std::cout << "Initializing turn" << std::endl;
  target_relative_original =
      getRelativeAngle(angle_goal, initial_state.pos.theta);

  angle_goal = initial_state.pos.theta + target_relative_original;

  target_relative = target_relative_original;

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

  target_relative = getRelativeAngle(angle_goal, state.pos.theta);

  // edge case test if already at angle
  if (fabs(target_relative_original.convert(degree)) < 5.0) {
    SegmentStatus::next();
  }

  // 1 is unfinished, 0 is finished. Goes from 1->0 as the turn progresses
  angle_completion = (fabs(target_relative_original.convert(degree)) < 5.0)
                         ? 0
                         : fabs(target_relative.convert(degree) /
                                target_relative_original.convert(degree));

  std::cout << "Angle Completion: " << angle_completion << std::endl;
  // Set the turn state
  if (angle_completion < coast_coeff &&
      controller_state == TurnState::FULLPOWER) {
    std::cout << "Setting COAST" << std::endl;
    controller_state = TurnState::COAST;
  } else if (angle_completion < harsh_coeff &&
             controller_state == TurnState::COAST) {
    std::cout << "Setting BRAKE" << std::endl;
    controller_state = TurnState::BRAKE;
  }

  // ################## Return to Reckless controller robot movement on each

  switch (controller_state) {
    case TurnState::COAST:
      return SegmentStatus::drive(left_direction * coast_power,
                                  right_direction * coast_power);
      break;
    case TurnState::BRAKE:
      if (brake_start_time == -1) {
        // std::cout << "started brake" << std::endl;
        brake_start_time = pros::millis();
      } else if (brake_start_time < pros::millis() - brake_time ||
                 fabs(current_state.vel.angular.convert(degree / second)) <=
                     0.25) {    // Check if brake_time ms has elapsed
        brake_start_time = -1;  // reset for next run
        // std::cout << "finishing turn" << std::endl;
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