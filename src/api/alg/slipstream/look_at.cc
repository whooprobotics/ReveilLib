#ifdef PLATFORM_BRAIN

#include "rev/api/alg/slipstream/look_at.hh"
#include "api.h"

namespace rev {

LookAt::LookAt(
        double imax_power, 
        double icoast_power, 
        Position itarget_position, 
        QAngle idrop_angle, 
        double iharsh_coeff, 
        double icoast_coeff, 
        QTime ibrake_time) 
  : max_power(imax_power),
    coast_power(icoast_power),
    target_position(itarget_position),
    drop_angle(idrop_angle),
    harsh_coeff(iharsh_coeff),
    coast_coeff(icoast_coeff),
    brake_time(ibrake_time.convert(millisecond)) {}

LookAt::LookAt(
        double imax_power, 
        double icoast_power, 
        Position itarget_position, 
        QAngle idrop_angle, 
        double iharsh_coeff, 
        double icoast_coeff, 
        QTime ibrake_time,
        QTime itimeout) 
  : max_power(imax_power),
    coast_power(icoast_power),
    target_position(itarget_position),
    drop_angle(idrop_angle),
    harsh_coeff(iharsh_coeff),
    coast_coeff(icoast_coeff),
    brake_time(ibrake_time.convert(millisecond)),
    timeout((uint32_t)itimeout.convert(millisecond)) {}

LookAt::LookAt(
        double imax_power, 
        double icoast_power, 
        Position itarget_position, 
        QAngle iangle_offset,
        QAngle idrop_angle, 
        double iharsh_coeff, 
        double icoast_coeff, 
        QTime ibrake_time) 
  : max_power(imax_power),
    coast_power(icoast_power),
    target_position(itarget_position),
    angle_offset(iangle_offset),
    drop_angle(idrop_angle),
    harsh_coeff(iharsh_coeff),
    coast_coeff(icoast_coeff),
    brake_time(ibrake_time.convert(millisecond)) {}

LookAt::LookAt(
        double imax_power, 
        double icoast_power, 
        Position itarget_position, 
        QAngle iangle_offset,
        QAngle idrop_angle, 
        double iharsh_coeff, 
        double icoast_coeff, 
        QTime ibrake_time,
        QTime itimeout) 
  : max_power(imax_power),
    coast_power(icoast_power),
    target_position(itarget_position),
    angle_offset(iangle_offset),
    drop_angle(idrop_angle),
    harsh_coeff(iharsh_coeff),
    coast_coeff(icoast_coeff),
    brake_time(ibrake_time.convert(millisecond)),
    timeout((uint32_t)itimeout.convert(millisecond)) {}

void LookAt::init(OdometryState initial_state) {
  start_position = initial_state.pos;

  double computed_angle = atan2(
    target_position.y - start_position.y,
    target_position.x - start_position.x
  ).convert(degree);

  computed_angle = computed_angle - 360.0 * std::floor((computed_angle + 180.0) / 360.0);

  double angle1 = computed_angle - drop_angle.convert(degree);
  double angle2 = computed_angle + drop_angle.convert(degree);

  angle1 = angle1 - 360.0 * std::floor((angle1 + 180.0) / 360.0);
  angle2 = angle2 - 360.0 * std::floor((angle2 + 180.0) / 360.0);

  double offset1 = angle1 - start_position.theta.convert(degree);
  double offset2 = angle2 - start_position.theta.convert(degree);

  offset1 = offset1 - 360.0 * std::floor((offset1 + 180.0) / 360.0);
  offset2 = offset2 - 360.0 * std::floor((offset2 + 180.0) / 360.0);

  angle_goal = (fabs(offset1) < fabs(offset2))
    ? angle1 * degree : angle2 * degree;

  double norm = angle_goal.convert(degree) + angle_offset.convert(degree);
  norm = norm - 360.0 * std::floor((norm + 180.0) / 360.0);
  angle_goal = norm * degree;

  angle_goal = angle_goal - 360 * std::floor((angle_goal.convert(degree) + 180) / 360) * degree;

  angle_difference = angle_goal - initial_state.pos.theta;

  target_relative_original =
    (angle_difference.convert(degree) -
     360 * std::floor((angle_difference.convert(degree) + 180) / 360)) * degree;

  target_relative = target_relative_original;

  controller_state = TurnState::FULLPOWER;
  brake_start_time = -1;

  spin_direction = (target_relative_original < 0 * degree) ? -1 : 1;
}

SlipstreamSegmentStatus LookAt::step(OdometryState current_state) {
  angle_difference =
    angle_goal -
    (current_state.pos.theta -
     360 * std::floor((current_state.pos.theta.convert(degree) + 180) / 360) * degree);

  target_relative =
    (angle_difference.convert(degree) -
     360 * std::floor((angle_difference.convert(degree) + 180) / 360)) * degree;

  if (fabs(target_relative_original.convert(degree)) < 5.0)
    return SlipstreamSegmentStatus::next();

  if (std::copysign(1.0, target_relative.get_value()) !=
      std::copysign(1.0, target_relative_original.get_value()))
    return SlipstreamSegmentStatus::next();

  if (timeout && pros::millis() >= timeout)
    return SlipstreamSegmentStatus::next();

  if (fabs(target_relative.convert(degree)) <=
          fabs(current_state.vel.angular.convert(degree / second) * coast_coeff) &&
      controller_state != TurnState::BRAKE &&
      controller_state != TurnState::COAST) {
    controller_state = TurnState::COAST;
  }

  if (fabs(target_relative.convert(degree)) 
          fabs(current_state.vel.angular.convert(degree / second) * harsh_coeff) &&
      controller_state != TurnState::BRAKE) {
    controller_state = TurnState::BRAKE;
  }

  auto make_turn_power = [&](double power) -> SlipstreamSegmentStatus {
    return SlipstreamSegmentStatus::drive({
      power * spin_direction,
      power * -spin_direction,
      power * -spin_direction,
      power * spin_direction
    });
  };

  switch (controller_state) {
    case TurnState::COAST:
      return make_turn_power(coast_power);
    case TurnState::BRAKE:
      if (brake_start_time == -1) {
        brake_start_time = pros::millis();
      } else if (brake_start_time < (int)(pros::millis() - brake_time) ||
                 fabs(current_state.vel.angular.convert(degree / second)) <= 0.25) {
        brake_start_time = -1;
        return SlipstreamSegmentStatus::next();
      }
      return SlipstreamSegmentStatus::brake();
    case TurnState::FULLPOWER:
    default:
      return make_turn_power(max_power);
  }
}

void LookAt::clean_up() {}

}  // namespace rev

#endif