#ifdef PLATFORM_BRAIN

#include "rev/api/alg/slipstream/look_at.hh"

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
    brake_time(ibrake_time),
    turn_segment(imax_power, icoast_power, 0 * degree, iharsh_coeff, icoast_coeff, ibrake_time) {}

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
    brake_time(ibrake_time),
    turn_segment(imax_power, icoast_power, 0 * degree, iharsh_coeff, icoast_coeff, ibrake_time) {
      timeout = (uint32_t)itimeout.convert(millisecond);
    }

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
    brake_time(ibrake_time),
    turn_segment(imax_power, icoast_power, 0 * degree, iharsh_coeff, icoast_coeff, ibrake_time) {}

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
    brake_time(ibrake_time),
    turn_segment(imax_power, icoast_power, 0 * degree, iharsh_coeff, icoast_coeff, ibrake_time) {
      timeout = (uint32_t)itimeout.convert(millisecond);
    }

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

  if (timeout) {
    turn_segment = RecklessTurnSegment(
        max_power, 
        coast_power, 
        angle_goal, 
        harsh_coeff, 
        coast_coeff, 
        brake_time,
        timeout * millisecond
    );
    turn_segment.init(initial_state);
  } else {
    turn_segment = RecklessTurnSegment(
        max_power, 
        coast_power, 
        angle_goal, 
        harsh_coeff, 
        coast_coeff, 
        brake_time
    );
    turn_segment.init(initial_state);
  }
}

SlipstreamSegmentStatus LookAt::step(OdometryState current_state) {
  SegmentStatus reckless_status = turn_segment.step(current_state);

  switch (reckless_status.status) {
    case SegmentStatusType::NEXT:
      return SlipstreamSegmentStatus::next();
    case SegmentStatusType::BRAKE:
      return SlipstreamSegmentStatus::brake();
    case SegmentStatusType::DUMMY:
      return SlipstreamSegmentStatus::dummy();
    case SegmentStatusType::DRIVE:
    default:
      return SlipstreamSegmentStatus::drive({
        reckless_status.power_left,
        reckless_status.power_right,
        reckless_status.power_left,
        reckless_status.power_right
      });
  }
}

void LookAt::clean_up() {
  turn_segment.clean_up();
}

} // namespace rev

#endif