#include "rev/api/alg/slipstream/mecanum_to_point.hh"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "rev/api/alg/PID/PID.hh"
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/units/q_length.hh"

namespace rev {

void MecanumToPoint::init(OdometryState initial_state) {
  std::cout << "MecanumToPoint segment invoked" << std::endl;
  this->start_point = initial_state.pos;
  this->last_status = SlipstreamSegmentStatus::drive({0, 0, 0, 0});

  QLength x = target_point.x;
  QLength y = target_point.y;
  QLength current_x = initial_state.pos.y;
  QLength current_y = initial_state.pos.x;

  desired_heading = atan2(x - current_x, y - current_y);
  heading_locked = false;
  locked_heading = 0_deg;

  drivePID = PID(p.drive_k.p, p.drive_k.i, p.drive_k.d, p.drive_k.starti,
                 p.drive_settle.settle_error, p.drive_settle.settle_time,
                 p.drive_settle.large_settle_error,
                 p.drive_settle.large_settle_time, 0, p.timeout);

  headingPID = PID(p.turn_k.p, p.turn_k.i, p.turn_k.d, p.turn_k.starti);

  prev_line_settled = false;
}

SlipstreamSegmentStatus MecanumToPoint::step(OdometryState current_state) {
  QLength x = target_point.x;
  QLength y = target_point.y;

  QLength current_y = current_state.pos.x;
  QLength current_x = current_state.pos.y;
  QAngle current_heading = current_state.pos.theta;

  if (drivePID.is_settled()) {
    return last_status = SlipstreamSegmentStatus::next();
  }

  desired_heading = atan2(x - current_x, y - current_y);

  bool line_settled =
      is_line_settled(x, y, desired_heading, current_x, current_y, p.exit_error);
      
  if (!(line_settled == prev_line_settled) && p.min_speed > 0)
    return last_status = SlipstreamSegmentStatus::next();
  prev_line_settled = line_settled;

  QLength drive_error = hypot(x - current_x, y - current_y);

  QAngle heading_error = reduce_negative_180_to_180(desired_heading - current_heading);
  double heading_scale_factor = std::cos(heading_error.convert(radian));

  double drive_output = drivePID.compute(drive_error.convert(inch));
  drive_output *= heading_scale_factor;

  if (drive_error < 6_in) {
    if (!heading_locked) {
      locked_heading = desired_heading;
      heading_locked = true;
    }
    heading_error = reduce_negative_180_to_180(locked_heading - current_heading);
  }

  double heading_error_deg = reduce_negative_90_to_90(heading_error.convert(degree));
  double heading_output = headingPID.compute(heading_error_deg);

  drive_output = std::clamp(drive_output,
                            -std::abs(heading_scale_factor) * p.max_speed,
                            std::abs(heading_scale_factor) * p.max_speed);
  heading_output = std::clamp(heading_output, -p.center_max_speed, p.center_max_speed);
  drive_output = clamp_min_voltage(drive_output, p.min_speed);

  double left_output = left_voltage_scaling(drive_output, heading_output);
  double right_output = right_voltage_scaling(drive_output, heading_output);

  rev::SlipstreamPower power = {
      .front_left_forward = left_output / 12,
      .front_right_forward = right_output / 12,
      .rear_left_forward = left_output / 12,
      .rear_right_forward = right_output / 12,

      .front_left_steer = left_output / 12,
      .front_right_steer = right_output / 12,
  };

  return last_status = SlipstreamSegmentStatus::drive(power);
}

void MecanumToPoint::clean_up() {}

double MecanumToPoint::progress() {
  return part_progress;
}

}  // namespace rev
