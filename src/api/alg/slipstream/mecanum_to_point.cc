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

  drivePID = PID(p.drive_k.p, p.drive_k.i, p.drive_k.d, p.drive_k.starti,
                 p.drive_settle.settle_error, p.drive_settle.settle_time,
                 p.drive_settle.large_settle_error,
                 p.drive_settle.large_settle_time, 0, p.timeout);

  turnPID = PID(p.turn_k.p, p.turn_k.i, p.turn_k.d, p.turn_k.starti,
                p.turn_settle.settle_error, p.turn_settle.settle_time,
                p.turn_settle.large_settle_error,
                p.turn_settle.large_settle_time, 0, p.timeout);

  // headingPID only corrects steer — no settling params needed
  headingPID =
      PID(p.heading_k.p, p.heading_k.i, p.heading_k.d, p.heading_k.starti);

  prev_line_settled = false;
}

SlipstreamSegmentStatus MecanumToPoint::step(OdometryState current_state) {
  // Coordinate convention matches mecanum_to_pose
  QLength x = target_point.y;
  QLength y = target_point.x;

  auto get_y = [&current_state]() { return current_state.pos.x; };
  auto get_x = [&current_state]() { return current_state.pos.y; };
  auto get_heading = [&current_state]() { return current_state.pos.theta; };

  // Only drive PID needs to settle — no target heading
  if (drivePID.is_settled()) {
    return last_status = SlipstreamSegmentStatus::next();
  }

  QAngle desired_heading = atan2(x - get_x(), y - get_y());

  bool line_settled =
      is_line_settled(x, y, desired_heading, get_x(), get_y(), p.exit_error);
  if (!(line_settled == prev_line_settled) && p.min_speed > 0)
    return last_status = SlipstreamSegmentStatus::next();
  prev_line_settled = line_settled;

  QLength drive_error = hypot(x - get_x(), y - get_y());

  // Angle robot must face to move toward target (degrees, for turn PID)
  QAngle turn_error_angle = reduce_negative_180_to_180(desired_heading - get_heading());

  // Zero turn correction when very close to avoid oscillation
  double turn_error = (drive_error < 6_in) ? 0.0 : turn_error_angle.convert(degree);

  QAngle center_angle_error = reduce_negative_180_to_180(desired_heading - get_heading());
  double heading_scale_factor = std::cos(center_angle_error.convert(radian));
  double center_heading_error = reduce_negative_90_to_90(center_angle_error.convert(degree));

  double drive_output = drivePID.compute(drive_error.convert(inch));
  double turn_output = turnPID.compute(turn_error);
  double center_drive_output = drive_output * heading_scale_factor;
  double center_heading_output = headingPID.compute(center_heading_error);

  drive_output = std::clamp(drive_output, -p.max_speed, p.max_speed);
  turn_output = std::clamp(turn_output, -p.turn_max_speed, p.turn_max_speed);
  center_drive_output = std::clamp(
      center_drive_output, -std::abs(heading_scale_factor) * p.center_max_speed,
      std::abs(heading_scale_factor) * p.center_max_speed);
  center_heading_output = std::clamp(center_heading_output, -p.center_max_speed,
                                     p.center_max_speed);

  drive_output = clamp_min_voltage(drive_output, p.min_speed);
  turn_output = clamp_min_voltage(turn_output, p.min_speed);
  center_drive_output = clamp_min_voltage(center_drive_output, p.min_speed);

  // Proportional scaling keeps left/right within bounds while preserving ratio
  double left_center_voltage =
      left_voltage_scaling(center_drive_output, center_heading_output);
  double right_center_voltage =
      right_voltage_scaling(center_drive_output, center_heading_output);

  // Field-centric mecanum mixing — heading_error in radians, same units as
  // heading_raw
  double heading_raw = get_heading().convert(radian);
  double heading_error_rad =
      std::atan2((y - get_y()).convert(inch), (x - get_x()).convert(inch));

  double left_front_output = (drive_output * std::cos(heading_raw + heading_error_rad - M_PI / 4)) + turn_output;
  double left_back_output = (drive_output * std::cos(-heading_raw - heading_error_rad + 3 * M_PI / 4)) + turn_output;
  double right_back_output = (drive_output * std::cos(heading_raw + heading_error_rad - M_PI / 4)) - turn_output;
  double right_front_output = (drive_output * std::cos(-heading_raw - heading_error_rad + 3 * M_PI / 4)) - turn_output;

  rev::SlipstreamPower power = {
      .front_left_forward = left_front_output,
      .front_right_forward = right_front_output,
      .rear_left_forward = left_back_output,
      .rear_right_forward = right_back_output,

      .front_left_steer = left_center_voltage,
      .front_right_steer = right_center_voltage,
  };

  return last_status = SlipstreamSegmentStatus::drive(power);
}

void MecanumToPoint::clean_up() {}

double MecanumToPoint::progress() {
  return part_progress;
}

}  // namespace rev
