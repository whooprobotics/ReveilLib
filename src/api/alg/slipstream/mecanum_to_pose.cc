#include "rev/api/alg/slipstream/mecanum_to_pose.hh"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "rev/api/alg/PID/PID.hh"
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/alg/stop/stop.hh"
#include "rev/api/units/q_length.hh"

namespace rev {

void MecanumToPose::init(OdometryState initial_state) {
  std::cout << "MecanumToPose segment invoked" << std::endl;
  this->start_point = initial_state.pos;
  this->last_status = SlipstreamSegmentStatus::drive({0, 0, 0, 0});

  PID drivePID(p.drive_k.p, p.drive_k.i, p.drive_k.d,
               p.drive_k.starti,  // PID constants
               p.drive_settle.settle_error, p.drive_settle.settle_time,
               p.drive_settle.large_settle_error,
               p.drive_settle.large_settle_time,  // Settling parameters
               0, p.timeout                       // Exit parameters
  );

  PID turnPID(p.turn_k.p, p.turn_k.i, p.turn_k.d,
              p.turn_k.starti,  // PID constants
              p.turn_settle.settle_error, p.turn_settle.settle_time,
              p.turn_settle.large_settle_error,
              p.turn_settle.large_settle_time,  // Settling parameters
              0, p.timeout                      // Exit parameters
  );
  prev_line_settled = false;
}

SlipstreamSegmentStatus MecanumToPose::step(OdometryState current_state) {
  QLength x = target_point.y;
  QLength y = target_point.x;
  QAngle angle = target_point.theta;

  auto get_y = [&current_state]() { return current_state.pos.x; };
  auto get_x = [&current_state]() { return current_state.pos.y; };
  auto get_heading = [&current_state]() { return current_state.pos.theta; };

  if ((drivePID.is_settled() && turnPID.is_settled())) {
    return last_status = SlipstreamSegmentStatus::next();
  }

  QAngle desired_heading = atan2(x - get_x(), y - get_y());

  bool line_settled = is_line_settled(x, y, desired_heading, get_x(), get_y(), p.exit_error);

  if (!(line_settled == prev_line_settled) && p.min_speed > 0) return last_status = SlipstreamSegmentStatus::next();
  prev_line_settled = line_settled;


  QLength drive_error = hypot(x - get_x(), y - get_y());
  QAngle turn_error = reduce_negative_180_to_180(angle - get_heading());

  double drive_output = drivePID.compute(drive_error.convert(inch));
  double turn_output = turnPID.compute(turn_error.convert(degree));

  drive_output = std::clamp(drive_output, -p.max_speed, p.max_speed);
  turn_output = std::clamp(turn_output, -p.turn_max_speed, p.turn_max_speed);

  drive_output = clamp_min_voltage(drive_output, p.min_speed);
  turn_output = clamp_min_voltage(turn_output, p.min_speed);

  QAngle heading_error = atan2(y - get_y(), x - get_x());

  double raw_heading_error = heading_error.convert(degree);
  double heading_raw = get_heading().convert(radian);
  
  QAngle angle_to_target = reduce_negative_180_to_180(desired_heading - get_heading());
  double heading_scale_factor = std::cos(angle_to_target.convert(radian));

  double center_output = drive_output * heading_scale_factor;
  double center_max_speed = std::abs(heading_scale_factor) * p.center_max_speed;

  double left_center_voltage = std::clamp(center_output, -center_max_speed, center_max_speed);
  double right_center_voltage = std::clamp(center_output, -center_max_speed, center_max_speed);
    
  double left_front_output = (drive_output * std::cos(heading_raw) + raw_heading_error - M_PI / 4) + turn_output;
  double left_back_output = (drive_output * std::cos(-heading_raw) - raw_heading_error + 3 * M_PI / 4) + turn_output;
  double right_back_output = (drive_output * std::cos(heading_raw) + raw_heading_error - M_PI / 4) - turn_output;
  double right_front_output = (drive_output * std::cos(-heading_raw) - raw_heading_error + 3 * M_PI / 4) - turn_output;

  rev::SlipstreamPower power = {
    .front_left_forward = left_front_output,
    .front_right_forward = right_front_output,
    .rear_left_forward = left_back_output,
    .rear_right_forward = right_back_output,

    .front_left_steer = left_center_voltage,
    .front_right_steer = right_center_voltage
  };
  
  return last_status = SlipstreamSegmentStatus::drive(power);
}

void MecanumToPose::clean_up() {}

double MecanumToPose::progress() {
  return part_progress;
}

}  // namespace rev
