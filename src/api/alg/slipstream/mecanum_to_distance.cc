#include "rev/api/alg/slipstream/mecanum_to_distance.hh"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "rev/api/alg/PID/PID.hh"
#include "rev/api/alg/slipstream/segment.hh"

namespace rev {

void MecanumToDistance::init(OdometryState initial_state) {
  std::cout << "MecanumToDistance segment invoked" << std::endl;
  this->last_status = SlipstreamSegmentStatus::drive({0, 0, 0, 0});

  start_pos = initial_state.pos;

  drivePID = PID(p.drive_k.p, p.drive_k.i, p.drive_k.d, p.drive_k.starti,
                 p.drive_settle.settle_error, p.drive_settle.settle_time,
                 p.drive_settle.large_settle_error,
                 p.drive_settle.large_settle_time, p.exit_error.convert(inch), p.timeout);

  headingPID = PID(p.turn_k.p, p.turn_k.i, p.turn_k.d, p.turn_k.starti);

  heading = initial_state.pos.theta;
}

SlipstreamSegmentStatus MecanumToDistance::step(OdometryState current_state) {
  QLength dx = current_state.pos.y - start_pos.y;
  QLength dy = current_state.pos.x - start_pos.x;

  double heading_rad = heading.convert(radian);
  QLength traveled = dx * std::cos(heading_rad) + dy * std::sin(heading_rad);

  QLength drive_error = distance - traveled;
  QAngle heading_error =
      reduce_negative_180_to_180(heading - current_state.pos.theta);

  double drive_output = drivePID.compute(drive_error.convert(inch));
  double heading_output = headingPID.compute(heading_error.convert(degree));

  if (drivePID.is_settled()) {
    return last_status = SlipstreamSegmentStatus::next();
  }

  drive_output = std::clamp(drive_output, -p.max_speed, p.max_speed);
  heading_output =
      std::clamp(heading_output, -p.turn_max_speed, p.turn_max_speed);
  drive_output = clamp_min_voltage(drive_output, p.min_speed);

  rev::SlipstreamPower power = {
      .front_left_forward = (drive_output + heading_output) / 12,
      .front_right_forward = (drive_output - heading_output) / 12,
      .rear_left_forward = (drive_output + heading_output) / 12,
      .rear_right_forward = (drive_output - heading_output) / 12,

      .front_left_steer = heading_output / 12,
      .front_right_steer = -heading_output / 12,
  };

  return last_status = SlipstreamSegmentStatus::drive(power);
}

void MecanumToDistance::clean_up() {}

double MecanumToDistance::progress() {
  return part_progress;
}

}  // namespace rev
