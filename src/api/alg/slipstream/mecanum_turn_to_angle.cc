#include "rev/api/alg/slipstream/mecanum_turn_to_angle.hh"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "rev/api/alg/PID/PID.hh"
#include "rev/api/alg/slipstream/segment.hh"

namespace rev {

void MecanumTurnToAngle::init(OdometryState initial_state) {
  std::cout << "MecanumTurnToAngle segment invoked" << std::endl;
  this->last_status = SlipstreamSegmentStatus::drive({0, 0, 0, 0});

  QAngle raw_error =
      reduce_negative_180_to_180(target_angle - initial_state.pos.theta);
  prev_error = raw_error;
  prev_raw_error = raw_error;
  crossed = false;

  turnPID =
      PID(p.turn_k.p, p.turn_k.i, p.turn_k.d, p.turn_k.starti,
          p.turn_settle.settle_error, p.turn_settle.settle_time,
          p.turn_settle.large_settle_error, p.turn_settle.large_settle_time,
          p.exit_error.convert(degree), p.timeout);
}

SlipstreamSegmentStatus MecanumTurnToAngle::step(OdometryState current_state) {
  QAngle current_heading = current_state.pos.theta;
  QAngle raw_error = reduce_negative_180_to_180(target_angle - current_heading);

  if (raw_error.convert(degree) * prev_raw_error.convert(degree) < 0) {
    crossed = true;
  }
  prev_raw_error = raw_error;

  QAngle error = raw_error;

  if (p.min_speed != 0 && crossed &&
      error.convert(degree) * prev_error.convert(degree) < 0) {
    return last_status = SlipstreamSegmentStatus::next();
  }

  prev_error = error;

  double output = turnPID.compute(error.convert(degree));

  if (turnPID.is_settled()) {
    return last_status = SlipstreamSegmentStatus::next();
  }

  output = std::clamp(output, -p.max_speed, p.max_speed);
  output = clamp_min_voltage(output, p.min_speed);

  rev::SlipstreamPower power = {.front_left_forward = output / 12,
                                .front_right_forward = -output / 12,
                                .rear_left_forward = output / 12,
                                .rear_right_forward = -output / 12,

                                .front_left_steer = -output / 12,
                                .front_right_steer = output / 12};

  return last_status = SlipstreamSegmentStatus::drive(power);
}

void MecanumTurnToAngle::clean_up() {}

double MecanumTurnToAngle::progress() {
  return part_progress;
}

}  // namespace rev
