#ifdef PLATFORM_BRAIN

#include <iostream>
#include <cmath>
#include <algorithm>
#include "rev/api/v5/alg/slipstream/mecanum_segment.hh"
#include "rev/api/v5/alg/slipstream/segment.hh"
#include "rev/api/v5/alg/drive/stop/stop.hh"
#include "rev/api/common/units/q_length.hh"

namespace rev {

void MecanumSegment::init(OdometryState initial_state) {
  std::cout << "Mecanum segment invoked" << std::endl;
  this->start_point = initial_state.pos;
  this->last_status = SlipstreamSegmentStatus::drive({0, 0, 0, 0});
}

SlipstreamSegmentStatus MecanumSegment::step(OdometryState current_state) {

  // Check stop state
  StopState stop_state = stop->get_stop_state(current_state, target_point, start_point, drop_early);

  // Calculate progress as distance traveled / total distance
  QLength total_distance = sqrt(square(target_point.x - start_point.x) + square(target_point.y - start_point.y));
  QLength distance_traveled = sqrt(square(current_state.pos.x - start_point.x) + square(current_state.pos.y - start_point.y));

  if (total_distance > 0.01_in) {
    part_progress = std::min(1.0, (distance_traveled / total_distance).get_value());
  }

  // Prevent status from regressing
  if (last_status.status == SlipstreamSegmentStatusType::NEXT ||
      stop_state == StopState::EXIT)
    return last_status = SlipstreamSegmentStatus::next();

  if (last_status.status == SlipstreamSegmentStatusType::BRAKE ||
      stop_state == StopState::BRAKE)
    return last_status = SlipstreamSegmentStatus::brake();

  // Generate base power from motion controller
  SlipstreamPower base_power = motion->gen_powers(current_state, target_point, start_point, drop_early);

  // Handle coasting if needed
  if (stop_state == StopState::COAST) {
    double coast_power_val = stop->get_coast_power();
    SlipstreamPower coast_power = {
      base_power.front_left_forward * coast_power_val,
      base_power.front_right_forward * coast_power_val,
      base_power.rear_left_forward * coast_power_val,
      base_power.rear_right_forward * coast_power_val
    };
    return last_status = SlipstreamSegmentStatus::drive(coast_power);
  }

  // Apply correction
  SlipstreamPower corrected_power = correction->apply_correction(current_state, target_point, start_point, drop_early, base_power);

  return last_status = SlipstreamSegmentStatus::drive(corrected_power);
}

void MecanumSegment::clean_up() {}

double MecanumSegment::progress() {
  return part_progress;
}

}  // namespace rev

#endif