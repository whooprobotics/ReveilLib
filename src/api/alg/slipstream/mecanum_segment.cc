#include "rev/api/alg/slipstream/mecanum_segment.hh"
#include <iostream>
#include <cmath>
#include <algorithm>
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/alg/stop/stop.hh"
namespace rev {

void MecanumSegment::init(OdometryState initial_state) {
  std::cout << "Pilons segment invoked" << std::endl;
  this->start_point = initial_state.pos;
}

SlipstreamSegmentStatus MecanumSegment::step(OdometryState current_state) {

  // Generate base power from motion controller
  SlipstreamPower base_power = motion->gen_powers(current_state, target_point, start_point, drop_early);

  // Apply correction
  SlipstreamPower corrected_power = correction->apply_correction(current_state, target_point, start_point, drop_early, base_power);

  // Check if we should stop
  StopState stop_state = stop->get_stop_state(current_state, target_point, start_point, drop_early);
  if (stop_state == StopState::EXIT) {
    return SlipstreamSegmentStatus::next();
  } else if (stop_state == StopState::BRAKE) {
    return SlipstreamSegmentStatus::brake();
  } else if (stop_state == StopState::COAST) {
    // Apply coast power to corrected power
    double coast_power = stop->get_coast_power();
    corrected_power.front_left_forward *= coast_power;
    corrected_power.front_right_forward *= coast_power;
    corrected_power.rear_left_forward *= coast_power;
    corrected_power.rear_right_forward *= coast_power;
  }

  // Calculate progress as distance traveled / total distance
  QLength total_distance = sqrt(square(target_point.x - start_point.x) + square(target_point.y - start_point.y));
  QLength distance_traveled = sqrt(square(current_state.pos.x - start_point.x) + square(current_state.pos.y - start_point.y));

  if (total_distance > 0.01_in) {
    part_progress = std::min(1.0, (distance_traveled / total_distance).get_value());
  }

  return SlipstreamSegmentStatus::drive(corrected_power);
}

void MecanumSegment::clean_up() {}

double MecanumSegment::progress() {
  return part_progress;
}

}  // namespace rev
