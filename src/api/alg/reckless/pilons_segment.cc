#include "rev/api/alg/reckless/pilons_segment.hh"
#include <iostream>
#include "rev/api/alg/reckless/path.hh"
namespace rev {

void PilonsSegment::init(OdometryState initial_state) {
  std::cout << "Pilons segment invoked" << std::endl;
  this->start_point = initial_state.pos;
}

SegmentStatus PilonsSegment::step(OdometryState current_state) {
  stop_state new_state = this->stop->get_stop_state(current_state, target_point,
                                                    start_point, drop_early);

  QLength d_to_start = sqrt((start_point.x - current_state.pos.x) *
                                (start_point.x - current_state.pos.x) +
                            (start_point.y - current_state.pos.y) *
                                (start_point.y - current_state.pos.y));

  QLength current_d = sqrt((current_state.pos.x - target_point.x) *
                               (current_state.pos.x - target_point.x) +
                           (current_state.pos.y - target_point.y) *
                               (current_state.pos.y - target_point.y));

  Number pct_progress = d_to_start / (d_to_start + current_d);
  part_progress = pct_progress.convert(Number(1.));

  // Prevent status from regressing
  if (last_status.status == SegmentStatusType::NEXT ||
      new_state == stop_state::EXIT)
    return last_status = SegmentStatus::next();

  if (last_status.status == SegmentStatusType::BRAKE ||
      new_state == stop_state::BRAKE)
    return last_status = SegmentStatus::brake();

  std::tuple<double, double> pows = this->motion->gen_powers(
      current_state, this->target_point, this->start_point, this->drop_early);

  // Handle coasting if needed
  if (new_state == stop_state::COAST) {
    double power = this->stop->get_coast_power();
    double left, right;
    std::tie(left, right) = pows;
    if (left + right < 0)
      power *= -1;
    return last_status = SegmentStatus::drive(power);
  }
  // Apply correction
  std::tuple<double, double> corrected_pows =
      this->correction->apply_correction(current_state, this->target_point,
                                         this->start_point, this->drop_early,
                                         pows);
  return SegmentStatus::drive(corrected_pows);
}

void PilonsSegment::clean_up() {}

double PilonsSegment::progress() {
  return part_progress;
}

}  // namespace rev