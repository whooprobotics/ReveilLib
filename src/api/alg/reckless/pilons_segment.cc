#include "rev/api/alg/reckless/path.hh"

namespace rev {

void RecklessPathSegment::init(OdometryState initial_state) {
  this->start_point = initial_state.pos;
}

SegmentStatus RecklessPathSegment::step(OdometryState current_state) {
  stop_state new_state = this->stop->get_stop_state(current_state, target_point,
                                                    start_point, drop_early);

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

void RecklessPathSegment::clean_up() {}

}  // namespace rev