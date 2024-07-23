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

  if (new_state == stop_state::COAST)
    return last_status = SegmentStatus::drive(this->stop->get_coast_power(),
                                              this->stop->get_coast_power());

  std::tuple<double, double> pows = this->motion->gen_powers(
      current_state, this->target_point, this->start_point, this->drop_early);
  std::tuple<double, double> corrected_pows =
      this->correction->apply_correction(current_state, this->target_point,
                                         this->start_point, this->drop_early,
                                         pows);
  return SegmentStatus::drive(corrected_pows);
}

void RecklessPathSegment::clean_up() {}

}  // namespace rev