#include "rev/api/alg/reckless/boomerang.hh"
#include <iostream>
#include "rev/api/alg/drive/stop/stop.hh"
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/reckless/path.hh"
namespace rev {

void BoomerangSegment::init(OdometryState initial_state) {
  std::cout << "Boomerang segment invoked" << std::endl;
  this->start_point = initial_state.pos;

  Number xi_facing = cos(initial_state.pos.theta);
  Number yi_facing = sin(initial_state.pos.theta);

  // Find dot product of initial facing and initial offset. If this dot product
  // is negative, the target point is behind the robot and it needs to reverse
  // to get there.
  QLength initial_longitudinal_distance =
      xi_facing * (target_point.x - initial_state.pos.x) +
      yi_facing * (target_point.y - initial_state.pos.y);

  if(initial_longitudinal_distance < 0_m)
    direction = -1;
}

SegmentStatus BoomerangSegment::step(OdometryState current_state) {

  QLength d_to_start = sqrt((start_point.x - current_state.pos.x) *
                                (start_point.x - current_state.pos.x) +
                            (start_point.y - current_state.pos.y) *
                                (start_point.y - current_state.pos.y));

  QLength current_d = sqrt((current_state.pos.x - target_point.x) *
                               (current_state.pos.x - target_point.x) +
                           (current_state.pos.y - target_point.y) *
                               (current_state.pos.y - target_point.y));

  stop_state new_state;
  if(close)
    new_state = this->stop->get_stop_state(current_state, target_point,
                                                    frozen_carrot_point, drop_early);
  else {
    new_state = stop_state::GO;

    if(abs(current_d) < 7.5_in) {
      close = true;
      frozen_carrot_point = current_state.pos;
    }
  }

  // Calculate carrot point
  QLength carrot_x = target_point.x - direction * lead * current_d * cos(target_point.theta);
  QLength carrot_y = target_point.y - direction * lead * current_d * sin(target_point.theta);

  Position carrot_point (carrot_x, carrot_y, 0_deg);


  this->part_progress = d_to_start.convert(inch) / (d_to_start + current_d).convert(inch);

  // Prevent status from regressing
  if (last_status.status == SegmentStatusType::NEXT ||
      new_state == stop_state::EXIT)
    return last_status = SegmentStatus::next();

  if (last_status.status == SegmentStatusType::BRAKE ||
      new_state == stop_state::BRAKE)
    return last_status = SegmentStatus::brake();

  std::tuple<double, double> pows = this->motion->gen_powers(
      current_state, carrot_point, this->start_point, this->drop_early);

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
      this->correction->apply_correction(current_state, carrot_point,
                                         this->start_point, this->drop_early,
                                         pows);
  return SegmentStatus::drive(corrected_pows);
}

void BoomerangSegment::clean_up() {}

double BoomerangSegment::progress() {
  return part_progress;
}

}  // namespace rev