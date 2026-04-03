#ifdef PLATFORM_BRAIN

#include <iostream>
#include <cmath>
#include <cassert>
#include "rev/api/v5/alg/reckless/pilons_segment.hh"
#include "rev/api/v5/alg/reckless/path.hh"

#define PI 3.1415926535

using std::cout, std::endl;
using std::tuple;

namespace rev {

DefaultPilonsParams default_params = {0.0, 0.0, 0_in, 0_s, 0_s, 0.0};

void PilonsSegment::init(OdometryState initial_state) {
  cout << "Pilons segment invoked" << endl;
  this->start_point = initial_state.pos;
}

SegmentStatus PilonsSegment::step(OdometryState current_state) {
  StopState new_state = this->stop->get_stop_state(current_state, target_point,
                                                    start_point, drop_early);

  QLength d_to_start = sqrt((start_point.x - current_state.pos.x) *
                                (start_point.x - current_state.pos.x) +
                            (start_point.y - current_state.pos.y) *
                                (start_point.y - current_state.pos.y));

  QLength current_d = sqrt((current_state.pos.x - target_point.x) *
                               (current_state.pos.x - target_point.x) +
                           (current_state.pos.y - target_point.y) *
                               (current_state.pos.y - target_point.y));

  this->part_progress = d_to_start.convert(inch) / (d_to_start + current_d).convert(inch);

  // Prevent status from regressing
  if (last_status.status == SegmentStatusType::NEXT ||
      new_state == StopState::EXIT)
    return last_status = SegmentStatus::next();

  if (last_status.status == SegmentStatusType::BRAKE ||
      new_state == StopState::BRAKE)
    return last_status = SegmentStatus::brake();

  tuple<double, double> pows;

  pows = this->motion->gen_powers(
    current_state, this->target_point, this->start_point, this->drop_early);

  // Handle coasting if needed
  if (new_state == StopState::COAST) {
    double power = this->stop->get_coast_power();
    double left, right;
    std::tie(left, right) = pows;
    if (left + right < 0)
      power *= -1;
    return last_status = SegmentStatus::drive(power);
  }
  // Apply correction
  // again, temporary measure for 3.1.0
  // REFACTOR THIS IN 4.0.0
  tuple<double, double> corrected_pows = this->correction->apply_correction(current_state, this->target_point,
                                       this->start_point, this->drop_early,
                                       pows);
  return SegmentStatus::drive(corrected_pows);
}

void PilonsSegment::clean_up() {}

double PilonsSegment::progress() {
  return part_progress;
}

tuple<double, double> PilonsSegment::gen_powers(
    OdometryState current_state) {
  Number xi_facing = cos(start_point.theta);
  Number yi_facing = sin(target_point.theta);

  // Find dot product of initial facing and initial offset. If this dot product
  // is negative, the target point is behind the robot and it needs to reverse
  // to get there.
  QLength initial_longitudinal_distance =
      xi_facing * (target_point.x - start_point.x) +
      yi_facing * (target_point.y - start_point.y);

  bool isBackwards = (initial_longitudinal_distance.get_value() < 0);

  double opower = isBackwards ? -(default_params.power) : default_params.power;

  return std::make_tuple(opower, opower);
}

QAngle near_semicircle(rev::QAngle angle, rev::QAngle reference) {
  return rev::radian *
         (std::round((reference.get_value() - angle.get_value()) / PI) * PI +
          angle.get_value());
}

tuple<double, double> PilonsSegment::pilons_correction(
    rev::OdometryState current_state,
    std::tuple<double, double> powers) {
  Pose pos_current = current_state.pos;

  // Find the pose which is at target_state
  Pose pos_final = target_point;
  // but make this reference frame face directly away from the start state
  pos_final.theta =
      atan2(pos_final.y - start_point.y, pos_final.x - start_point.x);

  // If the robot starts facing more than 90 degrees from that, flip it to face
  // towards the robot
  pos_final.theta = near_semicircle(pos_final.theta, start_point.theta);

  // Reframe the robots current position in reference to the target state
  Pose error = pos_current.to_relative(pos_final);

  // The angle from the perspective of the target point
  // We subtract from the facing angle so we have the actual error angle
  QAngle error_angle = -error.theta + atan2(error.y, error.x);

  error_angle = near_semicircle(error_angle, 0_deg);

  double correction = abs(error.y + error.x * tan(error.theta)) > abs(max_error)
                          ? k_correction * error_angle.get_value()
                          : 0.0;

  if (std::get<0>(powers) < 0)
    correction = -correction;

  if (correction > 0)
    return std::make_tuple(std::get<0>(powers),
                           std::get<1>(powers) * exp(-correction));
  else if (correction < 0)
    return std::make_tuple(std::get<0>(powers) * exp(correction),
                           std::get<1>(powers));
  else
    return powers;
}

}  // namespace rev

#endif