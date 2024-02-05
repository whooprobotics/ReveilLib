#include "rev/api/alg/reckless/reckless.hh"
#include "iostream"
#include "pros/rtos.hpp"

namespace rev {

stop_state lsstate = stop_state::GO;
Reckless::Reckless(std::shared_ptr<Chassis> ichassis,
                   std::shared_ptr<Odometry> iodometry)
    : chassis(ichassis), odometry(iodometry) {}

void Reckless::step() {
  if (is_completed())  // Don't step the controller if it is not running for
                       // obvious reasons
    return;

  // If we are out of steps to complete, don't try to complete a step
  if (current_segment >= current_path.segments.size()) {
    std::cout << "Completed motion with " << current_path.segments.size()
              << " segments" << std::endl;
    status = RecklessStatus::DONE;
    partial_progress = -1.0;
    current_segment = 0;
    chassis->stop();
    return;
  }

  OdometryState current_state = odometry->get_state();

  auto seg = current_path.segments.at(current_segment);

  // TODO: define step function

  auto current_stop_state = seg.stop->get_stop_state(
      current_state, seg.target_point, seg.start_point, seg.drop_early);

  if (current_stop_state != lsstate) {
    std::cout << "State change occured to "
              << (current_stop_state == stop_state::GO      ? "GO"
                  : current_stop_state == stop_state::COAST ? "COAST"
                                                            : "BRAKE")
              << std::endl;
    lsstate = current_stop_state;
  }

  switch (current_stop_state) {
    case stop_state::GO: {
      auto pows = seg.motion->gen_powers(current_state, seg.target_point,
                                         seg.start_point, seg.drop_early);
      auto [power_left, power_right] = seg.correction->apply_correction(
          current_state, seg.target_point, seg.start_point, seg.drop_early,
          pows);
      chassis->drive_tank(power_left, power_right);
      // Safety, should never matter
      brake_time = -1;
      break;
    } break;
    case stop_state::COAST: {
      // If the final state is somewhere behind the start state, we need to
      // invert the facing vector
      Number xi_facing = cos(seg.start_point.theta);
      Number yi_facing = sin(seg.start_point.theta);

      // Find dot product of initial facing and initial offset. If this dot
      // product is negative, the target point is behind the robot and it needs
      // to reverse to get there.
      QLength initial_longitudinal_distance =
          xi_facing * (seg.target_point.x - seg.start_point.x) +
          yi_facing * (seg.target_point.y - seg.start_point.y);

      double coast_power = seg.stop->get_coast_power();
      // If its negative, we're goin backwards
      if (initial_longitudinal_distance.get_value() < 0)
        coast_power = -coast_power;
      chassis->drive_tank(coast_power, coast_power);
      // Safety, should never matter, just like Josh DeBerry
      brake_time = -1;
      break;
    } break;
    case stop_state::BRAKE: {
      // Check if we havent started braking yet
      if (brake_time == -1) {
        chassis->set_brake_harsh();
        chassis->stop();
        brake_time = pros::millis();
      }
      // Check if enough time has elapsed to stop braking
      else if (pros::millis() > brake_time + 250) {
        chassis->set_brake_coast();
        brake_time = -1;
        current_segment++;
        // Ensure the next segment knows where we're really starting
        if (current_segment < current_path.segments.size())
          current_path.segments.at(current_segment).start_point =
              current_state.pos;
      }
      break;
    } break;
    // Just like brake but without the braking
    case stop_state::EXIT:
      chassis->set_brake_coast();
      chassis->stop();
      current_segment++;
      // Ensure the next segment knows where we're really starting
      if (current_segment < current_path.segments.size())
        current_path.segments.at(current_segment).start_point =
            current_state.pos;
      // Safety, should never matter
      brake_time = -1;
      break;
  }  // switch (current_stop_state) {

  // Update progress
  QLength csx = current_state.pos.x - seg.start_point.x;
  QLength csy = current_state.pos.y - seg.start_point.y;

  QLength tsx = seg.target_point.x - seg.start_point.x;
  QLength tsy = seg.target_point.y - seg.start_point.y;

  auto csts = csx * tsx + csy * tsy;
  auto tsts = tsx * tsx + tsy * tsy;
  Number current_segment_progress = csts / tsts;
  partial_progress =
      (double)current_segment + current_segment_progress.convert(number);
}

/**
 * This function starts the robot along a path
 */
void Reckless::go(RecklessPath path) {
  if (!is_completed())
    breakout();
  current_segment = 0;
  current_path = path;
  current_path.segments.at(0).start_point = odometry->get_state().pos;
  status = RecklessStatus::ACTIVE;
  std::cout << "Started motion with " << current_path.segments.size()
            << " segments" << std::endl;
}

/**
 * This function returns the current status of the controller
 */
RecklessStatus Reckless::get_status() {
  return status;
}

/**
 * This function gets the current progress along the total path. [0,1] for the
 * first segment, [1,2] second segment, etc. Returns -1.0 if the controller is
 * not running. Returns the integer upper bound of a motion if that motion has
 * invoked a harsh stop
 */
double Reckless::progress() {
  return partial_progress;
}
/**
 * This function returns true if the status is DONE, and false otherwise
 */
bool Reckless::is_completed() {
  return get_status() == RecklessStatus::DONE;
}
/**
 * This function immediately sets the status to DONE and ends the current motion
 */
void Reckless::breakout() {
  status = RecklessStatus::DONE;
}

/**
 * @brief Add a segment to the path under construction
 *
 * @param segment The segment to add
 * @return RecklessPath& An ongoing path builder
 */
RecklessPath& RecklessPath::with_segment(RecklessPathSegment segment) {
  segments.push_back(segment);
  return *this;
}
}  // namespace rev