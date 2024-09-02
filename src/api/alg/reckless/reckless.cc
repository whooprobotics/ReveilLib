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

  SegmentStatus drive_state = seg->step(current_state);

  switch (drive_state.status) {
    case SegmentStatusType::DRIVE: {
      chassis->drive_tank(drive_state.power_left, drive_state.power_right);
      // Safety, should never matter
      brake_time = -1;
      break;
    } break;
    case SegmentStatusType::BRAKE:
      // Check if we havent started braking yet
      if (brake_time == -1) {
        chassis->set_brake_harsh();
        chassis->stop();
        brake_time = pros::millis();
      }
      // Check if enough time has elapsed to stop braking
      // If enough time has passed then it will just proceed as if it recieved a
      // NEXT
      else if (pros::millis() <= brake_time + 250)
        break;
    // Just like brake but without the braking
    case SegmentStatusType::NEXT:
      chassis->set_brake_coast();
      chassis->stop();
      seg->clean_up();
      current_segment++;
      // Ensure the next segment knows where we're really starting
      if (current_segment < current_path.segments.size())
        current_path.segments.at(current_segment)->init(current_state);
      // Safety, should never matter
      brake_time = -1;
      break;
    case SegmentStatusType::DUMMY:
      break;
  }  // switch (current_stop_state) {

  // TODO: figure out a way to update progress with the new segment format
}

void Reckless::await() {
  // Technically we can make this lighter with a condition variable or
  // semaphore but its not worth it in this case because its a very quick
  // check
  while (!is_completed())
    pros::delay(10);
}

bool Reckless::is_ready() {
  return this->is_completed();
}

/**
 * This function starts the robot along a path
 */
void Reckless::go(RecklessPath path) {
  if (!is_completed())
    breakout();
  current_segment = 0;
  current_path = path;
  current_path.segments.at(0)->init(odometry->get_state());
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
 * This function immediately sets the status to DONE and ends the current
 * motion
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
  segments.push_back(std::make_shared<RecklessPathSegment>(segment));
  return *this;
}

}  // namespace rev