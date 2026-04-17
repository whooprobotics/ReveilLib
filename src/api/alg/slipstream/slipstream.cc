#include <iostream>
#include "pros/rtos.hpp"
#include "rev/api/alg/stop/stop.hh"
#include "rev/api/alg/slipstream/slipstream.hh"
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/hardware/chassis/holonomic_chassis.hh"

namespace rev {

Slipstream::Slipstream(std::shared_ptr<HolonomicChassis> ichassis,
                       std::shared_ptr<Odometry> iodometry)
   : chassis(ichassis), odometry(iodometry) {}

void Slipstream::step() {
  if (is_completed())
    return;

  if (current_segment >= current_path.segments.size()) {
    std::cout << "Completed motion with " << current_path.segments.size() << " segments" << std::endl;
    status = SlipstreamStatus::DONE;
    partial_progress = current_path.segments.size();
    current_segment = 0;
    chassis->stop();
    return;
  }

  OdometryState current_state = odometry->get_state();
  auto seg = current_path.segments.at(current_segment);
  SlipstreamSegmentStatus drive_state = seg->step(current_state);

  partial_progress = (double) current_segment + seg->progress();

  switch(drive_state.status) {
    case rev::SlipstreamSegmentStatusType::DRIVE:
      chassis->drive_holonomic(drive_state.power);
      brake_start_time = -1;
      break;
    case rev::SlipstreamSegmentStatusType::BRAKE:
      if (brake_start_time == -1) {
        chassis->set_brake_harsh();
        chassis->stop();
        brake_start_time = pros::millis();
        break;
      }
      else if (pros::millis() <= brake_start_time + 250)
        break;
    case rev::SlipstreamSegmentStatusType::NEXT:
      chassis->set_brake_coast();
      chassis->stop();
      seg->clean_up();
      current_segment++;

      if (current_segment < current_path.segments.size())
        current_path.segments.at(current_segment)->init(current_state);
      brake_start_time = -1;
      break;
    case rev::SlipstreamSegmentStatusType::DUMMY:
      break;
  }
}

void Slipstream::await() {
  while (!is_completed())
    pros::delay(10);
}

bool Slipstream::is_ready() {
  return this->is_completed();
}

void Slipstream::go(SlipstreamPath path) {
  if (!is_completed())
    breakout();
  current_segment = 0;
  current_path = path;
  current_path.segments.at(0)->init(odometry->get_state());
  status = SlipstreamStatus::ACTIVE;
  std::cout << "Started Slipstream motion with " << current_path.segments.size()
            << " segments" << std::endl;
}

SlipstreamStatus Slipstream::get_status() {
  return status;
}

double Slipstream::progress() {
  return partial_progress;
}

bool Slipstream::is_completed() {
  return get_status() == SlipstreamStatus::DONE;
}

void Slipstream::breakout() {
  status = SlipstreamStatus::DONE;
  chassis->drive_tank(0, 0);
}
} // namespace rev