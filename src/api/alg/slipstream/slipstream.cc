#include "rev/api/alg/slipstream/slipstream.hh"
#include <iostream>
#include "pros/rtos.hpp"

namespace rev {

StopState lsstate = StopState::GO;
Slipstream::Slipstream(std::shared_ptr<Chassis> ichassis,
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
  
  }
}

} // namespace rev