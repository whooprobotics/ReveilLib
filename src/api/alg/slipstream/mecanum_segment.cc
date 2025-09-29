#include "rev/api/alg/slipstream/mecanum_segment.hh"
#include <iostream>
#include "rev/api/alg/slipstream/segment.hh"
namespace rev {

void MecanumSegment::init(OdometryState initial_state) {
  std::cout << "Pilons segment invoked" << std::endl;
  this->start_point = initial_state.pos;
}

SlipstreamSegmentStatus MecanumSegment::step(OdometryState current_state) {
  // TODO: Actually implement logic
  SlipstreamPower power = {0, 0, 0, 0};

  return SlipstreamSegmentStatus::drive(power);
}

void MecanumSegment::clean_up() {}

double MecanumSegment::progress() {
  return part_progress;
}

}  // namespace rev
