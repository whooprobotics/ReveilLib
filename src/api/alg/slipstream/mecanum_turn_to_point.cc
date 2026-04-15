#include "rev/api/alg/slipstream/mecanum_turn_to_point.hh"
#include <cmath>
#include <iostream>
#include "rev/api/alg/slipstream/mecanum_turn_to_angle.hh"

namespace rev {

void MecanumTurnToPoint::init(OdometryState initial_state) {
  std::cout << "MecanumTurnToPoint segment invoked" << std::endl;

  QLength x = target_point.x;
  QLength y = target_point.y;
  QLength current_x = initial_state.pos.y;
  QLength current_y = initial_state.pos.x;

  QAngle target_angle = atan2(x - current_x, y - current_y) + p.offset;

  inner = MecanumTurnToAngle(target_angle, p);
  inner.init(initial_state);
}

SlipstreamSegmentStatus MecanumTurnToPoint::step(OdometryState current_state) {
  return inner.step(current_state);
}

void MecanumTurnToPoint::clean_up() {
  inner.clean_up();
}

double MecanumTurnToPoint::progress() {
  return inner.progress();
}

}  // namespace rev
