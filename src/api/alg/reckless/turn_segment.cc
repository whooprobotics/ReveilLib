#include "rev/api/alg/reckless/turn_segment.hh"
#include "rev/api/alg/drive/turn/turn.hh"

namespace rev {

RecklessTurnSegment::RecklessTurnSegment(double imax_power, 
                        double icoast_power, QAngle iangle, 
                        double iharsh_coeff, double icoast_coeff)
        : max_power(imax_power),
          coast_power(icoast_power),
          harsh_coeff(iharsh_coeff),
          coast_coeff(icoast_coeff),
          angle(iangle) {
          }

void RecklessTurnSegment::init(OdometryState initial_state) {
  
}

SegmentStatus RecklessTurnSegment::step(OdometryState current_state) {
  // if (controller_state == TurnState::INACTIVE) {
  //   return;
  // }

  OdometryState state = current_state;

  // Full power turn
  if (controller_state == TurnState::FULLPOWER) {
    return SegmentStatus::drive(max_power * left_direction,
                        max_power * right_direction);
  }
  // Low power turn
  else if (controller_state == TurnState::COAST) {
    return SegmentStatus::drive(left_direction * coast_power,
                                right_direction * coast_power);
  }
  // Activating hard brakes
  else if (controller_state == TurnState::BRAKE) {
    return SegmentStatus::brake();
  }


}

void RecklessTurnSegment::clean_up() {}

} // namespace rev