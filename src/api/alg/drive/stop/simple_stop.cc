#include "rev/api/alg/drive/stop/simple_stop.hh"

namespace rev {

SimpleStop::SimpleStop(QTime iharsh_threshold,
                       QTime icoast_threshold,
                       double icoast_power)
    : harsh_threshold(iharsh_threshold),
      coast_threshold(icoast_threshold),
      coast_power(fabs(icoast_power)) {}
stop_state SimpleStop::get_stop_state(OdometryState current_state,
                                      Position target_state,
                                      Position start_state,
                                      QLength drop_early) {
  QLength x_distance = target_state.x - current_state.pos.x;
  QLength y_distance = target_state.y - current_state.pos.y;

  // numbers are weird and x is sin here
  // If this starts bugging, this is the first thing we should check.
  Number x_dot = cos(current_state.pos.facing);
  Number y_dot = sin(current_state.pos.facing);

  QLength longitudinal_distance =
      x_dot * x_distance + y_dot * y_distance - drop_early;
  QSpeed longitudinal_speed =
      x_dot * current_state.vel.xv + y_dot * current_state.vel.yv;

  // Now for the other things
  if (abs(longitudinal_speed * harsh_threshold) >= abs(longitudinal_distance) || stop_state_last == stop_state::BRAKE){
    stop_state_last = stop_state::BRAKE;
    return stop_state::BRAKE;
  }

  // If we've passed the target, its stop time
  if (longitudinal_distance.get_value() < 0) {
    stop_state_last = stop_state::BRAKE;
    return harsh_threshold.convert(second) > 0.01 ? stop_state::BRAKE
                                                  : stop_state::EXIT;
  }
  if (abs(longitudinal_speed * coast_threshold) >= abs(longitudinal_distance) || stop_state_last == stop_state::COAST){
    stop_state_last = stop_state::COAST;
    return stop_state::COAST;
  }

  return stop_state::GO;
}

double SimpleStop::get_coast_power() {
  return coast_power;
}

}  // namespace rev