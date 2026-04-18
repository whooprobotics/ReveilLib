#ifdef PLATFORM_BRAIN

#include "rev/api/v5/alg/drive/stop/simple_holonomic_stop.hh"

namespace rev {

rev::QAngle near_semicircle(rev::QAngle angle, rev::QAngle reference);

SimpleHolonomicStop::SimpleHolonomicStop(QTime iharsh_threshold,
                                         QTime icoast_threshold,
                                         double icoast_power)
    : harsh_threshold(iharsh_threshold),
      coast_threshold(icoast_threshold),
      coast_power(fabs(icoast_power)) {}

SimpleHolonomicStop::SimpleHolonomicStop(QTime iharsh_threshold,
                                         QTime icoast_threshold,
                                         double icoast_power,
                                         QTime itimeout)
    : harsh_threshold(iharsh_threshold),
      coast_threshold(icoast_threshold),
      coast_power(fabs(icoast_power)) {
  timeout = itimeout; // TODO: Handle timeout, currently not being used
}

StopState SimpleHolonomicStop::get_stop_state(OdometryState current_state,
                                              Position target_state,
                                              Position start_state,
                                              QLength drop_early) {

  QSpeed total_speed = calculate_total_speed(current_state.vel);

  Pose pos_current = current_state.pos;
  Pose pos_target = target_state;

  QLength total_distance = calculate_total_distance(pos_current, pos_target, drop_early);

  // total distance can be negative if we pass drop_early, so brake if past
  if (total_distance.get_value() < 0) {
    stop_state_last = StopState::BRAKE;
    return harsh_threshold.convert(second) > 0.001 ? StopState::BRAKE
                                                   : StopState::EXIT;
  }

  // if robot will reach target within harsh threshold time, brake
  if (total_speed * harsh_threshold > total_distance ||
      stop_state_last == StopState::BRAKE) {
    stop_state_last = StopState::BRAKE;
    return StopState::BRAKE;
  }

  // if robot will reach target within coast threshold time, coast
  if (total_speed * coast_threshold > total_distance ||
      stop_state_last == StopState::COAST) {
    stop_state_last = StopState::COAST;
    return StopState::COAST;
  }

  return StopState::GO;
}

double SimpleHolonomicStop::get_coast_power() {
  return coast_power;
}

QSpeed SimpleHolonomicStop::calculate_total_speed(const Velocity& velocity) {
  // Calculate the magnitude of translational velocity (Euclidean norm)
  // Ignore angular, should not be too relevant
  QSpeed translational_speed = sqrt(square(velocity.xv) +
                                   square(velocity.yv));

  return translational_speed;
}

QLength SimpleHolonomicStop::calculate_total_distance(const Pose& current_pos,
                                                     const Pose& target_pos,
                                                     QLength drop_early) {
  QLength dx = target_pos.x - current_pos.x;
  QLength dy = target_pos.y - current_pos.y;
  QLength translational_distance = sqrt(square(dx) + square(dy));

  // Subtract the drop_early distance
  QLength total_distance = translational_distance - drop_early;

  return total_distance;
}

}  // namespace rev

#endif