#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include <cmath>
#include "pros/rtos.hpp"

namespace rev {
DriftlessSim::DriftlessSim(QSpeed iv_max,
                           QAngularSpeed iw_max,
                           QFrequency ilinear_decay_rate,
                           QFrequency iangular_decay_rate,
                           QFrequency ilinear_brake_decay_rate,
                           QFrequency iangular_brake_decay_rate)
    : v_max(iv_max),
      w_max(iw_max),
      linear_decay_rate(ilinear_decay_rate),
      angular_decay_rate(iangular_decay_rate),
      linear_brake_decay_rate(ilinear_brake_decay_rate),
      angular_brake_decay_rate(iangular_brake_decay_rate) {}

void DriftlessSim::drive_tank(double left, double right) {
  // If a power exceeds 1.0, scale both accordingly
  double maxp = std::max(fabs(left), fabs(right));
  if (maxp > 1.0) {
    left /= maxp;
    right /= maxp;
  }

  linear_power = (left + right) / 2;
  angular_power = (left - right) / 2;
}

void DriftlessSim::drive_arcade(double forward, double yaw) {
  // If the 2 powers combine to exceed 1.0, scale both accordingly
  double totp = fabs(forward) + fabs(yaw);
  if (totp > 1.0) {
    forward /= totp;
    yaw /= totp;
  }

  linear_power = forward;
  angular_power = yaw;
}

void DriftlessSim::set_brake_harsh() {
  use_harsh_brake_mode = true;
}

void DriftlessSim::set_brake_coast() {
  use_harsh_brake_mode = false;
}

void DriftlessSim::stop() {
  drive_arcade(0.0, 0.0);
}

OdometryState DriftlessSim::get_state() {
  return state;
}

void DriftlessSim::set_position(Position pos) {
  state.pos = pos;
}

void DriftlessSim::reset_position() {
  set_position({0.0_in, 0.0_in, 0.0_deg});
}
void DriftlessSim::step() {
  // Find speeds we're approaching
  QSpeed v_target = v_max * linear_power;
  QAngularSpeed w_target = w_max * angular_power;

  // Find time change
  int32_t time_now = pros::millis();
  QTime dt = (time_now - time_h) * millisecond;
  time_h = time_now;

  // Calculate post-iteration stuff
  QSpeed v_after;
  QAngularSpeed w_after;

  // Handle harsh braking
  if (fabs(linear_power) < 0.005 && fabs(angular_power) < 0.005 &&
      use_harsh_brake_mode) {
    v_after = v_current - v_current * dt * linear_brake_decay_rate;
    w_after = w_current - w_current * dt * angular_brake_decay_rate;
  }
  // Handle everything else
  else {
    v_after = v_current + (v_target - v_current) * dt * linear_decay_rate;
    w_after = w_current + (w_target - w_current) * dt * angular_decay_rate;
  }

  // calculate facing delta
  // we will assume we can find this by integrating the average of the starting
  // w and the ending w, because a more complex integral is hard
  QAngle facing_change = (w_after + w_current) / 2 * dt;
  QLength distance_traveled_arc = (v_after + v_current) / 2 * dt;

  // Chord length = l*sinc(df/2)
  QLength distance_traveled_line;
  if (facing_change == 0_deg)
    distance_traveled_line = distance_traveled_arc;
  else
    distance_traveled_line = 2 * distance_traveled_arc / facing_change *
                             radian * sin(facing_change / 2);

  // Average travel angle
  QAngle travel_angle = state.pos.facing + facing_change / 2;

  QLength dX = distance_traveled_line * cos(travel_angle);
  QLength dY = distance_traveled_line * sin(travel_angle);

  // Update positions
  state.pos.x += dX;
  state.pos.y += dY;
  state.pos.facing += facing_change;

  // Update velocities
  v_current = v_after;
  w_current = w_after;
  state.vel.angular = w_after;
  state.vel.xv = v_after * cos(travel_angle);
  state.vel.yv = v_after * sin(travel_angle);
}
}  // namespace rev