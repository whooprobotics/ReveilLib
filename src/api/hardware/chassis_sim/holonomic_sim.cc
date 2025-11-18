#include "rev/api/hardware/chassis_sim/holonomic_sim.hh"
#include "pros/rtos.hpp"
#include <cmath>

namespace rev {

HolonomicSim::HolonomicSim(QSpeed iv_max,
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
      angular_brake_decay_rate(iangular_brake_decay_rate) {
  state.pos.x = 0_in;
  state.pos.y = 0_in;
  state.pos.theta = 0_deg;
  state.vel.xv = 0_mps;
  state.vel.yv = 0_mps;
  state.vel.angular = 0_rpm;
}

void HolonomicSim::drive_holonomic(SlipstreamPower power) {
  // Convert motor powers to movement components
  // This is a simplified mecanum drive kinematics
  forward_power = (power.front_left_forward + power.front_right_forward + power.rear_left_forward + power.rear_right_forward) / 4.0;
  strafe_power = (-power.front_left_forward + power.front_right_forward + power.rear_left_forward - power.rear_right_forward) / 4.0;
  yaw_power = (-power.front_left_forward + power.front_right_forward - power.rear_left_forward + power.rear_right_forward) / 4.0;
}

void HolonomicSim::drive_holonomic(double forward, double yaw, double strafe) {
  forward_power = forward;
  strafe_power = strafe;
  yaw_power = yaw;
}

void HolonomicSim::drive_tank(double left, double right) {
  // Convert tank drive to arcade drive
  drive_arcade((left + right) / 2.0, (right - left) / 2.0);
}

void HolonomicSim::drive_arcade(double forward, double yaw) {
  forward_power = forward;
  strafe_power = 0.0;
  yaw_power = yaw;
}

void HolonomicSim::set_brake_harsh() {
  use_harsh_brake_mode = true;
}

void HolonomicSim::set_brake_coast() {
  use_harsh_brake_mode = false;
}

void HolonomicSim::stop() {
  forward_power = 0.0;
  strafe_power = 0.0;
  yaw_power = 0.0;
}

OdometryState HolonomicSim::get_state() {
  return state;
}

void HolonomicSim::set_position(Position pos) {
  state.pos = pos;
}

void HolonomicSim::reset_position() {
  state.pos.x = 0_in;
  state.pos.y = 0_in;
  state.pos.theta = 0_deg;
}

void HolonomicSim::step() {
  // Get current time
  int32_t time = pros::millis();
  if (time_h == -1) {
    time_h = time;
    return;
  }

  // Calculate time delta
  int32_t dt_ms = time - time_h;
  time_h = time;
  QTime dt = dt_ms * millisecond;

  // Choose decay rates based on brake mode
  QFrequency linear_decay = use_harsh_brake_mode ? linear_brake_decay_rate : linear_decay_rate;
  QFrequency angular_decay = use_harsh_brake_mode ? angular_brake_decay_rate : angular_decay_rate;

  // Update velocities based on power inputs
  QSpeed target_vx = forward_power * v_max;
  QSpeed target_vy = strafe_power * v_max;
  QAngularSpeed target_w = yaw_power * w_max;

  // Apply first-order dynamics with decay
  double linear_alpha = 1.0 - std::exp(-linear_decay.convert(Hz) * dt.convert(second));
  double angular_alpha = 1.0 - std::exp(-angular_decay.convert(Hz) * dt.convert(second));

  vx_current = vx_current + linear_alpha * (target_vx - vx_current);
  vy_current = vy_current + linear_alpha * (target_vy - vy_current);
  w_current = w_current + angular_alpha * (target_w - w_current);

  // Update position using current velocities
  // Convert velocities to robot frame
  double theta_rad = state.pos.theta.convert(radian);
  double cos_theta = std::cos(theta_rad);
  double sin_theta = std::sin(theta_rad);

  // Transform velocities from robot frame to world frame
  QSpeed world_vx = vx_current * cos_theta - vy_current * sin_theta;
  QSpeed world_vy = vx_current * sin_theta + vy_current * cos_theta;

  // Update position
  state.pos.x = state.pos.x + world_vx * dt;
  state.pos.y = state.pos.y + world_vy * dt;
  state.pos.theta = state.pos.theta + w_current * dt;

  // Normalize theta to [-180, 180] degrees
  while (state.pos.theta > 180_deg) {
    state.pos.theta = state.pos.theta - 360_deg;
  }
  while (state.pos.theta < -180_deg) {
    state.pos.theta = state.pos.theta + 360_deg;
  }

  // Update velocity state (in world frame for odometry)
  state.vel.xv = world_vx;
  state.vel.yv = world_vy;
  state.vel.angular = w_current;
}

}  // namespace rev