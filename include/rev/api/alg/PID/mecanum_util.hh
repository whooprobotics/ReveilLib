#pragma once

#include "pros/rtos.hpp"
#include "mecanum_units.hh"
#include <cmath>

// Header only util file since the constants struct is in the header

struct Constants {
  // Drive PID
  double drive_kp;
  double drive_ki;
  double drive_kd;
  double drive_starti;

  // Drive Exiting
  QLength drive_settle_error;
  QTime drive_settle_time;
  QLength drive_large_settle_error;
  QTime drive_large_settle_time;
  QTime drive_timeout;

  // Drive Motion Chaining
  QLength drive_exit_error;
  double drive_min_speed;
  double drive_max_speed;
  
  // Turn PID
  double turn_kp;
  double turn_ki;
  double turn_kd;
  double turn_starti;

  // Turn Exiting
  QAngle turn_settle_error;
  QTime turn_settle_time;
  QAngle turn_large_settle_error;
  QTime turn_large_settle_time;
  QTime turn_timeout;
  
  // Turn Motion Chaining
  QAngle turn_exit_error;
  double turn_min_speed;
  double turn_max_speed;
};

template <typename T>
constexpr T sign(T value) {
	return value < 0 ? -1 : 1;
}

inline double to_rad(double angle_deg) {
  return angle_deg * (M_PI / 180.0);
}

inline double to_deg(double angle_rad) {
  return (angle_rad * (180.0 / M_PI));
}

inline bool is_line_settled(double desired_X, double desired_Y, double desired_angle_deg, double current_X, double current_Y, QLength exit_error) {
  return (desired_Y - current_Y) * cos(to_rad(desired_angle_deg)) <= -(desired_X - current_X) * sin(to_rad(desired_angle_deg)) + exit_error.internal();
}

enum class TurnDirection {
  NONE,
  CLOCKWISE,
  COUNTERCLOCKWISE
};

inline double reduce_negative_180_to_180(double angle) {
  angle = fmod(angle + 180.0, 360.0);
  if (angle < 0) angle += 360.0;
  return angle - 180.0;
}

inline double angle_error(double error, TurnDirection direction) {
  if (direction == TurnDirection::NONE) return reduce_negative_180_to_180(error);
  if (direction == TurnDirection::CLOCKWISE) return error < 0 ? error + 360 : error;
  if (direction == TurnDirection::COUNTERCLOCKWISE) return error > 0 ? error - 360 : error;
  return reduce_negative_180_to_180(error);
}

inline double reduce_0_to_360(double angle) {
  while(!(angle >= 0 && angle < 360)) {
    if(angle < 0) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return angle;
}

inline double deadband(double input, double width){
  if (std::abs(input) < width) { return 0; }
  return input;
}

inline double clamp_min_voltage(double drive_output, double drive_min_voltage) {
  if (drive_output < 0 && drive_output > -drive_min_voltage) {
    return -drive_min_voltage;
  }
  if (drive_output > 0 && drive_output < drive_min_voltage) {
    return drive_min_voltage;
  }
  return drive_output;
}

inline double clamp(double input, double min, double max) {
  if (input > max) { return max; }
  if (input < min) { return min; }
  return input;
}