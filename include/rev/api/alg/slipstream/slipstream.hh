#pragma once

#include <memory>
#include "rev/api/alg/PID/PID.hh"
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/slipstream/path.hh"
#include "rev/api/async/async_awaitable.hh"
#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/chassis/holonomic_chassis.hh"

namespace rev {

// This holds all the default constants for the PID slipstream segments
struct Constants {
  // Drive PID
  double drive_kp;
  double drive_ki;
  double drive_kd;
  double drive_starti;

  // Drive Exiting
  double drive_settle_error;
  QTime drive_settle_time;
  double drive_large_settle_error;
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
  double turn_settle_error;
  QTime turn_settle_time;
  double turn_large_settle_error;
  QTime turn_large_settle_time;
  QTime turn_timeout;

  // Turn Motion Chaining
  QAngle turn_exit_error;
  double turn_min_speed;
  double turn_max_speed;

  // Center/Strafe max speed (used by drive_to_point and drive_to_pose)
  double center_max_speed;
};

extern Constants constants;

inline bool is_line_settled(QLength desired_X,
                            QLength desired_Y,
                            QAngle desired_angle_deg,
                            QLength current_X,
                            QLength current_Y,
                            QLength exit_error) {
  return (desired_Y - current_Y) * cos(desired_angle_deg) <=
         -(desired_X - current_X) * sin(desired_angle_deg) + exit_error;
}

enum class TurnDirection { NONE, CLOCKWISE, COUNTERCLOCKWISE };

inline QAngle reduce_negative_180_to_180(QAngle angle) {
  angle = mod(angle + 180.0_deg, 360.0_deg);
  if (angle < 0_deg)
    angle += 360.0_deg;
  return angle - 180.0_deg;
}

inline QAngle angle_error(QAngle error, TurnDirection direction) {
  if (direction == TurnDirection::NONE)
    return reduce_negative_180_to_180(error);
  if (direction == TurnDirection::CLOCKWISE)
    return error < 0_deg ? error + 360.0_deg : error;
  if (direction == TurnDirection::COUNTERCLOCKWISE)
    return error > 0_deg ? error - 360.0_deg : error;
  return reduce_negative_180_to_180(error);
}

inline QAngle reduce_0_to_360(QAngle angle) {
  while (!(angle >= 0_deg && angle < 360.0_deg)) {
    if (angle < 0_deg) {
      angle += 360.0_deg;
    }
    if (angle >= 360.0_deg) {
      angle -= 360.0_deg;
    }
  }
  return angle;
}

inline double deadband(double input, double width) {
  if (std::abs(input) < width) {
    return 0;
  }
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

inline double reduce_negative_90_to_90(double angle) {
  while (!(angle >= -90.0 && angle < 90.0)) {
    if (angle < -90.0)
      angle += 180.0;
    if (angle >= 90.0)
      angle -= 180.0;
  }
  return angle;
}

inline double left_voltage_scaling(double drive_output, double heading_output) {
  double ratio = std::max(std::abs(drive_output + heading_output),
                          std::abs(drive_output - heading_output)) /
                 12.0;
  if (ratio > 1.0)
    return (drive_output + heading_output) / ratio;
  return drive_output + heading_output;
}

inline double right_voltage_scaling(double drive_output,
                                    double heading_output) {
  double ratio = std::max(std::abs(drive_output + heading_output),
                          std::abs(drive_output - heading_output)) /
                 12.0;
  if (ratio > 1.0)
    return (drive_output - heading_output) / ratio;
  return drive_output - heading_output;
}

enum class SlipstreamStatus { ACTIVE, DONE };

class Slipstream : public AsyncRunnable, public AsyncAwaitable {
 public:
  Slipstream(std::shared_ptr<HolonomicChassis> ichassis,
             std::shared_ptr<Odometry> iodometry);

  void step() override;

  void await() override;

  bool is_ready() override;

  void go(SlipstreamPath path);

  void go(std::initializer_list<std::shared_ptr<SlipstreamSegment>> path) {
    go(SlipstreamPath(path));
  }

  SlipstreamStatus get_status();

  double progress();

  bool is_completed();

  void breakout();

  void set_constants(Constants constants);

 private:
  std::shared_ptr<HolonomicChassis> chassis;
  std::shared_ptr<Odometry> odometry;
  SlipstreamPath current_path;
  SlipstreamStatus status{SlipstreamStatus::DONE};

  size_t current_segment{0};
  long long brake_start_time = -1;
  double partial_progress{-1.0};
};

struct Turn {
  PIDParams turn_k = {.p = constants.turn_kp,
                      .i = constants.turn_ki,
                      .d = constants.turn_kd,
                      .starti = constants.turn_starti};

  settleParams turn_settle = {
      .settle_error = constants.turn_settle_error,
      .settle_time = constants.turn_settle_time,
      .large_settle_error = constants.turn_large_settle_error,
      .large_settle_time = constants.turn_large_settle_time};

  double min_speed = constants.turn_min_speed;
  double max_speed = constants.turn_max_speed;
  
  QAngle exit_error = constants.turn_exit_error;
  QTime timeout = constants.turn_timeout;
  QAngle offset = 0_deg;
};

struct Drive {
  PIDParams drive_k = {.p = constants.drive_kp,
                       .i = constants.drive_ki,
                       .d = constants.drive_kd,
                       .starti = constants.drive_starti};
  PIDParams turn_k = {.p = constants.turn_kp,
                      .i = constants.turn_ki,
                      .d = constants.turn_kd,
                      .starti = constants.turn_starti};

  settleParams drive_settle = {
      .settle_error = constants.drive_settle_error,
      .settle_time = constants.drive_settle_time,
      .large_settle_error = constants.drive_large_settle_error,
      .large_settle_time = constants.drive_large_settle_time};
  settleParams turn_settle = {
      .settle_error = constants.turn_settle_error,
      .settle_time = constants.turn_settle_time,
      .large_settle_error = constants.turn_large_settle_error,
      .large_settle_time = constants.turn_large_settle_time};

  double max_speed = constants.drive_max_speed;
  double min_speed = constants.drive_min_speed;

  QLength exit_error = constants.drive_exit_error;
  double turn_max_speed = constants.turn_max_speed;
  double center_max_speed = constants.center_max_speed;
  QTime timeout = constants.drive_timeout;
};

}  // namespace rev