#pragma once

#include <memory>
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

}  // namespace rev