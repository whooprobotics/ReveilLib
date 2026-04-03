#pragma once

class PID {
public:
  PID();
  PID(double kp, double ki, double kd, double starti);
  PID(double kp, double ki, double kd, double starti, double settle_error, double settle_time, double large_settle_error, double large_settle_time, double exit_error, double timeout);

  double compute(double error);

  bool is_settled();

  double error = 0;
  double kp = 0;
  double ki = 0;
  double kd = 0;
  double starti = 0;
  double settle_error = 0;
  double settle_time = 0;
  double large_settle_error = 0;
  double large_settle_time = 0;
  double exit_error = 0;
  double timeout = 0;
  double accumulated_error = 0;
  double previous_error = 0;
  double output = 0;
  double time_spent_settled = 0;
  double time_spent_large_settled = 0;
  double time_spent_running = 0;
  bool exiting = false;
};
