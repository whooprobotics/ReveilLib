#pragma once

#include <ostream>
#include "rev/api/units/all_units.hh"

namespace rev {

struct PIDParams {
  double p;
  double i;
  double d;
  double starti;
};

inline std::ostream& operator<<(std::ostream& os, const PIDParams& k) {
  return os << "PIDParams{p=" << k.p << ", i=" << k.i
            << ", d=" << k.d << ", starti=" << k.starti << "}";
}

struct settleParams {
  double settle_error;
  QTime settle_time;
  double large_settle_error;
  QTime large_settle_time;
};

inline std::ostream& operator<<(std::ostream& os, const settleParams& s) {
  return os << "settleParams{settle_error=" << s.settle_error
            << ", settle_time=" << s.settle_time.convert(millisecond) << "ms"
            << ", large_settle_error=" << s.large_settle_error
            << ", large_settle_time=" << s.large_settle_time.convert(millisecond) << "ms}";
}

class PID {
public:
  PID();
  PID(double kp, double ki, double kd, double starti);
  PID(double kp, double ki, double kd, double starti, double settle_error, QTime settle_time, double large_settle_error, QTime large_settle_time, double exit_error, QTime timeout);

  double compute(double error, QTime dt = 10_ms);

  bool is_settled();

  void reset();

  double error = 0;
  double kp = 0;
  double ki = 0;
  double kd = 0;
  double starti = 0;
  double settle_error = 0;
  QTime settle_time = 0_ms;
  double large_settle_error = 0;
  QTime large_settle_time = 0_ms;
  double exit_error = 0;
  QTime timeout = 0_ms;
  double accumulated_error = 0;
  double previous_error = 0;
  double output = 0;
  QTime time_spent_settled = 0_ms;
  QTime time_spent_large_settled = 0_ms;
  QTime time_spent_running = 0_ms;
  bool exiting = false;
};

} // namespace rev


