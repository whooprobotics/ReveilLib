#ifdef PLATFORM_BRAIN

#include <cmath>
#include "rev/api/v5/alg/PID/PID.hh"
#include "rev/util/mathutil.hh"

namespace rev {
PID::PID() {};

PID::PID(double kp, double ki, double kd, double starti)
    : kp(kp), ki(ki), kd(kd), starti(starti) {};

PID::PID(double kp,
         double ki,
         double kd,
         double starti,
         double settle_error,
         QTime settle_time,
         double large_settle_error,
         QTime large_settle_time,
         double exit_error,
         QTime timeout)
    : kp(kp),
      ki(ki),
      kd(kd),
      starti(starti),
      settle_error(settle_error),
      settle_time(settle_time),
      large_settle_error(large_settle_error),
      large_settle_time(large_settle_time),
      exit_error(exit_error),
      timeout(timeout) {};

double PID::compute(double error, QTime dt) {
  if (fabs(error) < starti) {
    accumulated_error += error;
  }
  if (sgn(error) != sgn(previous_error)) {
    accumulated_error = 0;
  }

  output = kp * error + ki * accumulated_error + kd * (error - previous_error);

  previous_error = error;

  if (fabs(error) < settle_error) {
    time_spent_settled += dt;
  } else {
    time_spent_settled = 0_ms;
  }

  if (fabs(error) < large_settle_error) {
    time_spent_large_settled += dt;
  } else {
    time_spent_large_settled = 0_ms;
  }

  if (fabs(error) < exit_error && exit_error != 0) {
    exiting = true;
  }

  time_spent_running += dt;

  return output;
}

bool PID::is_settled() {
  if (time_spent_running > timeout && timeout != 0_ms) {
    return true;
  }
  if (time_spent_settled > settle_time ||
      time_spent_large_settled > large_settle_time) {
    return true;
  }
  if (exiting) {
    exiting = false;
    return true;
  }
  return false;
}

void PID::reset() {
  error = 0;
  accumulated_error = 0;
  previous_error = 0;
  output = 0;
  time_spent_settled = 0_ms;
  time_spent_large_settled = 0_ms;
  time_spent_running = 0_ms;
  exiting = false;
}

}  // namespace rev

#endif