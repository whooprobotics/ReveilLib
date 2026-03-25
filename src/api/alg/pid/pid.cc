#include "rev/api/alg/pid/pid.hh"
#include <algorithm>

namespace rev {

PID::PID(double kp, double ki, double kd, double integral_limit)
    : kp(kp), ki(ki), kd(kd), integral_limit(integral_limit),
      integral(0), prev_error(0), prev_time(0) {}

double PID::update(double target, double current) {
  uint32_t now = pros::millis();
  double dt = (prev_time == 0) ? 0.02 : (now - prev_time) / 1000.0;
  prev_time = now;

  double error = target - current;

  integral += error * dt;
  integral = std::clamp(integral, -integral_limit, integral_limit);

  double derivative = (dt > 0) ? (error - prev_error) / dt : 0.0;
  prev_error = error;

  double output = kp * error + ki * integral + kd * derivative;
  return std::clamp(output, -1.0, 1.0);
}

void PID::reset() {
  integral = 0;
  prev_error = 0;
  prev_time = 0;
}

} // namespace rev
