#include "rev/api/hardware/devices/gyroscope/imu.hh"
#include <cmath>
#include "pros/error.h"

namespace rev {

Imu::Imu(int port) : inertial(port), port(port) {}

double Imu::get_heading() {
  return inertial.get_heading();
}

bool Imu::is_calibrating() {
  return inertial.is_calibrating();
}

std::pair<std::uint8_t, std::uint8_t> Imu::check_port() {
  uint8_t p = inertial.get_heading() == PROS_ERR_F ? port : 0;
  return std::make_pair(p, 0);
}

}  // namespace rev