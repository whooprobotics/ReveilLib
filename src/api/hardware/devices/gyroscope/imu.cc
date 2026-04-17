#include "rev/api/hardware/devices/gyroscope/imu.hh"
#include <cmath>
#include "pros/error.h"

namespace rev {

Imu::Imu(int port) : imu(port), port(port) {}

double Imu::get_heading() {
  return imu.get_heading();
}

bool Imu::is_calibrating() {
  return imu.is_calibrating();
}

std::pair<std::uint8_t, std::uint8_t> Imu::check_port() {
  uint8_t p = imu.get_heading() == PROS_ERR_F ? port : 0;
  return std::make_pair(p, 0);
}

void Imu::calibrate() {
  imu.reset(true);
}

}  // namespace rev