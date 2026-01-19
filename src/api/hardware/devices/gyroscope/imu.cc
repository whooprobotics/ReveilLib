#include <cmath>
#include "pros/error.h"
#include "rev/api/hardware/devices/gyroscope/imu.hh"

using std::pair, std::make_pair;

namespace rev {

Imu::Imu(int port) : inertial(port), port(port) {}

double Imu::get_heading() {
  return inertial.get_heading();
}

bool Imu::is_calibrating() {
  return inertial.is_calibrating();
}

pair<uint8_t, uint8_t> Imu::check_ports() {
  uint8_t p = inertial.get_heading() == PROS_ERR_F ? port : 0;
  return make_pair(p, 0);
}

}  // namespace rev