#include "rev/api/hardware/devices/gyroscope/imu.hh"

namespace rev {

Imu::Imu(int port) : inertial(port) {}

double Imu::get_heading() {
  return inertial.get_heading();
}

bool Imu::is_calibrating() {
  return inertial.is_calibrating();
}

}  // namespace rev