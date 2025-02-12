#include "rev/api/hardware/devices/gyroscope/dual_imu.hh"

namespace rev {

DualImu::DualImu(int port1, int port2) : inertial1(port1), inertial2(port2) {}

double DualImu::get_heading() {
  return (inertial1.get_heading() + inertial2.get_heading()) / 2;
}

bool DualImu::is_calibrating() {
  return inertial1.is_calibrating() || inertial2.is_calibrating();
}


} // namespace rev