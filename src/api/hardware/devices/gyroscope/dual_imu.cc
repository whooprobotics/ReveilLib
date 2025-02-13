#include "rev/api/hardware/devices/gyroscope/dual_imu.hh"

namespace rev {

DualImu::DualImu(int port1, int port2) : inertial1(port1), inertial2(port2) {}

double DualImu::get_heading() {
  double h1 = inertial1.get_heading();
  double h2 = inertial2.get_heading();
  if (std::abs(h1-h2) < 180){return (h1+h2)/2;}
  double avg = ((std::max(h1,h2)-360)+(std::min(h1,h2)))/2;
  if (avg < 0){return avg + 360;}
  return avg;
}

bool DualImu::is_calibrating() {
  return inertial1.is_calibrating() || inertial2.is_calibrating();
}


} // namespace rev