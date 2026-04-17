#include "rev/api/hardware/devices/gyroscope/dual_imu.hh"
#include "math.h" // done for the infinity definition in error.h
#include "pros/error.h"
namespace rev {

DualImu::DualImu(int port1, int port2) : inertial1(port1), inertial2(port2), port1(port1), port2(port2) {}

double DualImu::get_heading() {
  double h1 = inertial1.get_heading();
  double h2 = inertial2.get_heading();

  if (h1 == PROS_ERR_F && h2 == PROS_ERR_F) {
    return PROS_ERR_F;
  }
  if (h1 == PROS_ERR_F) return h2;
  if (h2 == PROS_ERR_F) return h1;

  if (std::abs(h1 - h2) < 180)
    return (h1 + h2) / 2;
  double avg = (h1 + h2 - 360) / 2;
  if (avg < 0)
    return avg + 360;
  return avg;
}

bool DualImu::is_calibrating() {
  return inertial1.is_calibrating() || inertial2.is_calibrating();
}

std::pair<uint8_t, uint8_t> DualImu::check_port() {
  uint8_t p1 = inertial1.get_heading() == PROS_ERR_F ? port1 : 0;
  uint8_t p2 = inertial2.get_heading() == PROS_ERR_F ? port2 : 0;

  return std::make_pair(p1, p2); 
}

}  // namespace rev