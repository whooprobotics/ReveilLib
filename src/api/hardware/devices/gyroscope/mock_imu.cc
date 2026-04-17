#include "rev/api/hardware/devices/gyroscope/mock_imu.hh"

namespace rev {

MockImu::MockImu(double initial_angle) : angle(initial_angle) {}

double MockImu::get_heading() {
  return angle;
}

void MockImu::set_angle(double new_angle) {
  angle = new_angle;
}

bool MockImu::is_calibrating() {
  return false;
}

std::pair<uint8_t, uint8_t> MockImu::check_port() {
  return std::make_pair(0, 0);
}

}  // namespace rev