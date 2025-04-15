#pragma once
#include "rev/api/hardware/devices/gyroscope/gyroscope.hh"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {

class MockImu : public Gyroscope {
 public:
  MockImu(double initial_angle);
  double get_heading();
  void set_angle(double new_angle);
  bool is_calibrating();
  std::pair<uint8_t, uint8_t> check_port();

 private:
  double angle = 0.0;
};

}  // namespace rev