#pragma once
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {

class MockImu : Gyroscope {
  public:
    MockImu(double initial_angle);
    double get_heading();
    void set_angle(double new_angle);
    bool is_calibrating();
  private:
    double angle = 0.0;
};

} // namespace rev