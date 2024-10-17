#pragma once

#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {
  
class MockQuadEncoder {
  public:
    MockQuadEncoder(int initial_reading);
    double get_position();
    int get_value();
    void increment();
    void decrement();
  private:
    int value = 0;
};

} // namespace rev