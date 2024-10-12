#pragma once

#include "rev/api/hardware/devices/rotary_sensors.hh"

namespace rev {
  
class MockRotarySensor {
  public:
    MockRotarySensor(int reading);
    double get_position();
    int get_value();
    void increment();
    void decrement();
  private:
    int value = 0;
};

} // namespace rev