#pragma once

#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {

class MockQuadEncoder : public ReadOnlyRotarySensor {
 public:
  MockQuadEncoder(int initial_reading);
  double get_position() override;
  int get_value();
  void increment();
  void decrement();
  int get_looparounds();

 private:
  int value = 0;
  int looparounds = 0;
};
}  // namespace rev