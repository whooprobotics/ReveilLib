#pragma once
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {

class MockRotarySensor : public ReadOnlyRotarySensor {
 public:
  MockRotarySensor(int reading);
  double get_position() override;
  std::pair<std::uint8_t, std::uint8_t> check_port() override;
  int get_value();
  void increment();
  void decrement();

 private:
  int value = 0;
};

}  // namespace rev