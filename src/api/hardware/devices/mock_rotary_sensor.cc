#include "rev/api/hardware/devices/mock_rotary_sensor.hh"

namespace rev {

MockRotarySensor::MockRotarySensor(int initial_reading) : value(initial_reading) {}

double MockRotarySensor::get_position() {
  return (double) value / 100;
}

int MockRotarySensor::get_value() {
  return value;
}

void MockRotarySensor::increment() {
  if (value == 35999) {
    value = 0;
    return;
  }
  
  value++;
}

void MockRotarySensor::decrement() {
  if (value == 0) {
    value = 35999;
    return;
  }

  value--;
}

} // namespace rev