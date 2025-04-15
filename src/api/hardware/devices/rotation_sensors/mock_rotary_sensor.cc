#include "rev/api/hardware/devices/rotation_sensors/mock_rotary_sensor.hh"

namespace rev {

MockRotarySensor::MockRotarySensor(int initial_reading)
    : value(initial_reading) {}

double MockRotarySensor::get_position() {
  return (double)value / 100;
}

int MockRotarySensor::get_value() {
  return value;
}

void MockRotarySensor::increment() {
  value++;
}

void MockRotarySensor::decrement() {
  value--;
}

std::pair<std::uint8_t, std::uint8_t> MockRotarySensor::check_port() {
  return std::make_pair(0, 0);
}

}  // namespace rev