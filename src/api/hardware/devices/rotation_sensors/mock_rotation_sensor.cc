#include "rev/api/hardware/devices/rotation_sensors/mock_rotation_sensor.hh"

namespace rev {

MockRotationSensor::MockRotationSensor(int initial_reading)
    : value(initial_reading) {}

double MockRotationSensor::get_position() {
  return (double)value / 100;
}

int MockRotationSensor::get_value() {
  return value;
}

void MockRotationSensor::increment() {
  value++;
}

void MockRotationSensor::decrement() {
  value--;
}

std::pair<std::uint8_t, std::uint8_t> MockRotationSensor::check_ports() {
  return std::make_pair(0, 0);
}

}  // namespace rev