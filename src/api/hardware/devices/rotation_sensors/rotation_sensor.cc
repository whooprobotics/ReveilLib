#include "rev/api/hardware/devices/rotation_sensors/rotation_sensor.hh"

namespace rev {

RotationSensor::RotationSensor(std::uint8_t port,
                               const bool reverse_flag = false)
    : sensor(port, reverse_flag) {}

double RotationSensor::get_position() {
  return (double)sensor.get_position() / 100;
}

}  // namespace rev