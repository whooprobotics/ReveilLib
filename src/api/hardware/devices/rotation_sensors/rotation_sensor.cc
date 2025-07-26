#include "rev/api/hardware/devices/rotation_sensors/rotation_sensor.hh"
#include <cmath>
#include "pros/error.h"

namespace rev {

RotationSensor::RotationSensor(std::uint8_t port) : sensor(port, false), port(port) {}

RotationSensor::RotationSensor(std::uint8_t port,
                               const bool reverse_flag = false)
    : sensor(port, reverse_flag), port(port) {}

// Returns the angle of the rotation sensor in degrees
double RotationSensor::get_position() {
  return (double)sensor.get_position() / 100;
}

std::pair<uint8_t, uint8_t> RotationSensor::check_port() {
  if (sensor.get_position() == PROS_ERR)
    return std::make_pair(port, 0);
  return std::make_pair(0, 0);
}

}  // namespace rev