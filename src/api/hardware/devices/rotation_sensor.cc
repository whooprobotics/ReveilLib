#include "rev/api/hardware/devices/rotation_sensor.hh"

namespace rev {

RotationSensor::RotationSensor(pros::Rotation isensor) : sensor(isensor) {}

double RotationSensor::get_position() {
  return (double) sensor.get_position() / 100;
}
} // namespace rev