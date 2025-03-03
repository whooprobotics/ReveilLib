#pragma once

#include "pros/rotation.hpp"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {
/**
 * @brief Implementation of the Vex Rotation sensor under the
 * ReadOnlyRotarySensor interface
 *
 */

class RotationSensor : public ReadOnlyRotarySensor {
 public:
  RotationSensor(std::uint8_t port, const bool reverse_flag);

  double get_position() override;

 private:
  pros::Rotation sensor;
};

}  // namespace rev