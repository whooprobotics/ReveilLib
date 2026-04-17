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
  RotationSensor(std::uint8_t port);
  RotationSensor(std::uint8_t port, const bool reverse_flag);

  double get_position() override;
  std::pair<uint8_t, uint8_t> check_port() override;

 private:
  pros::Rotation sensor;
  std::uint8_t port;
};

}  // namespace rev