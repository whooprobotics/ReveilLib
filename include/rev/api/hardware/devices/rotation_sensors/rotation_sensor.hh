#pragma once

#include "pros/rotation.hpp"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {
/**
 * @brief Implementation of the Vex Rotation sensor under the
 * RotarySensor interface
 * 
 */
class RotationSensor : public RotarySensor {
 public:
  /**
   * @brief Constructs a new Vex Rotation sensor
   * 
   * @param port Smart port number
   */
  RotationSensor(std::uint8_t port);

  /**
   * @brief Constructs a new Vex Rotation sensor
   * 
   * @param port Smart port number
   * @param reverse_flag Reverses the direction of the encoder
   */
  RotationSensor(std::uint8_t port, const bool reverse_flag);

  /**
   * @brief Gets the current position of the sensor
   * 
   * @returns Degrees rotated from the zero position
   */
  double get_position() override;

  /**
   * @brief Verifies that the sensor port is properly connected
   * 
   * @returns A pair of <port, 0> for the port if not connected, zeros otherwise
   */
  std::pair<uint8_t, uint8_t> check_ports() override;

 private:
  pros::Rotation sensor;
  std::uint8_t port;
};
}  // namespace rev