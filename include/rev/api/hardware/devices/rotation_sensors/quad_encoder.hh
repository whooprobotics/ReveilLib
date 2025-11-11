#pragma once

#include "pros/adi.hpp"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {

/**
 * @brief Implementation of the Rev Robotics Through-Bore Encoder under the RotarySensor interface
 * 
 */
class QuadEncoder : public RotarySensor {
 public:
  /**
   * @brief Constructs a new Rev Quadrature encoder
   * 
   * @param top ADI port number, static_cast<uint8_t> from a char
   * @param bottom ADI port number, static_cast<uint8_t> from a char
   */
  QuadEncoder(std::uint8_t top, std::uint8_t bottom);

  /**
   * @brief Constructs a new Rev Quadrature encoder
   * 
   * @param top ADI port number, static_cast<uint8_t> from a char
   * @param bottom ADI port number, static_cast<uint8_t> from a char
   * @param reverse_flag Reverses the direction of the encoder
   */
  QuadEncoder(std::uint8_t top, std::uint8_t bottom, const bool reverse_flag);

  /**
   * @brief Gets the current position of the sensor
   * 
   * @returns Degrees rotated from the zero position
   */
  double get_position() override;

  /**
   * @brief Verifies the sensor ports are properly connected
   * 
   * @returns A pair <port, port> if either port is not connected, zeros otherwise
   */
  std::pair<uint8_t, uint8_t> check_ports() override;

 private:
  pros::ADIEncoder sensor;
  std::pair<std::uint8_t, std::uint8_t> ports; // top, bottom
};
} // namespace rev