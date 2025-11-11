#pragma once
#include <utility>
#include <cstdint>

namespace rev {

/**
 * @brief Interface for rotational sensors used for odometry
 *
 */
class RotarySensor {
 public:
  /**
   * @brief Gets the current position of the sensor
   * @returns Degrees rotated from the zero position 
   */
  virtual double get_position() = 0;

  /**
   * @brief Verifies that the sensor port(s) are properly connected
   * @returns Port(s) that are not properly connected, zeros otherwise
   */
  virtual std::pair<std::uint8_t, std::uint8_t> check_ports() = 0;
};

}  // namespace rev