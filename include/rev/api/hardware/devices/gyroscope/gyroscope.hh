#pragma once

#include <utility>
#include <cstdint>

namespace rev {

/**
 * @brief Interface for IMU sensors used for odometry
 * 
 */
class Gyroscope {
 public:
  /**
   * @brief Gets the current heading of the sensor
   * @returns Current facing angle of the Gyroscope, normalized to [-180, 180]
   */
  virtual double get_heading() = 0;

  /**
   * @brief Checks if the Gyroscope is calibrating
   * 
   */
  virtual bool is_calibrating() = 0;

  /**
   * @brief Verifies that the sensor port(s) are properly connected
   * @returns Port(s) that are not properly connected, zeros otherwise
   */
  virtual std::pair<std::uint8_t, std::uint8_t> check_port() = 0;
};

}  // namespace rev