#pragma once

#include "pros/optical.hpp"

/**
 * @brief Implementation of a Vex optical sensor with port checking
 * 
 */
namespace rev {
class Optical : public pros::Optical {
 public:
  /**
   * @brief Constructs a new Optical sensor
   * 
   * @param port V5 smart port number
   */
  Optical(std::uint8_t port);

  /**
   * @brief Verifies that the sensor is properly connected to the port
   * 
   * @returns Port that is not connected, zero otherwise
   */
  std::uint8_t check_port(void);
 private:
  std::uint8_t _port;
};
}  // namespace rev