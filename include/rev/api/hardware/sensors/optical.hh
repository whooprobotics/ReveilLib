#pragma once
#include "pros/optical.hpp"

/**
 * Wrapper class for the pros::Optical class, implementing a
 * Vex optical sensor.
 */
namespace rev {
class Optical : public pros::Optical {
 public:
  Optical(std::uint8_t port);
  std::uint8_t check_port(void);
 private:
  std::uint8_t _port;
};

} // namespace rev