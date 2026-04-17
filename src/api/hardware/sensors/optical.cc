#include "rev/api/hardware/sensors/optical.hh"
#include <cmath>
#include "pros/error.h"

rev::Optical::Optical(uint8_t port) : pros::Optical(port), _port(port) {}

uint8_t rev::Optical::check_port(void) {
  if (std::abs(get_hue()) == PROS_ERR)
    return get_port();
  return 0;
}