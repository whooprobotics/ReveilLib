#pragma once

#ifdef PLATFORM_BRAIN
#include "pros/adi.hpp"
#include "rev/api/v5/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {

class QuadEncoder : public ReadOnlyRotarySensor {
 public:
  QuadEncoder(char top, char bottom, const bool reverse_flag);

  double get_position() override;
  std::pair<uint8_t, uint8_t> check_port() override;

 private:
  pros::ADIEncoder sensor;
  std::pair<std::uint8_t, std::uint8_t> ports; // top, bottom
};

}  // namespace rev

#endif