#pragma once

#include <cstdint>
#include <memory>
#include "pros/adi.hpp"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {

class QuadEncoder : public ReadOnlyRotarySensor {
 public:
  QuadEncoder(std::uint8_t top, std::uint8_t bottom, const bool reverse_flag);

  double get_position() override;

 private:
  pros::ADIEncoder sensor;
};

}  // namespace rev