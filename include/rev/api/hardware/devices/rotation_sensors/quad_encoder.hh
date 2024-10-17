#pragma once

#include <memory>
#include <cstdint>
#include "pros/adi.hpp"
#include "rev/api/hardware/devices/rotary_sensors.hh"

namespace rev {

class QuadEncoder : public ReadOnlyRotarySensor {
  public:
    QuadEncoder(std::uint8_t top, std::uint8_t bottom, bool reverse);

    double get_position() override;

  private:
    std::shared_ptr<pros::ADIEncoder> sensor;

};

} // namespace rev