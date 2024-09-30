#pragma once

#include <memory>
#include "pros/adi.hpp"
#include "rev/api/hardware/devices/rotary_sensors.hh"

namespace rev {

class QuadEncoder : public ReadOnlyRotarySensor {
  public:
    QuadEncoder(pros::ADIEncoder isensor);

    double get_position();

  private:
    std::shared_ptr<pros::ADIEncoder> sensor;

};

} // namespace rev