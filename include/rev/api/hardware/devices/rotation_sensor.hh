#pragma once

#include "pros/rotation.hpp"
#include "rev/api/hardware/devices/rotary_sensors.hh"

namespace rev {

class RotationSensor : public ReadOnlyRotarySensor {
  public:
    RotationSensor(pros::Rotation isensor);

    double get_position();

  private:
    pros::Rotation sensor;

};

} // namespace rev