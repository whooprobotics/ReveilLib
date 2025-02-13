#pragma once

#include <memory>
#include <cstdint>
#include "pros/adi.hpp"
#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {
/**
 * Wrapper class for the pros::ADIEncoder class. This class exists to extend the TwoRotationInertialOdometry algorithms (both linear & 45 degree) to work with quadrature encoders.
 */
class QuadEncoder : public ReadOnlyRotarySensor {
  public:
    QuadEncoder(std::uint8_t top, std::uint8_t bottom, const bool reverse_flag);
    double get_position() override; // Returns degrees the sensor has rotated
  private:
    pros::ADIEncoder sensor;
};

} // namespace rev