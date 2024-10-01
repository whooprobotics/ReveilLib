#include "rev/api/hardware/devices/quad_encoder.hh"

namespace rev {

QuadEncoder::QuadEncoder(std::uint8_t top, std::uint8_t bottom, bool reverse = false) {\
  sensor = std::make_shared<pros::ADIEncoder>(top, bottom, reverse);
}

double QuadEncoder::get_position() {
  return (double) sensor->get_value() * 360.0 / 8192.0;
}

} // namespace rev