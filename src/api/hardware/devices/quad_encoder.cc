#include "rev/api/hardware/devices/quad_encoder.hh"
#include <memory>

namespace rev {

QuadEncoder::QuadEncoder(pros::ADIEncoder isensor) : sensor(std::make_shared<pros::ADIEncoder>(isensor)) {}

double QuadEncoder::get_position() {
  return (double) sensor->get_value() * 360.0 / 8192.0;
}

} // namespace rev