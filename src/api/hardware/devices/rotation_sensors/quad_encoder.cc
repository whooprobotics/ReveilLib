#include "rev/api/hardware/devices/rotation_sensors/quad_encoder.hh"

namespace rev {

QuadEncoder::QuadEncoder(std::uint8_t top, std::uint8_t bottom, const bool reverse_flag = false) {
  sensor = std::make_shared<pros::ADIEncoder>(top, bottom, reverse_flag);
}

double QuadEncoder::get_position() {
  if (previous_ticks > 7500 && sensor->get_value() < 1000) {
    looparounds++;
  } else if (previous_ticks < 1000 && sensor->get_value() > 7500) {
    looparounds--;
  }
  
  return (double) (sensor->get_value() * 360.0 / 8192.0) + (looparounds * 360.0);
}

} // namespace rev