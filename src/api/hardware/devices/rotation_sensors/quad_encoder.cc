#include "rev/api/hardware/devices/rotation_sensors/quad_encoder.hh"

namespace rev {

QuadEncoder::QuadEncoder(std::uint8_t top, std::uint8_t bottom, const bool reverse_flag = false) : sensor(top, bottom, reverse_flag) {}

double QuadEncoder::get_position() {  
  return (double) (sensor.get_value() * 360.0 / 8192.0);
}

} // namespace rev