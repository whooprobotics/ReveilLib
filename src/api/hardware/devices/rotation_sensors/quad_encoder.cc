#include "rev/api/hardware/devices/rotation_sensors/quad_encoder.hh"

namespace rev {

QuadEncoder::QuadEncoder(std::uint8_t top,
                         std::uint8_t bottom,
                         const bool reverse_flag = false)
    : sensor(top, bottom, reverse_flag), ports(std::make_pair(top, bottom)) {}

double QuadEncoder::get_position() {
  return (double)sensor.get_value() * 360.0 / 8192.0;
}

std::pair<uint8_t, uint8_t> QuadEncoder::check_port() {
  if (sensor.reset() == PROS_ERR) {
    return ports;
  }
  return std::make_pair(0, 0);
}

}  // namespace rev