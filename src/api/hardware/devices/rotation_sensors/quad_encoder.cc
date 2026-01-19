#include "rev/api/hardware/devices/rotation_sensors/quad_encoder.hh"

namespace rev {

QuadEncoder::QuadEncoder(char top,
                         char bottom)
    : sensor(top, bottom, false), ports(std::make_pair(top, bottom)) {}

QuadEncoder::QuadEncoder(char top,
                         char bottom,
                         const bool reverse_flag = false)
    : sensor(static_cast<int>(top), static_cast<int>(bottom), reverse_flag), ports(std::make_pair(top, bottom)) {}

double QuadEncoder::get_position() {
  return (double)sensor.get_value() * 360.0 / 8192.0;
}

std::pair<std::uint8_t, std::uint8_t> QuadEncoder::check_ports() {
  if (sensor.reset() == PROS_ERR) {
    return std::make_pair(static_cast<uint8_t>(ports.first), static_cast<uint8_t>(ports.second));
  }
  return std::make_pair(0, 0);
}

}  // namespace rev