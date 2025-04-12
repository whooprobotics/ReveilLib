#include "rev/api/hardware/devices/rotation_sensors/mock_quad_encoder.hh"

namespace rev {

MockQuadEncoder::MockQuadEncoder(int initial_reading)
    : value(initial_reading) {}

double MockQuadEncoder::get_position() {
  return (double)(value * 360.0 / 8192.0) + (looparounds * 360.0);
}

int MockQuadEncoder::get_value() {
  return value + (looparounds * 8192);
}

void MockQuadEncoder::increment() {
  if (value == 8191) {
    looparounds++;
    value = 0;
    return;
  }

  value++;
}

void MockQuadEncoder::decrement() {
  if (value == 0) {
    looparounds--;
    value = 8191;
    return;
  }
  value--;
}

int MockQuadEncoder::get_looparounds() {
  return looparounds;
}

std::pair<std::uint8_t, std::uint8_t> MockQuadEncoder::check_port() {
  return std::make_pair(0, 0);
}

}  // namespace rev