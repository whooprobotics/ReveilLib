#include "rev/api/hardware/devices/mock_quad_encoder.hh"

namespace rev {

MockQuadEncoder::MockQuadEncoder(int initial_reading) : value(initial_reading) {}

double MockQuadEncoder::get_position() {
  return (double) value * 360.0 / 8192.0;
}

int MockQuadEncoder::get_value() {
  return value;
}

void MockQuadEncoder::increment() {
  if (value == 8191) {
    value = 0;
    return;
  }

  value++;
}

void MockQuadEncoder::decrement() {
  if (value == 0) {
    value = 8191;
    return;
  }
  value--;
}

} // namespace rev