#include "rev/api/hardware/devices/rotation_sensors/mock_quad_encoder.hh"

namespace rev {

MockQuadEncoder::MockQuadEncoder(int initial_reading, int res) : value(initial_reading), resolution(res) {}

double MockQuadEncoder::get_position() {
  return (double) (value * 360.0 / resolution); // Converts ticks to degrees rotated
}

int MockQuadEncoder::get_value() {
  return value;
}

void MockQuadEncoder::increment() {
  value++;
}

void MockQuadEncoder::decrement() {
  value--;
}

} // namespace rev