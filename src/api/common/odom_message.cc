#include "rev/api/common/msg/message.hh"

namespace rev {

OdomMessage::OdomMessage(double ix, double iy, double itheta) :
      x(ix), y(iy), theta(itheta) {
  type = MessageType::ODOM;
}

OdomMessage::OdomMessage() : OdomMessage(0, 0, 0) {
  type = MessageType::ODOM;
}

uint32_t OdomMessage::get_size() {
  return 3 * sizeof(double);
}

void OdomMessage::serialize(uint8_t* buffer) {
  double data[3] = {x, y, theta};
  std::memcpy(buffer, &data, sizeof(data));
}

void OdomMessage::deserialize(uint8_t* buffer) {
  double data[3];
  std::memcpy(&data, buffer, sizeof(data));
  x = data[0];
  y = data[1];
  theta = data[2];
}

void OdomMessage::set_position(double ix, double iy, double itheta) {
  x = ix;
  y = iy;
  theta = itheta;
}

}  // namespace rev