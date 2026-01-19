#include "rev/api/common/msg/message.hh"

namespace rev {

InitMessage::InitMessage(AutonRoute iroute) :
      route(iroute) {
  type = MessageType::INIT;
}

InitMessage::InitMessage() : InitMessage(AutonRoute::DRVR) {
  type = MessageType::INIT;
}

uint32_t InitMessage::get_size() {
  return sizeof(AutonRoute);
}

void InitMessage::serialize(uint8_t* buffer) {
  std::memcpy(buffer, &route, sizeof(AutonRoute));
}

void InitMessage::deserialize(uint8_t* buffer) {
  std::memcpy(&route, buffer, sizeof(AutonRoute));
}

}  // namespace rev