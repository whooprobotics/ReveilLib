#include "rev/api/copro/msg/copro_message_node.hh"

#ifdef PLATFORM_RASPI

#include <unistd.h>
#include <fcntl.h>

using std::unique_ptr, std::vector, std::queue, std::make_unique;

namespace rev {

CoproMessageNode::CoproMessageNode() {
  memset(buffer, 0, sizeof(buffer));
  brain_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
}

CoproMessageNode::~CoproMessageNode() {
  close(brain_fd);
}

int8_t CoproMessageNode::write_message(unique_ptr<Message> message) {
  
}

void CoproMessageNode::parse_messages() {

}

vector<unique_ptr<Message>> CoproMessageNode::get_latest_message(MessageType message) {

}

int8_t CoproMessageNode::read_message() {

}

}  // namespace rev

#endif