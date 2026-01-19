#pragma once

#ifdef PLATFORM_RASPI

#include "rev/api/common/msg/message_node.hh"

namespace rev {

class CoproMessageNode : public MessageNode {
 public:
  CoproMessageNode();
  ~CoproMessageNode();
  int8_t write_message(std::unique_ptr<Message> message) override;
  void parse_messages() override;
  std::vector<std::unique_ptr<Message>> get_latest_message(MessageType type) override;

 private:
  int8_t read_message() override;

  uint8_t buffer[32 * TOTAL_MAX_MESSAGE_SIZE];
  size_t buffer_index = 0;
  std::queue<std::unique_ptr<Message>> available_messages[5];
  int brain_fd;
};

}  // namespace rev

#endif