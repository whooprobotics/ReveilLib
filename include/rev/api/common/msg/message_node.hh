#pragma once

#include <memory>
#include <queue>
#include <vector>
#include "rev/api/common/msg/types.hh"
#include "rev/api/common/msg/message.hh"

namespace rev {

// so here's the idea
// on each device, create a message node object (probably a shared pointer)
// because IO on the brain/Raspi work differently, each device will have its own implementation
// so for example let's call the BrainMessageNode b_node and the RaspiMessageNode p_node
// a message node runs as an std::thread (or pros::Task on the brain)
// the target function of the thread/task is parse_messages, so this will run in the background
// as robot control and odom and stuff happens

// when we want to send a message, we simply call b_node->write_message() or p_node->write_message()

// when we want to check for messages, we call b_node->check_messages() or p_node->check_messages()
// on the MessageType of message we are looking for
// so an odometry algorithm running on the RasPi will call p_node->check_messages(MessageType::ODOM)

// since the buffer can hold up to 32 messages, check_messages will return a vector of messages
// since we want whatever needs to read the messages to have as much up-to-date information as it can have
// this also helps prevent the buffer from getting filled up too much
// messages will be sorted in the vector chronologically from oldest (index 0) to newest just by proxy of how parse_messages works

// parse_messages will have an infinite while loop that continually reads data into the buffer & assembles messages
// parse_messages calls read_message which reads data from the USB into the buffer
// then, parse_messages checks if there is a complete message in the buffer
  // this has to be done since any given read from the USB connection doesn't guarantee that all data from one message will be there
  // i.e. it's possible for the brain to read data in from USB while the pi is writing, so it won't read all data from that message in
  // we need to store the data in a buffer until all of the data for that particular message has been read in
// so once each loop, it will check if the buffer contains a complete message.
// if the buffer contains a complete message, it will assemble the data into a shared_ptr<Message>, put that into available_messages, and clear it from the buffer

class MessageNode {
 public:
  virtual int8_t write_message(std::unique_ptr<Message> message) = 0;
  virtual void parse_messages() = 0;
  virtual std::vector<std::unique_ptr<Message>> get_latest_message(MessageType type) = 0;

 private:
  virtual int8_t read_message() = 0;

  // unsigned char buffer[32 * TOTAL_MAX_MESSAGE_SIZE];
  // unsigned char* buffer_position = buffer;
  // std::queue<std::unique_ptr<Message>> available_messages[];
};

}  // namespace rev