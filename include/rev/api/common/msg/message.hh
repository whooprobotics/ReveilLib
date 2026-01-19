#pragma once

#include "rev/api/common/msg/types.hh"

namespace rev {

/**
 * @brief Interface for messages
 */
struct Message {
  MessageType type;

  virtual uint32_t get_size() = 0;
  virtual void serialize(uint8_t* buffer) = 0;
  virtual void deserialize(uint8_t* buffer) = 0;
};

/**
 * @brief Message type containing odometry data
 */
struct OdomMessage : public Message {
  // DISTANCE UNITS ARE IN INCHES
  double x;
  double y;
  double theta; // ANGLE UNITS IN DEGREES

  OdomMessage();
  OdomMessage(double ix, double iy, double itheta);
  uint32_t get_size() override;
  void serialize(uint8_t* buffer) override;
  void deserialize(uint8_t* buffer) override;
  void set_position(double ix, double iy, double itheta);
};

/**
 * @brief Message to trigger frontend code initialization
 */
struct InitMessage : public Message {
  AutonRoute route;

  InitMessage(AutonRoute iroute);

  InitMessage();

  uint32_t get_size() override;
  void serialize(uint8_t* buffer) override;
  void deserialize(uint8_t* buffer) override;
};

}  // namespace rev