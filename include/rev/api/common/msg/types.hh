#pragma once

#include <cstring>
#include <cstdint>
#include "rev/util/math/pose.hh"

/*
 * The format of each message shall be defined with the following structure:
 * one HEADER character (8 bit binary)
 * the source (either BRAIN or RASPI) (2 chars)
 * the destination (either BRAIN or RASPI) (2 chars)
 * the message type (8 bit binary)
 * the message
 * checksum length (later)
 * checksum (later)
 * one FOOTER character (8 bit binary)
*/
#define HEADER 0x00
#define FOOTER 0xAA
#define BRAIN "V5"
#define RASPI "PI"

#define INIT_MESSAGE_SIZE sizeof(uint8_t)
#define AUTO_MESSAGE_SIZE sizeof(uint8_t)
#define ODOM_MESSAGE_SIZE 3 * sizeof(double)

#define TOTAL_MAX_MESSAGE_SIZE 24 + ODOM_MESSAGE_SIZE

namespace rev {

/**
 * @brief Enum type for each type of message
 */
enum class MessageType : uint8_t {
  INIT = 0,
  AUTO = 1,
  DRVR = 2,
  ODOM = 3,
  TERM = 4 
};

/**
 * @brief Enum type for each type of auton route
 */
enum class AutonRoute : uint8_t {
  RED_QUAL = 0,
  RED_ELIM = 1,
  BLUE_QUAL = 2,
  BLUE_ELIM = 3,
  SKLS = 4,
  DRVR = 5 
};

}  // namespace rev