#pragma once
#include <utility>
#include <cstdint>

namespace rev {

class ReadOnlyRotarySensor {
  /**
   * @brief Interface for rotational sensors used for odometry
   *
   */
 public:
  virtual double get_position() = 0;
  virtual std::pair<std::uint8_t, std::uint8_t> check_port() = 0;
};

}  // namespace rev