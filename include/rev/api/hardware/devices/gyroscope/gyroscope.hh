#pragma once

#include <utility>
#include <cstdint>

namespace rev {

class Gyroscope {
 public:
  virtual double get_heading() = 0;
  virtual bool is_calibrating() = 0;
  virtual std::pair<std::uint8_t, std::uint8_t> check_port() = 0;
};

}  // namespace rev