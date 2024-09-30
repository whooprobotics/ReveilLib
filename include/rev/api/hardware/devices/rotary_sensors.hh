#pragma once
#include <cstdint>

namespace rev {

class ReadOnlyRotarySensor {
  /** 
   * @brief Interface for rotational sensors used for odometry
   *
   */
  public:
    virtual std::int32_t get_position() const = 0;

};

} // namespace rev