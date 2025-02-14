#pragma once

namespace rev {

class ReadOnlyRotarySensor {
  /**
   * @brief Interface for rotational sensors used for odometry
   *
   */
 public:
  virtual double get_position() = 0;
};

}  // namespace rev