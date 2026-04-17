#pragma once

namespace rev {
/**
 * @brief Interface for classes which have something that can be waited for
 *
 */
class AsyncAwaitable {
 public:
  /**
   * @brief Blocking method which returns when the awaited event has occured
   *
   */
  virtual void await() = 0;

  /**
   * @brief Non-blocking status indicator
   *
   */
  virtual bool is_ready() = 0;
};

}  // namespace rev