#pragma once
namespace rev {
class AsyncRunnable {
 public:
  /**
   * @brief The step function for the runnable controller. This will be invoked
   * once every tdelta milliseconds.
   *
   */
  virtual void step() = 0;
};
}  // namespace rev