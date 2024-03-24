#pragma once

#include "rev/api/async/cross_platform_thread.hh"

namespace rev {
/**
 * @brief Interface for classes which should have associated threads
 *
 */
class AsyncRunnable {
 public:
  /**
   * @brief The step function for the runnable controller. This will be invoked
   * once every tdelta milliseconds.
   *
   */
  virtual void step() = 0;

  /**
   * @brief This function runs to main loop of the controller
   * 
   * It is virtual so technically speaking it *can* be overridden, however the preferred use is to override step() instead
   * 
   */
  virtual void loop() {
    while(true) {
      step();
      pros::delay(10);
    }
  }

  static void start_loop(void* runnable) {
    if(runnable)
      static_cast<AsyncRunnable*>(runnable)->loop();
  }

  void start_thread() {
    if(!thread) {
      thread = new CrossPlatformThread(start_loop, this, task_name);
    }
  }

  AsyncRunnable() {
    
  }

  AsyncRunnable(const char* const thread_name = "Rev Async Controller") {
    task_name = thread_name;
  }

 protected:
  CrossPlatformThread* thread {nullptr};
  const char* task_name = "Rev Async Controller";
};
}  // namespace rev