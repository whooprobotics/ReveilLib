#pragma once

#include <atomic>

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
    while(!deleting.load(std::memory_order_acquire) 
#ifndef OFF_ROBOT_TESTS
      && !pros::c::task_notify_take(true, 0)
#endif
    ) {
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
#ifndef OFF_ROBOT_TESTS
      pros::c::task_notify_when_deleting(pros::c::task_get_current(), thread, 1,
                               pros::E_NOTIFY_ACTION_INCR);
#endif
    }
  }

  AsyncRunnable() {
    
  }

  AsyncRunnable(const char* const thread_name = "Rev Async Controller") {
    task_name = thread_name;
  }

  ~AsyncRunnable() {
    deleting.store(true, std::memory_order_release);
    delete thread;
  }

 protected:
  CrossPlatformThread* thread {nullptr};
  
  std::atomic_bool deleting{false};
  const char* task_name = "Rev Async Controller";
};
}  // namespace rev