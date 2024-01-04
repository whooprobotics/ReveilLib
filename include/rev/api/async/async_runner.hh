#pragma once

#include <memory>
#ifndef OFF_ROBOT_TESTS
#include "pros/apix.h"
typedef pros::Task rthread;
#else
#include <thread>
#include "pros/rtos.hpp"
typedef std::thread rthread;
#endif
#include "rev/api/async/async_runnable.hh"
namespace rev {
/**
 * @brief Thread runner for AsyncRunnable
 *
 */
class AsyncRunner {
 public:
  AsyncRunner(std::shared_ptr<AsyncRunnable> icontroller,
              uint32_t itdelta = 10);

  ~AsyncRunner();

 private:
  std::shared_ptr<AsyncRunnable> controller;
  rthread* thread;
  uint32_t tdelta;  // Time to wait between iterations in millis

  bool active{true};

  // Helper function to launch thread
  static void run(void* context);

  void loop();
};
}  // namespace rev