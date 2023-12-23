#pragma once

#include <memory>
#ifndef OFF_ROBOT_TESTS
#include "pros/apix.h"
typedef pros::Task revthread;
#else
#include <thread>
typedef std::thread revthread;
#endif
#include "rev/api/async/async_runnable.hh"
namespace rev {
class AsyncRunner {
 public:
  AsyncRunner(std::shared_ptr<AsyncRunnable> icontroller,
              uint32_t itdelta = 10);

  ~AsyncRunner();

 private:
  std::shared_ptr<AsyncRunnable> controller;
  revthread* thread;
  uint32_t tdelta;  // Time to wait between iterations in millis

  // Helper function to launch thread
  static void run(void* context);

  void loop();
};
}  // namespace rev