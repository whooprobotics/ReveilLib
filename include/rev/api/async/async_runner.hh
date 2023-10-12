#pragma once

#include <memory>
#include "pros/rtos.hpp"
#include "rev/api/async/async_runnable.hh"
namespace rev {
class AsyncRunner {
 public:
  AsyncRunner(std::shared_ptr<AsyncRunnable> icontroller,
              uint32_t itdelta = 10);

 private:
  std::shared_ptr<AsyncRunnable> controller;
  pros::Task thread;
  uint32_t tdelta;  // Time to wait between iterations in millis

  // Helper function to launch thread
  static void run(void* context);

  void loop();
};
}  // namespace rev