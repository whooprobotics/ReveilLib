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
[[deprecated("Replaced by AsyncRunnable.start_thread()")]]
class AsyncRunner {
 public:
  AsyncRunner(std::shared_ptr<AsyncRunnable> icontroller,
              uint32_t itdelta = 10);

  ~AsyncRunner();
};
}  // namespace rev