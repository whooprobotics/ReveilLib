#include "rev/api/async/async_runner.hh"

namespace rev {
AsyncRunner::AsyncRunner(std::shared_ptr<AsyncRunnable> icontroller,
                         uint32_t itdelta)
    : controller(icontroller), tdelta(itdelta), thread(nullptr) {
#ifndef OFF_ROBOT_TESTS
  thread = new pros::Task(run, this, "ReveilLib Task");
#else
  thread = new std::thread(run, this);
#endif
}

AsyncRunner::~AsyncRunner() {
  // Exit the task when the runner is destroyed
  active = false;

  thread->join();
}

void AsyncRunner::run(void* context) {
  if (context)
    static_cast<AsyncRunner*>(context)->loop();
}

/**
 * @brief
 *
 * Don't leave this running for 49 straight days or it might crash... probably
 * not a bug thats worth fixing though due to the slight performance hit and the
 * fact that that will never happen.
 */

void AsyncRunner::loop() {
  uint32_t last_time = pros::millis();
  // Exit when this task gets notified
  while (active) {
    controller->step();

    // delay_until is more accurate for reasons
    pros::Task::delay_until(&last_time, tdelta);
  }
}
}  // namespace rev