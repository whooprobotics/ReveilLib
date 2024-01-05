/**
 * @file off_robot_utils.cc
 * @author your name (you@domain.com)
 * @brief This file is for reimplementing PROS utilities (mostly RTOS stuff) for
 * off-robot testing
 * @version 0.1
 * @date 2023-12-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <cstdint>
#include <thread>

#include "pros/rtos.hpp"

// begin stackoverflow code
// from
// https://stackoverflow.com/questions/2831841/how-to-get-the-time-in-milliseconds-in-c

typedef int64_t msec_t;

#if defined(__WIN32__)

#include <windows.h>

msec_t time_ms(void) {
  return timeGetTime();
}

#else

#include <sys/time.h>

msec_t time_ms(void) {
  struct timeval tv;
  gettimeofday(&tv, __null);
  return (msec_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

#endif

// end stackoverflow code

namespace pros {
long long TIME_INIT = -1;
uint32_t c::millis() {
  if (TIME_INIT == -1) {
    TIME_INIT = time_ms();
  }

  return time_ms() - TIME_INIT;
}

void c::delay(const uint32_t milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void task_delay_until(uint32_t* const prev_time, const uint32_t delta) {
  // This has to be atomic so it doesnt change as we execute, or we risk
  // introducing missed ticks
  uint32_t tstart = millis();
  // Adjust for time thats passed since the last time
  uint32_t ndelta = (delta + *prev_time) - tstart;

  if (ndelta > 0) {
    delay(ndelta);
    *prev_time += delta;
  } else {
    *prev_time = millis();
  }
}

void Task::delay_until(uint32_t* const prev_time, const uint32_t delta) {
  task_delay_until(prev_time, delta);
}
};  // namespace pros