/**
 * @file off_robot_utils.cc
 * @author your name (you@domain.com)
 * @brief This file is for reimplementing PROS utilities (such as delay) for
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
namespace c {
// Implementation of pros::millis() for windows and unix
// This is relative to the first call, not to program initialization,
// unfortunately
long long TIME_INIT = -1;
// Needs to be unmangled to properly imitate pros
extern "C" uint32_t millis();
uint32_t millis() {
  if (TIME_INIT == -1) {
    TIME_INIT = time_ms();
  }

  return time_ms() - TIME_INIT;
}

extern "C" void delay(const uint32_t milliseconds);
void delay(const uint32_t milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
};  // namespace c
};  // namespace pros