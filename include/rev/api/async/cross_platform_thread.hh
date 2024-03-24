#pragma once

/**
 * @file cross_platform_thread.hh
 * @author Drew Deaton (blake.deaton@tamu.edu)
 * @brief Cross-platform thread implementation, based on OkapiLib
 * @version 0.1
 * @date 2024-03-13
 * 
 * @copyright This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * 
 * This code is directly based on OkapiLib's CrossPlatformThread, located at
 * https://github.com/OkapiLib/OkapiLib/blob/master/include/okapi/api/coreProsAPI.hpp
 */

#ifndef OFF_ROBOT_TESTS
#include "pros/apix.h"
typedef pros::task_t BaseThread;
#else
#include <thread>
#include "pros/rtos.hpp"
typedef std::thread BaseThread;
#endif

namespace rev {

  /**
   * @brief Wrapper class for easing use of threads on Windows, Linux, and Vex V5
   * 
   */
  class CrossPlatformThread {
    public:

// On-robot constructor
#ifndef OFF_ROBOT_TESTS
    /**
     * @brief Construct a new CrossPlatformThread
     * 
     * @param ptr A pointer to the task function
     * @param params A pointer to an array of parameters for this thread
     * @param name The name of the thread (mostly for debug use)
     */
    CrossPlatformThread(void (*ptr)(void *),
                        void *params,
                        const char *const name = "ReveilLib Task") :
                        thread(pros::c::task_create(ptr, params, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, name)), name(name)               
    {};
#else
// Off-robot constructor
CrossPlatformThread(void (*ptr)(void *),
                        void *params,
                        const char *const name = "ReveilLib Task") :
                        thread(ptr, params), name(name)               
    {};
#endif

    ~CrossPlatformThread() {
#ifndef OFF_ROBOT_TESTS
                        if(pros::c::task_get_state(thread) != pros::E_TASK_STATE_DELETED)
                          pros::c::task_delete(thread);
#else
                        thread.join();
#endif  
    }

    private:
    BaseThread thread;
    std::string name;
  };
}