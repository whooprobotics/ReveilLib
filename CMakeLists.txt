cmake_minimum_required(VERSION 3.14)
project(ReveilLib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_compile_definitions(OFF_ROBOT_TESTS)

enable_testing()

include_directories(include)

# Add include files, source files, and test files
add_executable(ReveilLib
        
        src/api/async/async_runner.cc
        src/api/alg/drive/motion/cascading_motion.cc
        src/api/alg/drive/motion/proportional_motion.cc
        src/api/alg/drive/motion/constant_motion.cc
        src/api/alg/drive/turn/campbell_turn.cc
        src/api/alg/drive/stop/simple_stop.cc
        src/api/alg/drive/correction/pilons_correction.cc
        src/api/alg/drive/correction/no_correction.cc
        src/api/alg/reckless/reckless.cc
        src/api/hardware/chassis_sim/driftless_sim.cc
        test/template_test.cc
        test/off_robot_utils.cc
        test/driftless_sim_test.cc
        )

# Link against gtest
target_link_libraries(ReveilLib GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(ReveilLib)
