#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/async/async_runner.hh"
#include "rev/api/units/all_units.hh"

/*
TEST(DriftlessSim, SimVerifications) {
  using namespace rev;
  auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 1_Hz, 1_Hz,
                                            4_Hz, 4_Hz);

  AsyncRunner simrunner1(sim);

  sim->drive_arcade(1, 0);

  for (int i = 0; i < 20; ++i) {
    auto state = sim->get_state();
    std::cout << state.pos.x.convert(inch) << "in, "
              << state.pos.y.convert(inch) << "in," << std::endl;

    // std::cout << state.vel.xv.convert(inch / second) << "in/s, "
    //           << state.vel.yv.convert(inch / second) << "in/s, " <<
    //           std::endl;
    pros::delay(100);
  }

  sim->set_brake_harsh();
  sim->stop();

  ASSERT_NEAR(sim->get_state().pos.x.convert(inch), 68.35, 0.015);

  std::cout << "\n\nStopping:" << std::endl;

  for (int i = 0; i < 20; ++i) {
    auto state = sim->get_state();
    std::cout << state.pos.x.convert(inch) << "in, "
              << state.pos.y.convert(inch) << "in" << std::endl;
    pros::delay(100);
  }

  ASSERT_NEAR(sim->get_state().pos.x.convert(inch), 81.08, 0.015);
}
*/