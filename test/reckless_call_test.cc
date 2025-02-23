#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/alg/reckless/call.hh"
#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include "rev/rev.hh"

bool status_flag = false;
void set_flag() {
  status_flag = true;
}

/*

TEST(Call, RecklessVerifications) {
  using namespace rev;
  auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
                                            20_Hz, 20_Hz);

  AsyncRunner simrunner1(sim);

  std::shared_ptr<rev::Reckless> reckless =
      std::make_shared<Reckless>(sim, sim);

  AsyncRunner reckless_runner(reckless);

  const double kP = 0.0;
  const double kB = 0.015;

  reckless->go(RecklessPath().with_segment(rev::Call(set_flag)));

  while (!reckless->is_completed()) {
    pros::delay(500);
  }

  EXPECT_TRUE(status_flag);
}

*/