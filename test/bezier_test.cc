#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/hardware/chassis_sim/driftless_sim.hh"
#include "rev/api/async/async_runner.hh"
#include "rev/rev.hh"
#include "rev/api/alg/path_gen/bezier_curves.hh"

TEST(Bezier, BezierVerifications){
  using namespace rev;
  auto sim = std::make_shared<DriftlessSim>(60_in / second, 200_rpm, 5_Hz, 5_Hz,
                                            20_Hz, 20_Hz);

  AsyncRunner simrunner1(sim);

  std::shared_ptr<rev::Reckless> reckless =
      std::make_shared<Reckless>(sim, sim);
  
  AsyncRunner reckless_runner(reckless);

  PointVector p1 {0_in, 0_in};
  PointVector p2 {2_in, 2_in};
  std::vector<PointVector> path_points {{0_in, 0_in}, {2_ft, 2_ft}};

  reckless->go(RecklessPath()
                   .with_segment(BezierSegment(path_points)));

  while (!reckless->is_completed()) {
    auto state = sim->get_state();
    std::cout << state.pos.x.convert(inch) << "in, "
              << state.pos.y.convert(inch) << "in" << std::endl;

    pros::delay(500);
  }
}